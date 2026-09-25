#include "tiptilt_application.h"
#include "imageprocess.h"
#include "controls.h"
#include "firstlight.h"
#include "NIDAQmx.h"
#include <atomic>
#include <cctype>
#include <cstdio>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <limits>
#include <memory>
#include <mutex>
#include <vector>
#include "motorcontrols.h"
#include "utilheaders.h"
#include "Butterworth.h"

using namespace std;

extern HANDLE XPort, YPort;
double A00, A01, A10, A11;
double ClM00, ClM01, ClM10, ClM11;
double Vxoff, Vyoff, SlewRate;
double Kp, Kd, Ki;
double Akp, Aki, Akd;
int autoGuiderCorrectionTime;
int imageSaveAfterSecond;
int Ni, Nd;
double AA00, AA01, AA10, AA11; // Autoguider control matrix
extern FliSdk* fli;
double autoGuiderOffloadLimitX, autoGuiderOffloadLimitY;
extern int Err;
double tau;

/* Output limits */
double limMin;
double limMax;

/* Integrator limits */
double limMinInt;
double limMaxInt;
double minKp, maxKp, minKd, maxKd;
double mp, cp, md, cd;

/* Sample time (in seconds) */
double sampleTime;
int liveView;
bool useCameraFlat;
string NucMode;
bool tipTiltReady = false, autoGuiderReady = false;
mutex tipTiltMutex, autoGuiderMutex;
std::condition_variable tipTiltConditionalVariable, autoGuiderConditionalVariable;
bool displayReady = false;
mutex displayMutex;
deque<double**> displayQueue;
std::condition_variable displayConditionalVariable;
uint64_t displayCount;
bool autoPMode;
Butterworth butterworth;
vector <Biquad> coeffs;  // second-order sections (sos)
BiquadChain derivativeFilterX;
BiquadChain derivativeFilterY;
double cutOffFrequencyOfDerivativeError;
std::atomic<bool> shutdownRequested{false};

struct TipTiltCommand {
    double xShift = 0;
    double yShift = 0;
    double xTemp = 0;
    double yTemp = 0;
    uint64_t currCount = 0;
    uint64_t nbImagesReceived = 0;
    uint64_t commandId = 0;
};

struct AutoGuiderCommand {
    int xCounts = 0;
    int yCounts = 0;
};

struct WorkerHandles {
    HANDLE tipTilt = nullptr;
    HANDLE autoguider = nullptr;
};

struct UiRuntimeState {
    bool loggingInitialized = false;
    bool cameraReady = false;
    bool tipTiltReady = false;
    bool autoguiderReady = false;
    bool closedLoopRunning = false;
    bool fftReady = false;
    std::unique_ptr<RawImageReceivedObserver> observer;
    WorkerHandles handles;
};

TipTiltCommand latestTipTiltCommand;
AutoGuiderCommand latestAutoGuiderCommand;
UiRuntimeState uiRuntimeState;
std::atomic<uint64_t> nextTipTiltCommandId{0};
std::atomic<uint64_t> latestPublishedTipTiltCommandId{0};
std::atomic<uint64_t> latestAppliedTipTiltCommandId{0};

struct CorrectionResult {
    double x = 0;
    double y = 0;
};

static void initializeLoggingAndCommandBuffer() {
    setupLogging(2);
    log_info("Allocating memory for various parameters");
    COMMAND[14] = 13; // CR
    COMMAND[15] = 10; // LF
}

static void ensureLoggingInitialized() {
    if (!uiRuntimeState.loggingInitialized) {
        initializeLoggingAndCommandBuffer();
        uiRuntimeState.loggingInitialized = true;
    }
}

static bool initializeImagingAndControlHardware() {
    getDarkFlat();
    Err = getCalibrationMatrix();

    log_info("Initializing devices and acquiring one-time data");
    Err = initDev(0, 0, NucMode);
    if (Err != 0) {
        return false;
    }

    Err = getHammingWindow();
    Err = initializeFFT();
    if (Err != 0) {
        stopDev();
        return false;
    }

    log_info("Legacy console startup path requested, but only the Qt workflow is supported.");
    Err = -1;
    return Err == 0;
}

static void configureDerivativeFilterIfNeeded() {
    if (Nd <= 0) {
        return;
    }

    int filterOrder = 8;
    double overallGain = 1;
    butterworth.loPass(
        fpsCamera,
        cutOffFrequencyOfDerivativeError,
        0,
        filterOrder,
        coeffs,
        overallGain
    );

    derivativeFilterX.allocate(coeffs.size());
    derivativeFilterX.reset();
    derivativeFilterY.allocate(coeffs.size());
    derivativeFilterY.reset();
}

static bool initializeImagePipeline() {
    if (!uiRuntimeState.fftReady) {
        getDarkFlat();
        Err = getCalibrationMatrix();
        Err = getHammingWindow();
        Err = initializeFFT();
        if (Err != 0) {
            return false;
        }
        uiRuntimeState.fftReady = true;
    }

    configureDerivativeFilterIfNeeded();
    return true;
}

static void moveActuatorsToOffsetAndInitializeDaq() {
    oplog_write("Moving actuators to offset point");
    sendCommand(XPort, "set,0,-20.0");
    sendCommand(YPort, "set,0,-20.0");
    Sleep(LONG_DELAY);
    sendCommand(XPort, "sr,0," + to_string(SlewRate));
    sendCommand(YPort, "sr,0," + to_string(SlewRate));

    log_info("Initializing DAQ");
    Err = initDAQ();
    setVoltagesXY(Vxoff, Vyoff);
}

static void enableAutoguiderHardware() {
    CreateControllerConnection(1);
    enableMotor(1);
    enableMotor(2);
    setMotorFrequency(1, 350);
    setMotorFrequency(2, 350);
}

static void enableAutoguiderHardware(int comPort, int motorFrequency) {
    CreateControllerConnection(comPort);
    enableMotor(1);
    enableMotor(2);
    setMotorFrequency(1, motorFrequency);
    setMotorFrequency(2, motorFrequency);
}

static void disableAutoguiderHardware() {
    exitMotor(1);
    disableMotor(1);
    exitMotor(2);
    disableMotor(2);
    closeControllerConnection();
}

static WorkerHandles startWorkerThreads(RawImageReceivedObserver& observer) {
    WorkerHandles handles;
    DWORD tipTiltThreadID, autoguiderThreadID;

    handles.tipTilt = CreateThread(0, 0, closedLoopVoltageThread, NULL, 0, &tipTiltThreadID);
    handles.autoguider = CreateThread(0, 0, closedLoopAutoGuiderThread, NULL, 0, &autoguiderThreadID);

    return handles;
}

static void stopWorkerThreads(const WorkerHandles& handles) {
    shutdownRequested = true;
    tipTiltConditionalVariable.notify_all();
    autoGuiderConditionalVariable.notify_all();
    displayConditionalVariable.notify_all();

    WaitForSingleObject(handles.tipTilt, INFINITE);
    WaitForSingleObject(handles.autoguider, INFINITE);
    CloseHandle(handles.tipTilt);
    CloseHandle(handles.autoguider);
}

static void resetUiRuntimeState() {
    uiRuntimeState.observer.reset();
    uiRuntimeState.handles = {};
    uiRuntimeState.cameraReady = false;
    uiRuntimeState.tipTiltReady = false;
    uiRuntimeState.autoguiderReady = false;
    uiRuntimeState.closedLoopRunning = false;
}

static void parkActuatorsAtOffset() {
    log_info("Actuator going to offset point");
    Err = setVoltagesXY(-20, -20);
}

static void shutdownHardware(int autoguiderMode) {
    if (autoguiderMode) {
        disableAutoguiderHardware();
    }

    parkActuatorsAtOffset();
    Err = closeDAQ();
    Err = closeXYSerialPorts();
    Err = stopDev();
}

static int runCalibrationAxis(const CalibrationUiSettings& settings, int axis) {
    constexpr int kActuatorMode = 0;
    constexpr int kMotorMode = 1;

    if (settings.frames <= 1 || settings.framesPerPosition <= 0) {
        return -10;
    }

    const std::string axisName = axis == 0 ? "X" : "Y";
    auto axisLog = axis == 0 ? xlog : ylog;
    auto shiftLog = axis == 0 ? xvoltagefilelog : yvoltagefilelog;
    const int motor = axis == 0 ? 2 : 1;

    axisLog(axisName + "-Calibration Log");
    axisLog("Number of frames are " + std::to_string(settings.frames));
    axisLog("Number of frames per calib position are " + std::to_string(settings.framesPerPosition));

    if (settings.mode == kMotorMode) {
        if (CreateControllerConnection(settings.motorControllerComPort) != 0) {
            return -11;
        }
        enableMotor(1);
        enableMotor(2);
        setMotorFrequency(1, settings.motorFrequency);
        setMotorFrequency(2, settings.motorFrequency);
    }

    XShift = axis == 0 ? settings.voltageStart : settings.stationaryVoltage;
    YShift = axis == 0 ? settings.stationaryVoltage : settings.voltageStart;
    FRAMENUMBER = 1;

    if (settings.mode == kActuatorMode) {
        setVoltagesXY(XShift, YShift);
    } else {
        setVoltagesXY(settings.stationaryVoltage, settings.stationaryVoltage);
    }

    Sleep(LONG_DELAY);

    auto t0 = chrono::high_resolution_clock::now();
    std::vector<uint16_t> binnedImage(NPIX, 0);
    uint16_t* image = reinterpret_cast<uint16_t*>(fli->getRawImage());
    if (image == nullptr) {
        return -12;
    }
    bin_separately(image, binnedImage.data());

    char logLine[1024];
    const std::string referenceFileName =
        std::string(SAVEPATH) + "\\" + axisName + "_frame_000001_Ref.dat";
    std::snprintf(logLine, sizeof(logLine), "Image acquired : %s", referenceFileName.c_str());
    axisLog(logLine);

    std::snprintf(logLine, sizeof(logLine), "%lf,%lf,0,0", XShift, YShift);
    shiftLog(logLine);

    double fx = settings.mode == kMotorMode ? settings.stationaryVoltage : XShift;
    double fy = settings.mode == kMotorMode ? settings.stationaryVoltage : YShift;
    int flatFrameIndice = get_flat_indice(fx, fy);

    if (processReferenceImage(binnedImage.data(), true, flatFrameIndice) != 0) {
        return -13;
    }

    double* currentImage = (double*)mkl_malloc(sizeof(double) * NX * NY, 64);
    fftw_complex* currentImageFT = (fftw_complex*)mkl_malloc(sizeof(fftw_complex) * NPIXFT, 64);
    fftw_complex* correlatedImageFT = (fftw_complex*)mkl_malloc(sizeof(fftw_complex) * NPIXFT, 64);
    double* correlatedImage = (double*)mkl_malloc(sizeof(double) * NX * NY, 64);
    if (currentImage == nullptr || currentImageFT == nullptr || correlatedImageFT == nullptr || correlatedImage == nullptr) {
        if (currentImage != nullptr) mkl_free(currentImage);
        if (currentImageFT != nullptr) mkl_free(currentImageFT);
        if (correlatedImageFT != nullptr) mkl_free(correlatedImageFT);
        if (correlatedImage != nullptr) mkl_free(correlatedImage);
        return -14;
    }

    if (axis == 0) {
        XShift += settings.voltageRange / settings.frames;
    } else {
        YShift += settings.voltageRange / settings.frames;
    }

    if (settings.mode == kActuatorMode) {
        setVoltagesXY(XShift, YShift);
    } else {
        setMotorCount(motor, sgn(settings.voltageRange / settings.frames), int(settings.voltageRange / settings.frames));
        const float64 timeToCompleteMovement = settings.voltageRange * 1000 / (settings.motorFrequency * settings.frames);
        Sleep(timeToCompleteMovement * 3);
    }

    uint64_t counter = 0;
    bool updateReference = false;
    uint32_t autoGuiderCounter = 600;
    bool autoGuiderHappening = true;
    uint64_t refreshInterval = 100;
    uint64_t numRefImage = 100;

    for (int frameIndex = 1; frameIndex < settings.frames; ++frameIndex) {
        double shiftX = 0.0;
        double shiftY = 0.0;
        char shiftPrefix[128];
        std::snprintf(shiftPrefix, sizeof(shiftPrefix), "%lf,%lf,", XShift, YShift);

        for (int sampleIndex = 0; sampleIndex < settings.framesPerPosition; ++sampleIndex) {
            image = reinterpret_cast<uint16_t*>(fli->getRawImage());
            if (image == nullptr) {
                mkl_free(currentImage);
                mkl_free(currentImageFT);
                mkl_free(correlatedImageFT);
                mkl_free(correlatedImage);
                return -15;
            }

            bin_separately(image, binnedImage.data());
            const std::string fileName =
                std::string(SAVEPATH) + "\\" + axisName + "_frame_" +
                std::string(6 - std::to_string(frameIndex + 1).length(), '0') + std::to_string(frameIndex + 1) + "_" +
                std::string(6 - std::to_string(sampleIndex + 1).length(), '0') + std::to_string(sampleIndex + 1) + "_Cur.dat";
            std::snprintf(logLine, sizeof(logLine), "Image acquired : %s", fileName.c_str());
            axisLog(logLine);

            const tuple<double, double> shift = getImageShift(
                binnedImage.data(),
                currentImage,
                currentImageFT,
                correlatedImageFT,
                correlatedImage,
                sampleIndex,
                nullptr,
                0,
                0,
                &counter,
                &updateReference,
                autoGuiderCounter,
                autoGuiderHappening,
                refreshInterval,
                &numRefImage,
                true,
                flatFrameIndice,
                0
            );
            shiftX += get<0>(shift);
            shiftY += get<1>(shift);
        }

        if (axis == 0) {
            XShift += settings.voltageRange / settings.frames;
        } else {
            YShift += settings.voltageRange / settings.frames;
        }
        FRAMENUMBER += 1;

        if (settings.mode == kActuatorMode) {
            setVoltagesXY(XShift, YShift);
        } else {
            setMotorCount(motor, sgn(settings.voltageRange / settings.frames), int(settings.voltageRange / settings.frames));
            const float64 timeToCompleteMovement = settings.voltageRange * 1000 / (settings.motorFrequency * settings.frames);
            Sleep(timeToCompleteMovement * 10);
        }

        fx = settings.mode == kMotorMode ? settings.stationaryVoltage : XShift;
        fy = settings.mode == kMotorMode ? settings.stationaryVoltage : YShift;
        flatFrameIndice = get_flat_indice(fx, fy);

        std::snprintf(
            logLine,
            sizeof(logLine),
            "%s%lf,%lf",
            shiftPrefix,
            shiftX / settings.framesPerPosition,
            shiftY / settings.framesPerPosition
        );
        shiftLog(logLine);
    }

    if (settings.mode == kMotorMode) {
        setMotorCount(motor, sgn(-1 * settings.voltageRange), int(settings.voltageRange * 0.5));
        const float64 timeToCompleteMovement = settings.voltageRange * 1000 * 0.5 / settings.motorFrequency;
        Sleep(timeToCompleteMovement * 3);
        exitMotor(1);
        disableMotor(1);
        exitMotor(2);
        disableMotor(2);
        closeControllerConnection();
    }

    const auto t1 = chrono::high_resolution_clock::now();
    const chrono::duration<double> dt = chrono::duration_cast<chrono::duration<double>>(t1 - t0);
    std::snprintf(logLine, sizeof(logLine), "Number of frames: %d, In seconds: %.6f", settings.frames, dt.count());
    axisLog(logLine);
    std::snprintf(logLine, sizeof(logLine), "Frame rate: %.6f", settings.frames / dt.count());
    axisLog(logLine);

    mkl_free(currentImage);
    mkl_free(currentImageFT);
    mkl_free(correlatedImageFT);
    mkl_free(correlatedImage);
    return 0;
}

inline void closedLoopCallBack(uint16_t* const image, uint64_t nbImagesReceived, ClosedLoopContext& context);
inline CorrectionResult getCorrection(double xTemp, double yTemp, uint64_t nbImagesReceived, ClosedLoopContext& context);
DWORD WINAPI closedLoopVoltageThread(LPVOID lparam);
DWORD WINAPI closedLoopAutoGuiderThread(LPVOID lparam);


int InitializeCameraSessionForUi(const TipTiltUiSettings& settings) {
    ensureLoggingInitialized();
    if (uiRuntimeState.cameraReady) {
        return 0;
    }

    Err = initDev(settings.camera);
    if (Err != 0) {
        return Err;
    }

    Err = startCamera();
    if (Err != 0) {
        stopDev();
        return Err;
    }
    uiRuntimeState.observer = std::make_unique<RawImageReceivedObserver>();

    uiRuntimeState.cameraReady = true;
    return 0;
}

int InitializeTipTiltHardwareForUi(const TipTiltUiSettings& settings) {
    ensureLoggingInitialized();
    if (uiRuntimeState.tipTiltReady) {
        return 0;
    }

    if (!initializeImagePipeline()) {
        return -1;
    }

    // The Qt window owns preview rendering; do not open the legacy OpenCV live view.
    liveView = 0;

    if (settings.xComPort <= 0 || settings.yComPort <= 0) {
        return -2;
    }

    Err = openXYSerialPorts(settings.xComPort, settings.yComPort);
    if (Err != 0) {
        return Err;
    }

    moveActuatorsToOffsetAndInitializeDaq();
    uiRuntimeState.tipTiltReady = true;
    return 0;
}

int InitializeAutoguiderHardwareForUi(const TipTiltUiSettings& settings) {
    if (uiRuntimeState.autoguiderReady) {
        return 0;
    }

    enableAutoguiderHardware(settings.autoguiderComPort, settings.autoguiderMotorFrequency);
    uiRuntimeState.autoguiderReady = true;
    return 0;
}

int StartClosedLoopForUi(const TipTiltUiSettings& settings) {
    if (uiRuntimeState.closedLoopRunning) {
        return 0;
    }
    if (!uiRuntimeState.cameraReady || !uiRuntimeState.tipTiltReady) {
        return -1;
    }

    shutdownRequested = false;
    uiRuntimeState.observer->configureClosedLoop(
        closedLoopCallBack,
        settings.referenceRefreshFrames,
        Vxoff,
        Vyoff,
        settings.acquisitionMode,
        uiRuntimeState.autoguiderReady ? 1 : 0
    );
    uiRuntimeState.handles = startWorkerThreads(*uiRuntimeState.observer);
    uiRuntimeState.closedLoopRunning = true;
    return 0;
}

int ApplyManualTipTiltVoltagesForUi(double xVoltage, double yVoltage) {
    if (!uiRuntimeState.tipTiltReady) {
        return -1;
    }
    return setVoltagesXY(xVoltage, yVoltage);
}

int RunDarkFlatCaptureForUi(const DarkFlatUiSettings& settings) {
    if (!uiRuntimeState.cameraReady || !uiRuntimeState.tipTiltReady) {
        return -1;
    }
    if (uiRuntimeState.closedLoopRunning) {
        return -2;
    }
    if (settings.framesPerPosition <= 0 || settings.step <= 0) {
        return -3;
    }

    std::filesystem::create_directories(settings.outputDirectory);
    std::vector<uint16_t> binnedImage(NX * NY, 0);
    std::vector<uint32_t> meanImage(NX * NY, 0);

    for (int y = settings.yStart; y <= settings.yEnd; y += settings.step) {
        for (int x = settings.xStart; x <= settings.xEnd; x += settings.step) {
            setVoltagesXY(x, y);
            if (settings.settleTimeMs > 0) {
                Sleep(settings.settleTimeMs);
            }

            std::fill(meanImage.begin(), meanImage.end(), 0);
            for (int frameIndex = 0; frameIndex < settings.framesPerPosition; ++frameIndex) {
                const uint16_t* image = reinterpret_cast<const uint16_t*>(fli->getRawImage());
                if (image == nullptr) {
                    return -4;
                }
                bin_separately(const_cast<uint16_t*>(image), binnedImage.data());
                for (unsigned pixelIndex = 0; pixelIndex < NPIX; ++pixelIndex) {
                    meanImage[pixelIndex] += binnedImage[pixelIndex];
                }
            }

            for (unsigned pixelIndex = 0; pixelIndex < NPIX; ++pixelIndex) {
                binnedImage[pixelIndex] = static_cast<uint16_t>(meanImage[pixelIndex] / settings.framesPerPosition);
            }

            const std::filesystem::path filePath = std::filesystem::path(settings.outputDirectory) /
                (settings.filePrefix + "_" + std::to_string(x) + "_" + std::to_string(y) + ".dat");
            FILE* filePointer = nullptr;
            fopen_s(&filePointer, filePath.string().c_str(), "wb");
            if (filePointer == nullptr) {
                return -5;
            }
            fwrite(binnedImage.data(), sizeof(uint16_t), binnedImage.size(), filePointer);
            fclose(filePointer);
        }
    }

    setVoltagesXY(-20.0, -20.0);
    return 0;
}

int RunCalibrationForUi(const CalibrationUiSettings& settings) {
    if (!uiRuntimeState.cameraReady || !uiRuntimeState.tipTiltReady) {
        return -1;
    }
    if (uiRuntimeState.closedLoopRunning) {
        return -2;
    }
    if (settings.mode != 0 && settings.mode != 1) {
        return -3;
    }
    if (settings.axis < -1 || settings.axis > 1) {
        return -4;
    }

    if (settings.axis == -1) {
        const int xStatus = runCalibrationAxis(settings, 0);
        if (xStatus != 0) {
            return xStatus;
        }
        return runCalibrationAxis(settings, 1);
    }

    return runCalibrationAxis(settings, settings.axis);
}

void ShutdownTipTiltRuntimeForUi() {
    if (uiRuntimeState.closedLoopRunning) {
        if (uiRuntimeState.observer != nullptr) {
            uiRuntimeState.observer->disableClosedLoop();
        }
        stopWorkerThreads(uiRuntimeState.handles);
    }

    if (uiRuntimeState.autoguiderReady) {
        disableAutoguiderHardware();
    }

    if (uiRuntimeState.tipTiltReady) {
        parkActuatorsAtOffset();
        closeDAQ();
        closeXYSerialPorts();
    }

    if (uiRuntimeState.cameraReady) {
        stopCamera();
        stopDev();
    }

    resetUiRuntimeState();
    latestPublishedTipTiltCommandId.store(0);
    latestAppliedTipTiltCommandId.store(0);
    nextTipTiltCommandId.store(0);
}

bool CopyLatestPreviewFrameForUi(std::vector<uint16_t>& frame) {
    if (!uiRuntimeState.cameraReady || uiRuntimeState.observer == nullptr) {
        return false;
    }
    return uiRuntimeState.observer->copyLatestPreviewFrame(frame);
}

bool IsClosedLoopRunningForUi() {
    return uiRuntimeState.closedLoopRunning;
}

bool IsAutoguiderInitializedForUi() {
    return uiRuntimeState.autoguiderReady;
}

inline CorrectionResult getCorrection(
        double XTemp, double YTemp, uint64_t nbImagesReceived, ClosedLoopContext& context
) {
    double toCorrectXTemp, toCorrectYTemp;
    double integralErrorVoltageX = 0;
    double integralErrorVoltageY = 0;
    double derivativeErrorVoltageX = 0;
    double derivativeErrorVoltageY = 0;

    toCorrectXTemp = -1 * XTemp; // multiplied by -1, because error is set point minus measured value
    toCorrectYTemp = -1 * YTemp;

    double correctionShiftX, correctionShiftY;

    if (Ni > 0) {
        if (context.integralErrorX.size() == Ni) {
            context.integralErrorSumX -= context.integralErrorX.front();
            context.integralErrorSumY -= context.integralErrorY.front();
            context.integralErrorX.pop_front();
            context.integralErrorY.pop_front();
        }
        context.integralErrorX.push_back(toCorrectXTemp);
        context.integralErrorY.push_back(toCorrectYTemp);
        context.integralErrorSumX += toCorrectXTemp;
        context.integralErrorSumY += toCorrectYTemp;

        if (context.integralErrorX.size() < Ni) {
            integralErrorVoltageX = 0;
            integralErrorVoltageY = 0;
        } else {
            integralErrorVoltageX = context.integralErrorSumX;
            integralErrorVoltageY = context.integralErrorSumY;
        }
    }
    else {
        context.integralErrorSumX += toCorrectXTemp;
        context.integralErrorSumY += toCorrectYTemp;
        integralErrorVoltageX = context.integralErrorSumX;
        integralErrorVoltageY = context.integralErrorSumY;
    }

    if (Nd == 0) {
        derivativeErrorVoltageX = toCorrectXTemp - context.previousErrorX;
        derivativeErrorVoltageY = toCorrectYTemp - context.previousErrorY;
    }
    else if (Nd > 0) {
        const double derivativeInputX = toCorrectXTemp - context.previousErrorX;
        const double derivativeInputY = toCorrectYTemp - context.previousErrorY;
        if (context.derivativeCounter < static_cast<uint64_t>(Nd)) {
            context.derivativeCounter += 1;
            derivativeErrorVoltageX = 0;
            derivativeErrorVoltageY = 0;
        }
        else {
            derivativeErrorVoltageX = derivativeFilterX.processBiquadSample(derivativeInputX, coeffs.data());
            derivativeErrorVoltageY = derivativeFilterY.processBiquadSample(derivativeInputY, coeffs.data());
        }
    }
    else {
        // no derivative error if Nd is negative
        derivativeErrorVoltageX = 0;
        derivativeErrorVoltageY = 0;
    }

    context.previousErrorX = toCorrectXTemp;
    context.previousErrorY = toCorrectYTemp;

    double integralContributionX = Ki * integralErrorVoltageX;
    double integralContributionY = Ki * integralErrorVoltageY;

    if (integralContributionX <= limMinInt) {
        integralContributionX = limMinInt;
    }
    if (integralContributionX >= limMaxInt) {
        integralContributionX = limMaxInt;
    }
    if (integralContributionY <= limMinInt) {
        integralContributionY = limMinInt;
    }
    if (integralContributionY >= limMaxInt) {
        integralContributionY = limMaxInt;
    }

    double usedKp = Kp;
    double usedKd = Kd;

    if (autoPMode) {
        usedKp = max(fabs(toCorrectXTemp), fabs(toCorrectYTemp)) * mp + cp;
        if (usedKp >= maxKp) usedKp = maxKp;
        usedKd = md * usedKp + cd;
        if (usedKd >= maxKd) usedKd = maxKd;
    }

    correctionShiftX = usedKp * toCorrectXTemp + integralContributionX + usedKd * derivativeErrorVoltageX;
    correctionShiftY = usedKp * toCorrectYTemp + integralContributionY + usedKd * derivativeErrorVoltageY;

    double usedLimMin = limMin;
    double usedLimMax = limMax;

    if (correctionShiftX <= usedLimMin) {
        correctionShiftX = usedLimMin;
    }
    if (correctionShiftX >= usedLimMax) {
        correctionShiftX = usedLimMax;
    }
    if (correctionShiftY <= usedLimMin) {
        correctionShiftY = usedLimMin;
    }
    if (correctionShiftY >= usedLimMax) {
        correctionShiftY = usedLimMax;
    }

    CorrectionResult correction;
    correction.x = A00 * correctionShiftX + A01 * correctionShiftY;
    correction.y = A10 * correctionShiftX + A11 * correctionShiftY;

    char shiftLine[512];
    std::snprintf(
        shiftLine,
        sizeof(shiftLine),
        "%ld, %ld, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf",
        context.currCount, nbImagesReceived, toCorrectXTemp, toCorrectYTemp,
        integralContributionX, integralContributionY, derivativeErrorVoltageX,
        derivativeErrorVoltageY, correctionShiftX, correctionShiftY,
        correction.x, correction.y, usedKp
    );
    shift_uncorected_log(shiftLine);
    return correction;
}


inline void closedLoopCallBack(uint16_t* const image, uint64_t nbImagesReceived, ClosedLoopContext& context) {
    if (context.breakLoop) {
        return;
    }
    if (context.autoGuiderCounter > 0) context.autoGuiderCounter -= 1;

    tuple<double, double> XYIND;
    double XTemp, YTemp;
    double correctionX, correctionY;
    bool autoGuiderHappening = false;
    bin_separately(image, context.binnedImage);

    if (context.ignoreCount) {
        context.ignoreCount -= 1;
        return;
    }
    if (!context.numRefImage){
        context.numRefImage += 1;
        processReferenceImage(context.binnedImage);
        context.counter = 0;
        context.updateReference = false;
    }
    else {
        int flatIndice = get_flat_indice(context.xShift, context.yShift);
        XYIND = getImageShift(
                context.binnedImage,
                context.currentImage, context.currentImageFT, context.correlatedImageFT,
                context.correlatedImage, context.currCount,
                &context.imageSaveCounter, context.fpsCamera, imageSaveAfterSecond,
                &context.counter, &context.updateReference, context.autoGuiderCounter,
                autoGuiderHappening,
                context.refreshInterval, &context.numRefImage,
                useCameraFlat, flatIndice, liveView);
        XTemp = get<0>(XYIND);
        YTemp = get<1>(XYIND);

        context.sumX -= XTemp;
        context.sumY -= YTemp;

        if (
                context.autoGuiderMode == 1 &&
                (
                    context.offloadShiftsToAutoguider ||
                    (context.acqMode == 2 && context.rotatingCounter == autoGuiderCorrectionTime * context.fpsCamera)
                )
           ) {
            autoGuiderHappening = true;
            double corrVoltX;
            double corrVoltY;
            double pixelShiftX;
            double pixelShiftY;
            if (context.acqMode == 1) {
                context.integralVoltX += context.meanVoltX;
                context.integralVoltY += context.meanVoltY;
                corrVoltX = Akp * context.meanVoltX + Aki * context.integralVoltX;
                corrVoltY = Akp * context.meanVoltY + Aki * context.integralVoltY;
                pixelShiftX = ClM00 * corrVoltX + ClM01 * corrVoltY;
                pixelShiftY = ClM10 * corrVoltX + ClM11 * corrVoltY;
            }
            else {
                context.meanVoltX = -XTemp;
                context.meanVoltY = -YTemp;
                context.integralVoltX += context.meanVoltX;
                context.integralVoltY += context.meanVoltY;
                corrVoltX = Akp * context.meanVoltX + Aki * context.integralVoltX;
                corrVoltY = Akp * context.meanVoltY + Aki * context.integralVoltY;
                pixelShiftX = corrVoltX;
                pixelShiftY = corrVoltY;
                context.rotatingCounter = 0;
            }

            int autoCorX = int(AA00 * pixelShiftX + AA01 * pixelShiftY);
            int autoCorY = int(AA10 * pixelShiftX + AA11 * pixelShiftY);
            int limiter = 350;
            if (autoCorX > limiter) {
                autoCorX = limiter;
            }
            if (autoCorX < -1 * limiter) {
                autoCorX = -1 * limiter;
            }
            if (autoCorY > limiter) {
                autoCorY = limiter;
            }
            if (autoCorY < -1 * limiter) {
                autoCorY = -1 * limiter;
            }

            {
                lock_guard<mutex> aul(autoGuiderMutex);
                latestAutoGuiderCommand = {autoCorX, autoCorY};
                autoGuiderReady = true;
            }
            autoGuiderConditionalVariable.notify_one();

            context.offloadShiftsToAutoguider = false;
            int waitTimeInSeconds = 1;
            context.autoGuiderCounter = waitTimeInSeconds * context.fpsCamera;
            char autoguiderLine[256];
            std::snprintf(
                    autoguiderLine,
                    sizeof(autoguiderLine),
                    "%llu, %lf, %lf, %lf, %lf, %lf, %lf, %d, %d",
                    context.currCount, context.meanVoltX, context.meanVoltY, context.integralVoltX,
                    context.integralVoltY, corrVoltX, corrVoltY, autoCorX, autoCorY
            );
            autoguider_log(autoguiderLine);
        }

        const CorrectionResult correction = getCorrection(XTemp, YTemp, nbImagesReceived, context);
        correctionX = correction.x;
        correctionY = correction.y;

        if (context.acqMode == 1) {
            if ((context.xShift + correctionX < -20) || (context.xShift + correctionX > 130)) {
                context.errorNo = 1;
                context.breakLoop = true;
            }
            else if ((context.yShift + correctionY < -20) || (context.yShift + correctionY > 130)) {
                context.errorNo = 2;
                context.breakLoop = true;
            }
            else {
                context.errorNo = 0;
            }
            if (context.errorNo == 0) {
                context.xShift += correctionX;
                context.yShift += correctionY;
                double instantErrorX = fabs(context.xShift - Vxoff);
                double instantErrorY = fabs(context.yShift - Vyoff);
                if (
                        context.rotatingCounter == context.fpsCamera ||
                        instantErrorX > 40 ||
                        instantErrorY > 40
                   ) {
                    context.meanVoltX = (context.sumVoltageX / (context.rotatingCounter + 1)) - Vxoff;
                    context.meanVoltY = (context.sumVoltageY / (context.rotatingCounter + 1)) - Vyoff;
                    if (
                            (fabs(context.meanVoltX) > autoGuiderOffloadLimitX) ||
                            (fabs(context.meanVoltY) > autoGuiderOffloadLimitY)
                       ){
                        if ((context.autoGuiderCounter == 0) && !context.offloadShiftsToAutoguider) {
                            context.offloadShiftsToAutoguider = true;
                            context.updateReference = false;
                        }
                    }
                    context.sumVoltageX = 0;
                    context.sumVoltageY = 0;
                    context.rotatingCounter = 0;
                }
                context.sumVoltageX += context.xShift;
                context.sumVoltageY += context.yShift;
                {
                    lock_guard<mutex> ul(tipTiltMutex);
                    const uint64_t commandId = nextTipTiltCommandId.fetch_add(1, std::memory_order_relaxed) + 1;
                    latestTipTiltCommand = {
                            context.xShift,
                            context.yShift,
                            -XTemp,
                            -YTemp,
                            context.currCount,
                            nbImagesReceived,
                            commandId
                    };
                    latestPublishedTipTiltCommandId.store(commandId, std::memory_order_release);
                    tipTiltReady = true;
                }
                tipTiltConditionalVariable.notify_one();
            }
        }
        context.counter += 1;
        context.currCount += 1;
    }

    context.rotatingCounter += 1;
}

DWORD WINAPI closedLoopVoltageThread(LPVOID lparam) {
    while (true) {
        unique_lock<mutex> ul(tipTiltMutex);
        tipTiltConditionalVariable.wait(ul, []() {
            return latestPublishedTipTiltCommandId.load(std::memory_order_acquire) >
                           latestAppliedTipTiltCommandId.load(std::memory_order_acquire) ||
                   shutdownRequested.load();
        });
        if (
                shutdownRequested.load() &&
                latestPublishedTipTiltCommandId.load(std::memory_order_acquire) <=
                        latestAppliedTipTiltCommandId.load(std::memory_order_acquire)
           ) {
            break;
        }

        TipTiltCommand command;
        uint64_t publishedCommandId = 0;
        do {
            command = latestTipTiltCommand;
            publishedCommandId = latestPublishedTipTiltCommandId.load(std::memory_order_acquire);
        } while (command.commandId != publishedCommandId);
        ul.unlock();

        if (command.commandId <= latestAppliedTipTiltCommandId.load(std::memory_order_acquire)) {
            continue;
        }

        setVoltagesXY(command.xShift, command.yShift);
        latestAppliedTipTiltCommandId.store(command.commandId, std::memory_order_release);

        char shiftLine[256];
        std::snprintf(shiftLine, sizeof(shiftLine), "%llu, %llu, %lf, %lf, %lf, %lf, %d",
                command.currCount, command.nbImagesReceived, command.xTemp, command.yTemp,
                command.xShift, command.yShift, 0);
        shift_log(shiftLine);
    }
    return 0;
}

DWORD WINAPI closedLoopAutoGuiderThread(LPVOID lparam) {
    while (true) {
        unique_lock<mutex> aul(autoGuiderMutex);
        autoGuiderConditionalVariable.wait(aul, [](){return autoGuiderReady || shutdownRequested.load();});
        if (shutdownRequested.load() && !autoGuiderReady) {
            break;
        }
        AutoGuiderCommand command = latestAutoGuiderCommand;
        autoGuiderReady = false;
        aul.unlock();
        setMotorCount(2, sgn(command.xCounts), abs(command.xCounts));
        setMotorCount(1, sgn(command.yCounts), abs(command.yCounts));
    }
    return 0;
}
