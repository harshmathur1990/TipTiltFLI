#include "imageprocess.h"
#include "controls.h"
#include "firstlight.h"
#include "NIDAQmx.h"
#include <mutex>
#include "utilheaders.h"
#include "motorcontrols.h"
#include "pid.h"
#include <thread>

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

/* Sample time (in seconds) */
double sampleTime;
int liveView;
bool useCameraFlat;
string NucMode;
PIDController pidX;
PIDController pidY;
char logString[100000],  logString_2[100000], logString_3[100000];
int (*loggingfunc) (std::string) = NULL;
bool tipTiltReady = false, autoGuiderReady = false;
mutex tipTiltMutex, autoGuiderMutex;
deque<double> tipTiltQueue;
deque<uint64_t> tipTiltQueue_2;
deque<int> autoGuiderQueue;
std::condition_variable tipTiltConditionalVariable, autoGuiderConditionalVariable;
bool displayReady = false;
mutex displayMutex;
deque<double**> displayQueue;
std::condition_variable displayConditionalVariable;
uint64_t displayCount;

inline void closedLoopCallBack(uint16_t* const Image, uint64_t nbImagesReceived, double* binImgTime, double* imgWriteTime,
                        double* imgShiftTime, double* calcCorrectionTime, double* applyCorrectionTime,
                        uint64_t* curr_count, uint64_t* ignore_count, uint64_t NREFRESH, uint64_t* NumRefImage,
                        uint64_t* integralCounter, uint64_t* derivativeCounter, bool* breakLoop, uint64_t* counter,
                        uint64_t* status_count, double* XShift, double* YShift, uint64_t* error_no,
                        deque<double> &integralErrorX, deque<double> &integralErrorY,
                        deque<double> &derivativeErrorX, deque<double> &derivativeErrorY,
                        double *CurrentImage, fftw_complex *CurrentImageFT,
                        fftw_complex *CorrelatedImageFT, double *CorrelatedImage,
                        int fpsCamera, uint16_t* const binnedImage, int ACQMODE,
                        uint64_t* rotatingCounter, double *sumX, double *sumY,
                        int AUTOGUIDERMODE, double *sumVoltageX, double *sumVoltageY,
                        uint64_t *imageSaveCounter, double *integralVoltX, double *integralVoltY,
                        bool *updateReference, bool *offloadShiftsToAutoguider, uint32_t* autoGuiderCounter,
                        double* meanVoltX, double* meanVoltY);
inline int getCorrection(double XTemp, double YTemp, deque<double> &integralErrorX, deque<double> &integralErrorY,
                  deque<double> &derivativeErrorX, deque<double> &derivativeErrorY, uint64_t *integralCounter,
                  uint64_t *derivativeCounter, double *correctionX, double *correctionY,
                  uint64_t curr_count, uint64_t nbImagesReceived
);
[[noreturn]] DWORD WINAPI closedLoopVoltageThread(LPVOID lparam);
[[noreturn]] DWORD WINAPI closedLoopAutoGuiderThread(LPVOID lparam);
[[noreturn]] DWORD WINAPI displayThread(LPVOID lparam);

MKL_LONG counterImager=0;
double totalBinTime=0;

int main () {
    float XShift, YShift;
    setupLogging(2);
    log(logString);
    sprintf(logString, "Allocating memory for various parameters");
    log(logString);
    // Allocation
    COMMAND[14] = 13; // CR
    COMMAND[15] = 10; // LF

    // Other variables

    getDarkFlat();

	int  ACQMODE = 1, NREFREFRESH=100, AUTOGUIDER=0;

    Err = getCalibrationMatrix();

    sprintf(logString, "Initializing devices and acquiring one-time data");
    log(logString);
    cout << "Searching for the cameras... " << endl;
	Err = initDev(1, 0, NucMode);
	if (Err != 0) {
	    exit(Err);
	}
    cout << "Creating the window function... " << endl;
    Err = getHammingWindow();
    cout << "Creating the Fourier transform plans... " << endl;
    Err = initializeFFT();
    if (Err != 0) {
        Err = stopDev();
        return -1;
    }
    cout << "Opening the serial COM ports... " << endl;
    Err = openXYSerialPorts();
    cout << "Loading the calibration matrix... " << endl;


    pidX = {Kp, Ki, Kd,
                         tau,
                         limMin, limMax,
                         limMinInt, limMaxInt,
                         sampleTime };
    pidY = {Kp, Ki, Kd,
                          tau,
                          limMin, limMax,
                          limMinInt, limMaxInt,
                          sampleTime };
    PIDController_Init(&pidX);
    PIDController_Init(&pidY);
    // Go to offset point
    loggingfunc = oplog_write;
    sendCommand(XPort, "set,0,-20.0");
    sendCommand(YPort, "set,0,-20.0");
    Sleep(LONG_DELAY);
    sendCommand(XPort, "sr,0," + to_string(SlewRate));
    sendCommand(YPort, "sr,0," + to_string(SlewRate));

    cout << "Initializing DAQ..." << endl;
    Err = initDAQ();
    setVoltagesXY(Vxoff, Vyoff);
    cout << "Enter acquisition mode, 1: with corrections, 2: only shifts-no corrections... ";
    cin >> ACQMODE;
    cout.flush();
    cout << "Enter Autoguider mode (0: OFF, 1: ON)...";
    cin >> AUTOGUIDER;
    cout.flush();

    if (AUTOGUIDER) {
        CreateControllerConnection(1);
        enableMotor(1);
        enableMotor(2);
        setMotorFrequency(1, 350);
        setMotorFrequency(2, 350);
    }
    cout << "Number of frames to refresh reference : ";
    cin >> NREFREFRESH;
    cout.flush();
    // Start
    sprintf(logString, "Number of frames to refresh reference : %d", NREFREFRESH);
    log(logString);
    cout.flush();
    string slopeString;
    HANDLE tipTiltHandle, autoguiderHandle, displayHandle;
    DWORD tipTiltThreadID, autoguiderThreadID, displayThreadID;
//    std::thread tiptiltThread(closedLoopVoltageThread);
//    std::thread autoGuiderThread(closedLoopAutoGuiderThread);
    tipTiltHandle = CreateThread(0, 0, closedLoopVoltageThread, NULL, 0, &tipTiltThreadID);
    autoguiderHandle = CreateThread(0, 0, closedLoopAutoGuiderThread, NULL, 0, &autoguiderThreadID);
    RawImageReceivedObserver obs(
            closedLoopCallBack, NREFREFRESH,
            Vxoff, Vyoff, ACQMODE, AUTOGUIDER,
            imageSaveAfterSecond);
    if (liveView) {
        double* ci = obs.getCurrentImage();
        displayHandle = CreateThread(0, 0, displayThread, ci, 0, &displayThreadID);
    }

    startCamera();
    while(slopeString[0] != 'q') {
        cout << "Enter q to quit program" << endl;
        getline(std::cin, slopeString);
        if ((slopeString[0] == 't') || (slopeString[0] == 'T')) {
            cout << "Enter ACQMODE" << endl;
            cin>>ACQMODE;
            if ((ACQMODE == 0) || (ACQMODE == 1)) {
                obs.setACQMODE(ACQMODE);
            }
        }
        if ((slopeString[0] == 'p') || (slopeString[0] == 'P')) {
            cout << "Enter Kp" << endl;
            cin>>Kp;
            cout << "Enter Ki" << endl;
            cin>>Ki;
            cout << "Enter Kd" << endl;
            cin>>Kd;
        }
        if ((slopeString[0] == 'a') || (slopeString[0] == 'A')) {
            cout << "Enter Autoguider mode: "<<endl;
            int amode;
            cin>>amode;
            if ((amode == 1) && (AUTOGUIDER == 0)) {
                CreateControllerConnection(1);
                enableMotor(1);
                enableMotor(2);
                setMotorFrequency(1, 350);
                setMotorFrequency(2, 350);
                obs.setAUTOGUIDERMODE(amode);
                AUTOGUIDER = amode;
            }
            else if ((amode == 0) && (AUTOGUIDER == 1)) {
                obs.setAUTOGUIDERMODE(amode);
                exitMotor(1);
                disableMotor(1);
                exitMotor(2);
                disableMotor(2);
                closeControllerConnection();
                AUTOGUIDER = amode;
            }
        }
        if ((slopeString[0] == 'c') || (slopeString[0] == 'C')) {
            if (!AUTOGUIDER) {
                cout << "Autoguider off. Switch on first"<<endl;
                continue;
            }
            cout << "Enter counts"<<endl;
            int counts;
            cin>>counts;
            unique_lock<mutex> aul(autoGuiderMutex);
            autoGuiderQueue.push_back(counts);
            autoGuiderQueue.push_back(counts);
            autoGuiderReady = true;
            aul.unlock();
            autoGuiderConditionalVariable.notify_one();
            aul.lock();
            autoGuiderConditionalVariable.wait(aul, [](){return !autoGuiderReady;});
            aul.unlock();
        }
    }
    stopCamera();
    CloseHandle(tipTiltHandle);
    CloseHandle(autoguiderHandle);
    if (liveView) {
        CloseHandle(displayHandle);
    }
    if (AUTOGUIDER) {
        exitMotor(1);
        disableMotor(1);
        exitMotor(2);
        disableMotor(2);
        closeControllerConnection();
    }
    // Clearing data
    XShift = -20;
    YShift = -20;
    cout << "Actuator going to offset point..." << endl;
    Err = setVoltagesXY(XShift, YShift);
    Err = closeDAQ();
    Err = closeXYSerialPorts();
	Err = stopDev();
	cout << "Press a key and then enter to exit." << endl;
	char c;
	cin >> c;
	return 0;
}

inline int getCorrection(double XTemp, double YTemp, deque<double> &integralErrorX, deque<double> &integralErrorY,
                  deque<double> &derivativeErrorX, deque<double> &derivativeErrorY, uint64_t *integralCounter, uint64_t *derivativeCounter,
                  double *correctionX, double *correctionY, uint64_t curr_count, uint64_t nbImagesReceived
) {
    uint64_t i;
    double toCorrectXTemp, toCorrectYTemp;
    double xMean =  (double) (Nd + 1) / (double) 2;
    double xixbaryiybarX = 0, xixbaryiybarY = 0;
    double xixbarsq = 0;
    double integralErrorVoltageX = 0;
    double integralErrorVoltageY = 0;
    double derivativeErrorVoltageX = 0;
    double derivativeErrorVoltageY = 0;

    toCorrectXTemp = -1 * XTemp;
    toCorrectYTemp = -1 * YTemp;

    double correctionShiftX, correctionShiftY;

    if (Ni > 0) {
        if (integralErrorX.size() == Ni) {
            integralErrorX.pop_front();
            integralErrorY.pop_front();
        }
        integralErrorX.push_back(toCorrectXTemp);
        integralErrorY.push_back(toCorrectYTemp);

        if (integralErrorX.size() < Ni) {
            integralErrorVoltageX = 0;
            integralErrorVoltageY = 0;
        } else {
            integralErrorVoltageX = accumulate(integralErrorX.end() - Ni, integralErrorX.end(), 0.0);
            integralErrorVoltageY = accumulate(integralErrorY.end() - Ni, integralErrorY.end(), 0.0);
        }
    }
    else {
        if (integralErrorX.empty()) {
            integralErrorX.push_back(toCorrectXTemp);
            integralErrorY.push_back(toCorrectYTemp);
            integralErrorVoltageX = 0;
            integralErrorVoltageY = 0;
        }
        else {
            double sumX = integralErrorX.front();
            double sumY = integralErrorY.front();
            integralErrorX.pop_front();
            integralErrorY.pop_front();
            sumX += toCorrectXTemp;
            sumY += toCorrectYTemp;
            integralErrorX.push_back(sumX);
            integralErrorY.push_back(sumY);
            integralErrorVoltageX = sumX;
            integralErrorVoltageY = sumY;
        }
    }

    if (derivativeErrorX.size() == Nd) {
        derivativeErrorX.pop_front();
        derivativeErrorY.pop_front();
    }
    derivativeErrorX.push_back(toCorrectXTemp);
    derivativeErrorY.push_back(toCorrectYTemp);

    if (derivativeErrorX.size() < Nd) {
        derivativeErrorVoltageX = 0;
        derivativeErrorVoltageY = 0;
    }
    else {
        double meanDerivativeErrorVoltageX = 0;
        double meanDerivativeErrorVoltageY = 0;

        meanDerivativeErrorVoltageX = accumulate(derivativeErrorX.end()-Nd, derivativeErrorX.end(), 0.0) / Nd;
        meanDerivativeErrorVoltageY = accumulate(derivativeErrorY.end()-Nd, derivativeErrorY.end(), 0.0) / Nd;

        i = 0;
        for (auto it=derivativeErrorX.cbegin(); it!=derivativeErrorX.cend(); it++) {
            xixbaryiybarX += (*it - meanDerivativeErrorVoltageX) * (i + 1 - xMean);
            xixbarsq += ((i + 1 - xMean) * (i + 1 - xMean));
            i += 1;
        }
        i = 0;
        for(auto it=derivativeErrorY.cbegin(); it!=derivativeErrorY.cend(); it++){
            xixbaryiybarY += (*it - meanDerivativeErrorVoltageY) * (i + 1 - xMean);
            i += 1;
        }

        derivativeErrorVoltageX = xixbaryiybarX / xixbarsq;
        derivativeErrorVoltageY = xixbaryiybarY / xixbarsq;
    }

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
    correctionShiftX = Kp * toCorrectXTemp + integralContributionX + Kd * derivativeErrorVoltageX;
    correctionShiftY = Kp * toCorrectYTemp + integralContributionX + Kd * derivativeErrorVoltageY;

    if (correctionShiftX <= limMin) {
        correctionShiftX = limMin;
    }
    if (correctionShiftX >= limMax) {
        correctionShiftX = limMax;
    }
    if (correctionShiftY <= limMin) {
        correctionShiftY = limMin;
    }
    if (correctionShiftY >= limMax) {
        correctionShiftY = limMax;
    }
//    PIDController_Update(&pidX, 0, toCorrectXTemp);
//    PIDController_Update(&pidY, 0, toCorrectYTemp);

    *correctionX = A00*correctionShiftX + A01*correctionShiftY;
    *correctionY = A10*correctionShiftX + A11*correctionShiftY;

//    *correctionX = A00 * pidX.out + A01 * pidY.out;
//    *correctionY = A10 * pidX.out + A11 * pidY.out;

//    sprintf(logString, "%ld, %ld, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf", curr_count, nbImagesReceived, toCorrectXTemp, toCorrectYTemp, pidX.integrator, pidY.integrator, pidX.differentiator, pidY.differentiator, pidX.out, pidY.out, *correctionX, *correctionY);
    sprintf(
        logString,
        "%ld, %ld, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf",
        curr_count, nbImagesReceived, toCorrectXTemp, toCorrectYTemp,
        integralContributionX, integralContributionY, derivativeErrorVoltageX,
        derivativeErrorVoltageY, correctionShiftX, correctionShiftY,
        *correctionX, *correctionY
    );
    shift_uncorected_log(logString);
    return 0;
}


inline void closedLoopCallBack(
        uint16_t* const Image, uint64_t nbImagesReceived, double* binImgTime, double* imgWriteTime,
        double* imgShiftTime, double* calcCorrectionTime, double* applyCorrectionTime,
        uint64_t* curr_count, uint64_t* ignore_count,
        uint64_t NREFRESH, uint64_t* NumRefImage,
        uint64_t* integralCounter, uint64_t* derivativeCounter, bool* breakLoop, uint64_t* counter,
        uint64_t* status_count, double* XShift, double* YShift, uint64_t* error_no,
        deque<double> &integralErrorX, deque<double> &integralErrorY,
        deque<double> &derivativeErrorX, deque<double> &derivativeErrorY,
        double *CurrentImage, fftw_complex *CurrentImageFT,
        fftw_complex *CorrelatedImageFT, double *CorrelatedImage, int fpsCamera,
        uint16_t* const binnedImage, int ACQMODE, uint64_t* rotatingCounter,
        double *sumX, double *sumY, int AUTOGUIDERMODE, double *sumVoltageX, double *sumVoltageY,
        uint64_t *imageSaveCounter, double *integralVoltX, double *integralVoltY,
        bool *updateReference, bool *offloadShiftsToAutoguider, uint32_t* autoGuiderCounter,
        double* meanVoltX, double* meanVoltY
        )
{

    if (*breakLoop == true) {
        return;
    }
    if (*autoGuiderCounter > 0) *autoGuiderCounter -= 1;
    tuple<double, double> XYIND;
    double XTemp, YTemp;
    double correctionX, correctionY;
    bool autoGuiderHappening = false;
    bin_separately(Image, binnedImage);

    if (*ignore_count) {
        *ignore_count = *ignore_count - 1;
        return;
    }
    if (
            *offloadShiftsToAutoguider ||
            (ACQMODE == 2 && *rotatingCounter == autoGuiderCorrectionTime * fpsCamera)
            )
    {
        if (AUTOGUIDERMODE == 1) {
            autoGuiderHappening = true;
            unique_lock<mutex> aul(autoGuiderMutex);
            double corrVoltX;
            double corrVoltY;
            double pixelShiftX;
            double pixelShiftY;
            if (ACQMODE == 1) {
                *integralVoltX += *meanVoltX;
                *integralVoltY += *meanVoltY;
                corrVoltX = Akp * *meanVoltX + Aki * *integralVoltX;
                corrVoltY = Akp * *meanVoltY + Aki * *integralVoltY;
                pixelShiftX = ClM00 * corrVoltX + ClM01 * corrVoltY;
                pixelShiftY = ClM10 * corrVoltX + ClM11 * corrVoltY;
            }
            else {
                *meanVoltX = -XTemp;
                *meanVoltY = -YTemp;
                *integralVoltX += *meanVoltX;
                *integralVoltY += *meanVoltY;
                corrVoltX = Akp * *meanVoltX + Aki * *integralVoltX;
                corrVoltY = Akp * *meanVoltY + Aki * *integralVoltY;
                pixelShiftX = corrVoltX;
                pixelShiftY = corrVoltY;
                *rotatingCounter = 0;
            }

            int autoCorX = int(AA00 * pixelShiftX + AA01 * pixelShiftY);
            int autoCorY = int(AA10 * pixelShiftX + AA11 * pixelShiftY);

            int waitTimeInSeconds = 1;
            int multiplier = *offloadShiftsToAutoguider?waitTimeInSeconds:autoGuiderCorrectionTime;
            int limiter = multiplier * 350;
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
            autoGuiderQueue.push_back(autoCorX);
            autoGuiderQueue.push_back(autoCorY);
            autoGuiderReady = true;
            aul.unlock();
            autoGuiderConditionalVariable.notify_one();
            aul.lock();
            autoGuiderConditionalVariable.wait(aul, [](){return !autoGuiderReady;});
            aul.unlock();

            *offloadShiftsToAutoguider = false;
            *autoGuiderCounter = waitTimeInSeconds * fpsCamera;
            sprintf(
                    logString_3,
                    "%llu, %lf, %lf, %lf, %lf, %lf, %lf, %d, %d",
                    *curr_count, *meanVoltX, *meanVoltY, *integralVoltX,
                    *integralVoltY, corrVoltX, corrVoltY, autoCorX, autoCorY
            );
            autoguider_log(logString_3);
        }
    }

    if (
            !*NumRefImage || (
                 (*counter > NREFRESH || *updateReference) &&
                 (*autoGuiderCounter == 0 && !autoGuiderHappening)
            )
       ) {
        *NumRefImage += 1;
        processReferenceImage(binnedImage);
        *counter = 0;
        *updateReference = false;
    }
    else {
        int flatIndice = get_flat_indice(*XShift, *YShift);
        XYIND = getImageShift(
                binnedImage,
                CurrentImage, CurrentImageFT, CorrelatedImageFT,
                CorrelatedImage, *curr_count,
                imageSaveCounter, fpsCamera, imageSaveAfterSecond,
                useCameraFlat, flatIndice, liveView);
        XTemp = get<0>(XYIND);
        YTemp = get<1>(XYIND);

        *sumX -= XTemp;
        *sumY -= YTemp;

        getCorrection(XTemp, YTemp, integralErrorX, integralErrorY,
                      derivativeErrorX, derivativeErrorY, integralCounter, derivativeCounter,
                      &correctionX, &correctionY, *curr_count, nbImagesReceived
        );

        if (ACQMODE == 1) {
            if ((*XShift + correctionX < -20) || (*XShift + correctionX > 130)) {
                *error_no = 1;
//                cout << "error_no is "<< *error_no<<endl;
                *breakLoop = true;
            }
            else if ((*YShift + correctionY < -20) || (*YShift + correctionY > 130)) {
                *error_no = 2;
//                cout << "error_no is "<< *error_no<<endl;
                *breakLoop = true;
            }
            else {
                *error_no = 0;
            }
            if (*error_no == 0) {
                *XShift += correctionX;
                *YShift += correctionY;
                if (*rotatingCounter == fpsCamera) {
                    *meanVoltX = (*sumVoltageX / (*rotatingCounter + 1)) - Vxoff;
                    *meanVoltY = (*sumVoltageY / (*rotatingCounter + 1)) - Vyoff;
                    if ((fabs(*meanVoltX) > autoGuiderOffloadLimitX) || (fabs(*meanVoltY) > autoGuiderOffloadLimitY)){
                        if ((*autoGuiderCounter == 0) && !*offloadShiftsToAutoguider) {
                            *offloadShiftsToAutoguider = true;
                            *updateReference = false;
                        }
                    }
                    *sumVoltageX = 0;
                    *sumVoltageY = 0;
                    *rotatingCounter = 0;
                }
                *sumVoltageX += *XShift;
                *sumVoltageY += *YShift;
                unique_lock<mutex> ul(tipTiltMutex);
                tipTiltQueue.push_back(*XShift);
                tipTiltQueue.push_back(*YShift);
                tipTiltQueue.push_back(-XTemp);
                tipTiltQueue.push_back(-YTemp);
                tipTiltQueue_2.push_back(*curr_count);
                tipTiltQueue_2.push_back(nbImagesReceived);
                tipTiltReady = true;
                ul.unlock();
                tipTiltConditionalVariable.notify_one();
                ul.lock();
                tipTiltConditionalVariable.wait(ul, []() { return !tipTiltReady; });
                ul.unlock();
            }
        }
        *counter += 1;
        *curr_count += 1;
    }

    *rotatingCounter += 1;
}

[[noreturn]] DWORD WINAPI closedLoopVoltageThread(LPVOID lparam) {
    double XXShift, YYShift, XTemp, YTemp;
    uint64_t curr_count, nbImagesReceived;
    int pending = 0;
    while (true) {
        unique_lock<mutex> ul(tipTiltMutex);
        tipTiltConditionalVariable.wait(ul, []() { return tipTiltReady; });
        XXShift = tipTiltQueue.front();
        tipTiltQueue.pop_front();
        YYShift = tipTiltQueue.front();
        tipTiltQueue.pop_front();
        XTemp = tipTiltQueue.front();
        tipTiltQueue.pop_front();
        YTemp = tipTiltQueue.front();
        tipTiltQueue.pop_front();
        curr_count = tipTiltQueue_2.front();
        tipTiltQueue_2.pop_front();
        nbImagesReceived = tipTiltQueue_2.front();
        tipTiltQueue_2.pop_front();
        pending = tipTiltQueue.size() / 2;
        tipTiltReady = false;
        ul.unlock();
        tipTiltConditionalVariable.notify_one();
        setVoltagesXY(XXShift, YYShift);
        sprintf(logString_2, "%llu, %llu, %lf, %lf, %lf, %lf, %d", curr_count, nbImagesReceived, XTemp, YTemp, XXShift, YYShift, pending);
        shift_log(logString_2);
    }
}

[[noreturn]] DWORD WINAPI closedLoopAutoGuiderThread(LPVOID lparam) {
    int XXShift, YYShift;
    while (true) {
        unique_lock<mutex> aul(autoGuiderMutex);
        autoGuiderConditionalVariable.wait(aul, [](){return autoGuiderReady;});
        XXShift = autoGuiderQueue.front();
        autoGuiderQueue.pop_front();
        YYShift = autoGuiderQueue.front();
        autoGuiderQueue.pop_front();
        autoGuiderReady = false;
        aul.unlock();
        autoGuiderConditionalVariable.notify_one();
        setMotorCount(2, sgn(XXShift), abs(XXShift));
        setMotorCount(1, sgn(YYShift), abs(YYShift));
    }
}

[[noreturn]] DWORD WINAPI displayThread(LPVOID lparam) {
    double* image = (double*) lparam;
    cv::namedWindow("Live!", cv::WINDOW_NORMAL);
    cv::resizeWindow("Live!", 512, 512);
    auto imageEightBit = new uint8_t[NX * NY];
    auto image64Bit = new double[NX * NY];
    double min, max;
    double multiplyFactor;
    uint64_t count;
    while (true) {
        unique_lock<mutex> dul(displayMutex);
        displayConditionalVariable.wait(dul, [](){return displayReady;});
//        count = displayCount;
        displayReady = false;
        dul.unlock();
        for (unsigned i=0;i < NX * NY; i++) {
            image64Bit[i] = image[i];
        }
        min = *(min_element(image64Bit, image64Bit + NPIX));
        max = *(max_element(image64Bit, image64Bit + NPIX));
        for (unsigned i=0;i < NX * NY; i++) {
            image64Bit[i] = image64Bit[i] - min;
        }
        max = max - min;
        multiplyFactor = 1 / max;
        for (unsigned i=0;i < NX * NY; i++) {
            imageEightBit[i] = 255 * image64Bit[i] * multiplyFactor;
        }
        const cv::Mat img(cv::Size(NX, NY), CV_8UC1, imageEightBit);
        cv::imshow("Live!", img);
        cv::waitKey(10);
//        FILE *fp;
//        string fname = string(SAVEPATH) + "//Curr_" +  to_string(count) + ".dat";
//        fopen_s(&fp, fname.c_str(), "wb");
//        fwrite(image64Bit, sizeof(*image64Bit), NX * NY, fp);
//        fclose(fp);
    }
}