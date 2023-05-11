#include "imageprocess.h"
#include "controls.h"
#include "firstlight.h"
#include "NIDAQmx.h"
#include <mutex>
#include "utilheaders.h"
#include "motorcontrols.h"
#include "pid.h"

using namespace std;

extern HANDLE XPort, YPort;
double A00, A01, A10, A11;
double ClM00, ClM01, ClM10, ClM11;
double Vxoff, Vyoff, SlewRate;
double Kp, Kd, Ki;
int Ni, Nd;
double AA00, AA01, AA10, AA11; // Autoguider control matrix
extern FliSdk* fli;
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
PIDController pidX;
PIDController pidY;
char logString[100000],  logString_2[100000], logString_3[100000];
int (*loggingfunc) (std::string) = NULL;
bool tipTiltReady = false, autoGuiderReady = false, displayReady = false;
mutex tipTiltMutex, autoGuiderMutex;
deque<double> tipTiltQueue;
deque<uint64_t> tipTiltQueue_2;
deque<int> autoGuiderQueue;
std::condition_variable tipTiltConditionalVariable, autoGuiderConditionalVariable;
mutex displayMutex;
deque<double> displayQueue;
std::condition_variable displayConditionalVariable;

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
                        uint16_t* rotatingCounter, double *sumX, double *sumY,
                        int AUTOGUIDERMODE, double *sumVoltageX, double *sumVoltageY);
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
	int  ACQMODE = 1, NREFREFRESH=100, AUTOGUIDER=0;

    sprintf(logString, "Initializing devices and acquiring one-time data");
    log(logString);
    cout << "Searching for the cameras... " << endl;
	Err = initDev(1);
	if (Err != 0) {
	    exit(Err);
	}
//    Err = getDarkFlat();
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
    Err = getCalibrationMatrix();

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
//    sendCommand(XPort, "kp,0,0");
//    sendCommand(XPort, "ki,0,50");
//    sendCommand(XPort, "kd,0,0");
//    sendCommand(YPort, "kp,0,0");
//    sendCommand(YPort, "ki,0,50");
//    sendCommand(YPort, "kd,0,0");
    cout << "Initializing DAQ..." << endl;
    Err = initDAQ();
    setVoltagesXY(Vxoff, Vyoff);
//    Sleep(LONG_DELAY);
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
    tipTiltHandle = CreateThread(0, 0, closedLoopVoltageThread, NULL, 0, &tipTiltThreadID);
    autoguiderHandle = CreateThread(0, 0, closedLoopAutoGuiderThread, NULL, 0, &autoguiderThreadID);
//    displayHandle = CreateThread(0, 0, displayThread, NULL, 0, &displayThreadID);
    RawImageReceivedObserver obs(closedLoopCallBack, NREFREFRESH, Vxoff, Vyoff, ACQMODE, AUTOGUIDER);
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
    }
    stopCamera();
    CloseHandle(tipTiltHandle);
    CloseHandle(autoguiderHandle);
//    CloseHandle(displayHandle);
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
            integralErrorVoltageX = accumulate(integralErrorX.end() - Ni, integralErrorX.end(), 0.0) / Ni;
            integralErrorVoltageY = accumulate(integralErrorY.end() - Ni, integralErrorY.end(), 0.0) / Ni;
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
            integralErrorVoltageX = sumX / (curr_count + 1);
            integralErrorVoltageY = sumY / (curr_count + 1);
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
        uint16_t* const binnedImage, int ACQMODE, uint16_t* rotatingCounter,
        double *sumX, double *sumY, int AUTOGUIDERMODE, double *sumVoltageX, double *sumVoltageY
        )
{

    if (*breakLoop == true) {
        return;
    }
//    if (*rotatingCounter == fpsCamera) {
//        *rotatingCounter = 0;
//    }
    tuple<double, double> XYIND;
    double XTemp, YTemp;
    double correctionX, correctionY;

    bin_separately(Image, binnedImage);

    if (*ignore_count) {
        *ignore_count = *ignore_count - 1;
        return;
    }
    if (*counter > NREFRESH) {
        *NumRefImage += 1;
        processReferenceImage(binnedImage);
        *counter = 0;
    }
    else {
        XYIND = getImageShift(
                binnedImage,
                CurrentImage, CurrentImageFT, CorrelatedImageFT,
                CorrelatedImage, *curr_count);
        XTemp = get<0>(XYIND);
        YTemp = get<1>(XYIND);

        *sumX -= XTemp;
        *sumY -= YTemp;

        getCorrection(XTemp, YTemp, integralErrorX, integralErrorY,
                      derivativeErrorX, derivativeErrorY, integralCounter, derivativeCounter,
                      &correctionX, &correctionY, *curr_count, nbImagesReceived
        );

        if (*rotatingCounter == 10 * fpsCamera) {
            if (AUTOGUIDERMODE == 1) {
                unique_lock<mutex> aul(autoGuiderMutex);
                double meanVoltX = (*sumVoltageX / *curr_count) - 60;
                double meanVoltY = (*sumVoltageY / *curr_count) - 60;
                double pixelShiftX = ClM00 * meanVoltX + ClM01 * meanVoltY;
                double pixelShiftY = ClM10 * meanVoltX + ClM11 * meanVoltY;
                int autoCorX = int(0.5 * (AA00 * pixelShiftX + AA01 * pixelShiftY));
                int autoCorY = int(0.5 * (AA10 * pixelShiftX + AA11 * pixelShiftY));
                autoGuiderQueue.push_back(autoCorX);
                autoGuiderQueue.push_back(autoCorY);
                autoGuiderReady = true;
                aul.unlock();
                autoGuiderConditionalVariable.notify_one();
                aul.lock();
                autoGuiderConditionalVariable.wait(aul, [](){return !autoGuiderReady;});
                aul.unlock();
                sprintf(logString_3, "%lf, %lf, %d, %d", pixelShiftX, pixelShiftY, autoCorX, autoCorY);
                autoguider_log(logString_3);
            }
            *rotatingCounter = 0;
        }
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

DWORD WINAPI closedLoopVoltageThread(LPVOID lparam) {
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

DWORD WINAPI closedLoopAutoGuiderThread(LPVOID lparam) {
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
        setMotorCount(2, sgn(XXShift), XXShift);
        setMotorCount(1, sgn(YYShift), YYShift);
    }
}

DWORD WINAPI displayThread(LPVOID lparam) {
    cv::Mat img;
    double *image = (double*) malloc(sizeof(double) * NX * NY);
    while (true) {
        unique_lock<mutex> dul(displayMutex);
        displayConditionalVariable.wait(dul, [](){return displayReady;});
        for (unsigned ind =0; ind < NX * NY; ind++) {
            image[ind] = displayQueue.front();
            displayQueue.pop_front();
        }
        displayReady = false;
        dul.unlock();
        displayConditionalVariable.notify_one();
        const cv::Mat img(cv::Size(NX, NY), CV_64FC1, image);
        cv::imshow("Display Window", img);
        cv::waitKey(1);
    }
}