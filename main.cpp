#include "imageprocess.h"
#include "controls.h"
#include "firstlight.h"
#include "NIDAQmx.h"
#include <mutex>
#include "utilheaders.h"

using namespace std;

extern HANDLE XPort, YPort;
double A00, A01, A10, A11;
double Vxoff, Vyoff, SlewRate;
double Kp, Kd, Ki;
int Ni, Nd;
extern FliSdk* fli;
extern int Err;

char logString[100000];
int (*loggingfunc) (std::string) = NULL;
bool ready = false;
mutex g_mutex;
deque<float> volQueue;

inline void closedLoopCallBack(uint16_t* Image, uint64_t nbImagesReceived, double* binImgTime, double* imgWriteTime,
                        double* imgShiftTime, double* calcCorrectionTime, double* applyCorrectionTime,
                        uint64_t* curr_count, uint64_t* ignore_count, uint64_t NREFRESH, uint64_t* NumRefImage,
                        uint64_t* integralCounter, uint64_t* derivativeCounter, bool* breakLoop, uint64_t* counter,
                        uint64_t* status_count, float* XShift, float* YShift, uint64_t* error_no,
                        deque<double> &integralErrorX, deque<double> &integralErrorY,
                        deque<double> &derivativeErrorX, deque<double> &derivativeErrorY,
                        double *CurrentImage, fftw_complex *CurrentImageFT,
                        fftw_complex *CorrelatedImageFT, double *CorrelatedImage,
                        int fpsCamera, uint16_t* binnedImage, int ACQMODE,
                        uint16_t* rotatingCounter);
inline int getCorrection(double XTemp, double YTemp, deque<double> &integralErrorX, deque<double> &integralErrorY,
                  deque<double> &derivativeErrorX, deque<double> &derivativeErrorY, uint64_t *integralCounter,
                  uint64_t *derivativeCounter, double *correctionX, double *correctionY,
                  uint64_t curr_count, uint64_t nbImagesReceived
);
[[noreturn]] DWORD WINAPI closedLoopVoltageThread(LPVOID lparam);

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
	int  ACQMODE = 1, NREFREFRESH=100;

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

    // Go to offset point
    loggingfunc = oplog_write;
    sendCommand(XPort, "set,0,-20.0");
    sendCommand(YPort, "set,0,-20.0");
    Sleep(LONG_DELAY);
    sendCommand(XPort, "sr,0," + to_string(SlewRate));
    sendCommand(YPort, "sr,0," + to_string(SlewRate));
    cout << "Initializing DAQ..." << endl;
    Err = initDAQ();
//    XShift = Vxoff;
//    YShift = Vyoff;
//    cout << "Actuator going to offset point..." << endl;
//    Err = setVoltagesXY(XShift, YShift);
//    Sleep(LONG_DELAY);
    cout << "Enter acquisition mode, 1: with corrections, 2: only shifts-no corrections... ";
    cin >> ACQMODE;
    cout.flush();
    // Acquisition and computation
    cout.flush();

    cout << "Number of frames to refresh reference : ";
    cin >> NREFREFRESH;
    cout.flush();
    // Start
    sprintf(logString, "Number of frames to refresh reference : %d", NREFREFRESH);
    log(logString);
    cout.flush();
    string slopeString;
    unique_lock<mutex> ul(g_mutex);
    volQueue.push_back(Vxoff);
    volQueue.push_back(Vyoff);
    ready = true;
    ul.unlock();
    HANDLE myHandle;
    DWORD myThreadID;
    myHandle = CreateThread(0, 0, closedLoopVoltageThread, NULL, 0, &myThreadID);
    RawImageReceivedObserver obs(closedLoopCallBack, NREFREFRESH, Vxoff, Vyoff, ACQMODE);
    Sleep(1.5);
    startCamera();
    while(slopeString[0] != 'q') {
        cout << "Enter q to quit program" << endl;
        getline(std::cin, slopeString);
    }
    stopCamera();
    CloseHandle(myHandle);
//    CloseHandle(myHandle);
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

    toCorrectXTemp = -1 * XTemp;
    toCorrectYTemp = -1 * YTemp;

    double correctionShiftX, correctionShiftY;
    if (integralErrorX.size() == Ni) {
        integralErrorX.pop_front();
        integralErrorY.pop_front();
    }
    integralErrorX.push_back(toCorrectXTemp);
    integralErrorY.push_back(toCorrectYTemp);

    double integralErrorVoltageX = 0;
    double integralErrorVoltageY = 0;

    if (integralErrorX.size() < Ni) {
        integralErrorVoltageX = 0;
        integralErrorVoltageY = 0;
    } else {
        integralErrorVoltageX = accumulate(integralErrorX.end() - Nd, integralErrorX.end(), 0.0) / Ni;
        integralErrorVoltageY = accumulate(integralErrorY.end() - Nd, integralErrorY.end(), 0.0) / Ni;
    }

    if (derivativeErrorX.size() == Nd) {
        derivativeErrorX.pop_front();
        derivativeErrorY.pop_front();
    }
    derivativeErrorX.push_back(toCorrectXTemp);
    derivativeErrorY.push_back(toCorrectYTemp);

    double derivativeErrorVoltageX = 0;
    double derivativeErrorVoltageY = 0;

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


    correctionShiftX = Kp * toCorrectXTemp + Ki * integralErrorVoltageX + Kd * derivativeErrorVoltageX;
    correctionShiftY = Kp * toCorrectYTemp + Ki * integralErrorVoltageY + Kd * derivativeErrorVoltageY;

    *correctionX = A00*correctionShiftX + A01*correctionShiftY;
    *correctionY = A10*correctionShiftX + A11*correctionShiftY;

    if ((abs(toCorrectXTemp) >= 8) || (abs(toCorrectYTemp) >= 4)) {
        *correctionX = 0;
        *correctionY = 0;
    }
    sprintf(logString, "%ld, %ld, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf", curr_count, nbImagesReceived, toCorrectXTemp, toCorrectYTemp, integralErrorVoltageX, integralErrorVoltageY, derivativeErrorVoltageX, derivativeErrorVoltageY, correctionShiftX, correctionShiftY, *correctionX, *correctionY);
    shift_uncorected_log(logString);
    return 0;
}


inline void closedLoopCallBack(
        uint16_t* Image, uint64_t nbImagesReceived, double* binImgTime, double* imgWriteTime,
        double* imgShiftTime, double* calcCorrectionTime, double* applyCorrectionTime,
        uint64_t* curr_count, uint64_t* ignore_count,
        uint64_t NREFRESH, uint64_t* NumRefImage,
        uint64_t* integralCounter, uint64_t* derivativeCounter, bool* breakLoop, uint64_t* counter,
        uint64_t* status_count, float* XShift, float* YShift, uint64_t* error_no,
        deque<double> &integralErrorX, deque<double> &integralErrorY,
        deque<double> &derivativeErrorX, deque<double> &derivativeErrorY,
        double *CurrentImage, fftw_complex *CurrentImageFT,
        fftw_complex *CorrelatedImageFT, double *CorrelatedImage, int fpsCamera,
        uint16_t* binnedImage, int ACQMODE, uint16_t* rotatingCounter
        )
{
    chrono::high_resolution_clock::time_point t0, t1;
    chrono::duration<double> dt;

    if (*breakLoop == true) {
        return;
    }
    if (*rotatingCounter == fpsCamera) {
        *rotatingCounter = 0;
    }
    tuple<double, double> XYIND;
    int status, statusApply=0;
    double XTemp, YTemp;
    double correctionX, correctionY;

    int this_indice = *rotatingCounter;

    bin_separately(Image, binnedImage);

    if (*ignore_count) {
        *ignore_count = *ignore_count - 1;
        return;
    }
    if (*counter > NREFRESH) {
        *NumRefImage += 1;
        processReferenceImage(binnedImage, this_indice);
        *counter = 0;
    }
    else {
        XYIND = getImageShift(
                binnedImage, this_indice, fpsCamera,
                CurrentImage, CurrentImageFT, CorrelatedImageFT,
                CorrelatedImage, *curr_count);
        XTemp = get<0>(XYIND);
        YTemp = get<1>(XYIND);
        getCorrection(XTemp, YTemp, integralErrorX, integralErrorY,
                      derivativeErrorX, derivativeErrorY, integralCounter, derivativeCounter,
                      &correctionX, &correctionY, *curr_count, nbImagesReceived
        );

        if (ACQMODE == 1) {
            if ((*XShift + correctionX < -20) || (*XShift + correctionX > 130)) {
                *error_no = 1;
                cout << "error_no is "<< *error_no<<endl;
            }
            else if ((*YShift + correctionY < -20) || (*YShift + correctionY > 130)) {
                *error_no = 2;
                cout << "error_no is "<< *error_no<<endl;
            }
            else {
                *error_no = 0;
            }
            if (*error_no == 0) {
                *XShift += correctionX;
                *YShift += correctionY;
                unique_lock<mutex> ul(g_mutex);
                volQueue.push_back(*XShift);
                volQueue.push_back(*YShift);
                ready = true;
                ul.unlock();
                sprintf(logString, "%ld, %ld, %lf, %lf, %lf, %lf, %d", *curr_count, nbImagesReceived, -XTemp, -YTemp, *XShift, *YShift, statusApply);
                shift_log(logString);
            }
        }
        *counter += 1;
        *curr_count += 1;
    }
    *rotatingCounter += 1;
}

DWORD WINAPI closedLoopVoltageThread(LPVOID lparam) {
    float XXShift, YYShift;
    bool voltageSet = false;
    while (true) {
        unique_lock<mutex> ul(g_mutex);
        if (ready) {
            XXShift = volQueue.front();
            volQueue.pop_front();
            YYShift = volQueue.front();
            volQueue.pop_front();
            ready = false;
            voltageSet = true;
        }
        ul.unlock();
        if (voltageSet) {
            voltageSet = false;
            setVoltagesXY(XXShift, YYShift);
        }
    }
}