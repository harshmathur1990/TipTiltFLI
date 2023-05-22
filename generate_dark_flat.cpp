#include <iostream>
#include <cstdlib>
#include <fstream>
#include <string>
#include "firstlight.h"
#include "controls.h"
#include "imageprocess.h"
#include "utilheaders.h"
double AA00, AA01, AA10, AA11;
double autoGuiderOffloadLimitX, autoGuiderOffloadLimitY;
int autoGuiderCorrectionTime;
int imageSaveAfterSecond;

extern FliSdk* fli;
extern uint16_t width, height;
//fftw_plan PlanForward;
//fftw_plan PlanInverse;
//fftw_complex *ReferenceImageFT;
float XShift, YShift;
double ClM00, ClM01, ClM10, ClM11;
double A00, A01, A10, A11;
double tau;

/* Output limits */
double limMin;
double limMax;

/* Integrator limits */
double limMinInt;
double limMaxInt;

/* Sample time (in seconds) */
double sampleTime;
double Vxoff;
double Akp, Aki, Akd;
double Vyoff;
double SlewRate;
double Kp;
double Kd;
double Ki;
int Nd;
int Ni;
// Global variables : serialcom
bool COMSEND = FALSE;
bool STARTTHREAD = TRUE;
int FRAMENUMBER;
// Global variables : daqboard
// Global variables : general
auto TNow = chrono::system_clock::now();
//char SAVEPATH[64];
ofstream logfile;
ofstream shiftsfile;
int (*loggingfunc) (std::string) = NULL;
char logString[100000];
int liveView;
bool useCameraFlat;
string NucMode;
[[noreturn]] DWORD WINAPI displayThread(LPVOID lparam);
uint16_t* image;
uint16_t *binnedImage;
uint32_t *MeanImage;

bool displayReady;
mutex displayMutex;
std::condition_variable displayConditionalVariable;

int main() {
    chrono::high_resolution_clock::time_point t0, t1;
    chrono::duration<double> dt;
    setupLogging(3);
    loggingfunc = log;
    initDev(1, 0, "Bias");
    int NFRAMES=100;
    ofstream ImOut;
    binnedImage = (uint16_t*)calloc(NX*NY, sizeof(uint16_t));
    MeanImage = (uint32_t*)calloc(NX*NY, sizeof(uint32_t));
    std::string flatFilename = "MeanFlat";
    DWORD displayThreadID;
    HANDLE displayHandle;
    Err = openXYSerialPorts();
    COMMAND[14] = 13; // CR
    COMMAND[15] = 10; // LF
    sendCommand(XPort, "set,0,-20.0");
    sendCommand(YPort, "set,0,-20.0");
    Err = initDAQ();
    startCamera();

    fli->imageProcessing()->enableAutoClip(true);
//    displayHandle = CreateThread(0, 0, displayThread, NULL, 0, &displayThreadID);
    int counter = 0;
    for (int y=-20; y <= 130; y+= 5) {
        for (int x=-20;x<=130;x += 5) {
            setVoltagesXY(x, y);
            for (unsigned int m=0; m < NX*NY; m++) {
                MeanImage[m] = 0;
            }
            for (unsigned int i=0;i<NFRAMES;i++) {
                image = (uint16_t*)fli->getRawImage();
                bin_separately(image, binnedImage);
//                unique_lock<mutex> dul(displayMutex);
//                displayReady = true;
//                dul.unlock();
//                displayConditionalVariable.notify_one();
//                dul.lock();
//                displayConditionalVariable.wait(dul, [](){return !displayReady;});
//                dul.unlock();
                for (unsigned int k=0;k < NX*NY;k++) {
                    MeanImage[k] = MeanImage[k] + *(binnedImage + k);
                }
            }
            for (unsigned int j=0;j < NX*NY;j++) {
                binnedImage[j] = MeanImage[j] / NFRAMES;
            }
            FILE *fp;
            string fname = string("Flats") + "\\" + flatFilename + "_" + to_string(x) + "_" + to_string(y) + ".dat";
            fopen_s(&fp, fname.c_str(), "wb");
            fwrite(binnedImage, sizeof(*binnedImage), NX * NY, fp);
            fclose(fp);

            cout << "Progress:  " << counter*100/(LENGTHFLATARR * LENGTHFLATARR) << " %\r";
            cout.flush();
            counter += 1;
        }
    }

    stopCamera();
//    CloseHandle(displayHandle);
    t1 = chrono::high_resolution_clock::now();
    dt = chrono::duration_cast<chrono::duration<double>>(t1 - t0);

    cout << "Progress:  " << counter*100/(LENGTHFLATARR * LENGTHFLATARR) << " % FPS: "<< counter / dt.count()<<endl;
    XShift = -20;
    YShift = -20;
    setVoltagesXY(XShift, YShift);
    Err = closeDAQ();
    stopDev();
    Err = closeXYSerialPorts();

    cout << "Enter a key to exit"<<endl;
    cin>>NFRAMES;
    return 0;
}

DWORD WINAPI displayThread(LPVOID lparam) {
    cv::namedWindow("Live!", cv::WINDOW_NORMAL);
    cv::resizeWindow("Live!", NX, NY);
    auto imageEightBit = new uint8_t[NX * NY];
    auto image32Bit = new uint32_t[NX * NY];
    double min, max;
    double multiplyFactor;
    while (true) {
        unique_lock<mutex> dul(displayMutex);
        displayConditionalVariable.wait(dul, [](){return displayReady;});
        for (unsigned i=0;i < NX * NY; i++) {
            image32Bit[i] = MeanImage[i];
        }
        displayReady = false;
        dul.unlock();
        displayConditionalVariable.notify_one();
        min = *(min_element(image32Bit, image32Bit + NPIX));
        max = *(max_element(image32Bit, image32Bit + NPIX));
        for (unsigned i=0;i < NX * NY; i++) {
            image32Bit[i] = image32Bit[i] - min;
        }
        max = max - min;
        multiplyFactor = 1 / max;
        for (unsigned i=0;i < NX * NY; i++) {
            imageEightBit[i] = 255 * image32Bit[i] * multiplyFactor;
        }
        const cv::Mat img(cv::Size(NX, NY), CV_8UC1, imageEightBit);
        cv::imshow("Live!", img);
        cv::waitKey(1);
    }
}