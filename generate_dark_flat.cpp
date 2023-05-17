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

int main() {
    chrono::high_resolution_clock::time_point t0, t1;
    chrono::duration<double> dt;
    setupLogging(3);
    loggingfunc = log;
    initDev(1, 0, "Bias");
    int NFRAMES=100;
    ofstream ImOut;
    uint16_t *Image;
    uint16_t *binnedImage = (uint16_t*)calloc(NX*NY, sizeof(uint16_t));
    uint32_t *MeanImage = (uint32_t*)calloc(NX*NY, sizeof(uint32_t));
    std::string flatFilename = "MeanFlat";

    Err = openXYSerialPorts();
    COMMAND[14] = 13; // CR
    COMMAND[15] = 10; // LF
    sendCommand(XPort, "set,0,-20.0");
    sendCommand(YPort, "set,0,-20.0");
    Err = initDAQ();
    startCamera();

    fli->imageProcessing()->enableAutoClip(true);

    int counter = 0;
    for (int y=-20; y <= 130; y+= 5) {
        for (int x=-20;x<=130;x += 5) {
            setVoltagesXY(x, y);
            for (int j=0;j < NX*NY;j++) {
                MeanImage[j] = 0;
            }
            for (int i=0;i<NFRAMES;i++) {
                Image = (uint16_t*)fli->getRawImage();
                bin_separately(Image, binnedImage);
                for (int j=0;j < NX*NY;j++) {
                    MeanImage[j] += binnedImage[j];
                }
            }
            for (int j=0;j < NX*NY;j++) {
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