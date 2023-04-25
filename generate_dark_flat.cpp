#include <iostream>
#include <cstdlib>
#include <fstream>
#include <string>
#include "firstlight.h"
#include "controls.h"
#include "imageprocess.h"
#include "utilheaders.h"
#define DARK 0
#define FLAT 1
#define count(x) sizeof(x)/sizeof(uint16_t)

extern FliSdk* fli;
extern uint16_t width, height;
double *MasterDark;
double *MasterFlat;
double *HammingWindow;
double *ReferenceImage;
//fftw_plan PlanForward;
//fftw_plan PlanInverse;
//fftw_complex *ReferenceImageFT;
float XShift, YShift;
double A00;
double A01;
double A10;
double A11;
double Vxoff;
double Vyoff;
double SlewRate;
double Kp;
double Kd;
double Ki;
int Nd;
int Ni;
// Global variables : serialcom
string XCommand;
string YCommand;
bool COMSEND = FALSE;
bool STARTTHREAD = TRUE;
int FRAMENUMBER;
LPDWORD COMMERROR;
LPCOMSTAT COMMSTATUS;
HANDLE XPort, YPort;
char COMMAND[16];
DWORD dwBytesWritten;
int nWriteBytes;
// Global variables : daqboard
TaskHandle XDAQHandle=0;
TaskHandle YDAQHandle=0;
// Global variables : general
auto TNow = chrono::system_clock::now();
int Err = 1;
//char SAVEPATH[64];
ofstream logfile;
ofstream shiftsfile;
int (*loggingfunc) (std::string) = NULL;
char logString[100000];

int main() {
    setupLogging(3);
    loggingfunc = log;
    initDev(1);
    int mode = -1, NFRAMES=0, i, j, k;
    ofstream ImOut;
    uint16_t *Image;
    uint32_t *binnedImage = (uint32_t*)calloc(NX*NY, sizeof(uint32_t));
    uint64_t *MeanImage = (uint64_t*)calloc(NX*NY, sizeof(uint64_t));
    std::string darkFilename = "MeanDark.dat";
    std::string flatFilename = "MeanFlat.dat";

    while(mode != DARK && mode != FLAT) {
        std::cout<<"Enter dark(0) or flat(1)"<<std::endl;
        std::cin>>mode;
    }

    if (mode == DARK) {
        std::cout<<"Please close the shutter of the camera"<<std::endl;
    }
    else {
        std::cout<<"Please move the Image about the disc center"<<std::endl;
    }

    std::cout<<"Please enter the number of frames to be captured"<<std::endl;
    std::cin>>NFRAMES;
    Err = openXYSerialPorts();
    COMMAND[14] = 13; // CR
    COMMAND[15] = 10; // LF
    sendCommand(XPort, "set,0,-20.0");
    sendCommand(YPort, "set,0,-20.0");
    Err = initDAQ();
    std::cout<<"Enter Xshift and Yshift"<<std::endl;
    std::cin>>XShift;
    std::cin>>YShift;
    std::cout<<"XShift: "<<XShift<<"  YShift: "<<YShift<<std::endl;
    setVoltagesXY(XShift, YShift);
    startCamera();

    fli->imageProcessing()->enableAutoClip(true);

    for (i=0;i<NFRAMES;i++) {
        Image = (uint16_t*)fli->getRawImage();
        bin_image(Image, binnedImage, width, height);
        //cout << "Image Size: " << count(Image) << endl;
        for (j=0;j < NX*NY;j++) {
            MeanImage[j] += binnedImage[j];
        }
        cout << " ";
        for (k=0; k<i*50/NFRAMES; k++) cout << '=';
        cout << "  " << i*100/NFRAMES << "%   \r";
        cout.flush();
    }
    stopCamera();
    XShift = -20;
    YShift = -20;
    setVoltagesXY(XShift, YShift);
    Err = closeDAQ();
    stopDev();
    Err = closeXYSerialPorts();

    for (j=0;j < NX*NY;j++) {
        MeanImage[j]/= NFRAMES;
    }

    if (mode == DARK)  ImOut.open(darkFilename);
    else  ImOut.open(flatFilename);
//    ImOut.write((char*)MeanImage, sizeof(uint16_t)*width*height);
//    ImOut << "Test String" << "\n";
    for (j=0;j < NX*NY;j++) {
        ImOut<<MeanImage[j]<<"\n";
    }
    ImOut.close();


    cout<<endl;
    if (mode == DARK)
        std::cout<<"Wrote "<<darkFilename<<std::endl;
    else
        std::cout<<"Wrote "<<flatFilename<<std::endl;

    return 0;
}