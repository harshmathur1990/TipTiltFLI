#include "controls.h"
#include "utilheaders.h"
#include <mutex>
int XIND=0, YIND=0, MAXIND=16384;
double W0=0, WX=0, WY=0, WXY=0, ImageMean=0;
tuple<double, double, double, double> COEFF;
tuple<double, double> XYIND;
double CM[4];
double XShift, YShift;
double A00;
double A01;
double A10;
double A11;
double ClM00, ClM01, ClM10, ClM11;
double Vxoff;
double Vyoff;
double SlewRate;
double Kp;
double Kd;
double Ki;
double Akp, Aki, Akd;
int autoGuiderCorrectionTime;
int Nd;
int Ni;
double AA00, AA01, AA10, AA11; // Autoguider control matrix
double autoGuiderOffloadLimitX, autoGuiderOffloadLimitY;
double tau;
int imageSaveAfterSecond;
/* Output limits */
double limMin;
double limMax;

/* Integrator limits */
double limMinInt;
double limMaxInt;

/* Sample time (in seconds) */
double sampleTime;
// Global variables : serialcom

bool COMSEND = FALSE;
bool STARTTHREAD = TRUE;
int FRAMENUMBER;

// Global variables : daqboard

// Global variables : general
auto TNow = chrono::system_clock::now();
extern int Err;
char SAVEPATH[64];
ofstream logfile;
ofstream shiftsfile;
int (*loggingfunc) (std::string)=NULL;
char logString[100000];
bool displayReady = false;
mutex displayMutex;
deque<double**> displayQueue;
std::condition_variable displayConditionalVariable;
int liveView;
bool useCameraFlat;
string NucMode;

int main() {
    setupLogging(3);
    loggingfunc = log;
    int a;
    Err = openXYSerialPorts();
    if (Err != 0) {
        std::cout<<"Couldn't open serial port"<<Err<<std::endl;
    }
    COMMAND[14] = 13; // CR
    COMMAND[15] = 10; // LF
    sendCommand(XPort, "set,0, -20.0");
    sendCommand(YPort, "set,0, -20.0");
//    Sleep(10 * 1000);
    Err = initDAQ();
    XShift = 0.0;
    YShift = 0.0;
    setVoltagesXY(XShift, YShift);
    while(1) {
        std::cout<<"Enter Xshift and Yshift"<<std::endl;
        std::cin>>XShift;
        std::cin>>YShift;
        std::cout<<"XShift: "<<XShift<<"  YShift: "<<YShift<<std::endl;
        Err = setVoltagesXY(XShift, YShift);
        std::cout<<"Press 1 to continue entering voltages"<<std::endl;
        std::cin>>a;
        if (a != 1)
            break;
    }
    XShift = -20;
    YShift = -20;
    setVoltagesXY(XShift, YShift);
    Err = closeDAQ();
    Err = closeXYSerialPorts();
    return 0;
}