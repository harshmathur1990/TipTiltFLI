#include "firstlight.h"
#include "imageprocess.h"
#include "controls.h"
#include "common.h"

using namespace std;

// Global variables : imageprocess
double *MasterDark;
double *MasterFlat;
double *HammingWindow;
double *CurrentImage;
double *ReferenceImage;
double *CorrelatedImage;
fftw_plan PlanForward;
fftw_plan PlanInverse;
fftw_complex *CurrentImageFT;
fftw_complex *ReferenceImageFT;
fftw_complex *CorrelatedImageFT;
int XIND=0, YIND=0, MAXIND=16384;
double W0=0, WX=0, WY=0, WXY=0, ImageMean=0;
tuple<double, double, double, double> COEFF;
tuple<double, double> XYIND;
tuple<fftw_plan, fftw_plan> FFTWPlans;
double CM[4];
double XShift, YShift;
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
float64 XData[1] = {0.0};
float64 YData[1] = {0.0};
// Global variables : general
auto TNow = chrono::system_clock::now();
int Err = 1;
char SAVEPATH[64];
ofstream logfile;
ofstream shiftsfile;
extern FliSdk* fli;

int doCalib(int NFRAMES, int AXIS);

int main () {
    logfile.open(".\\log.txt");
    logfile << "Allocating memory for various parameters" << endl;
    // Memory allocation for external variables
    MasterDark = (double*) malloc(sizeof(double)*NPIX);
    MasterFlat = (double*) malloc(sizeof(double)*NPIX);
    HammingWindow = (double*) malloc(sizeof(double)*NPIX);
    CurrentImage = (double*) fftw_malloc(sizeof(double) * NPIX);
    ReferenceImage = (double*) fftw_malloc(sizeof(double) * NPIX);
    CorrelatedImage = (double*) fftw_malloc(sizeof(double) * NPIX);
    CurrentImageFT = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * NPIXFT);
    ReferenceImageFT = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * NPIXFT);
    CorrelatedImageFT = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * NPIXFT);
    Err = getDarkFlat();
    // Other variables
    int Loop=1, NFRAMES=10;
    double ExpTime = 0.0001;
    COMMAND[14] = 13; // CR
    COMMAND[15] = 10; // LF

    // Initialization for high speed
    logfile << "Initializing devices and acquiring one-time data" << endl;
    initDev(1);
    Err = getDarkFlat();
    for(int i=0; i<NPIX; i++) MasterFlat[i] = 1.0;
    Err = getHammingWindow();
    Err = getFFTWPlans();
    Err = openXYSerialPorts();
    sendCommand(XPort, "set,0,-20.0");
    sendCommand(YPort, "set,0,-20.0");

    logfile.close();

    startCamera();

    fli->imageProcessing()->enableAutoClip(true);
    while (Loop == 1) {
        Loop = 0;

        // Input: AOI (TRUE/FALSE), Global Shutter (TRUE/FALSE), Exposure time (Seconds)
        cout << "Enter exposure time in seconds : ";
        cin >> ExpTime;
        cout.flush();
        // Acquisition and computation
        cout << "Number of frames : ";
        cin >> NFRAMES;
        cout.flush();

        // Auto-save dir
        Err = getDateTimeDirName();
        Err = CreateDirectory(SAVEPATH ,NULL);

        cout << "Starting X Calibration... " << endl;
        logfile.open(string(SAVEPATH) + "\\Ylog.txt");
        logfile << "X-Calibration Log" << endl;
        logfile << "Exposure time is set to : " << ExpTime << " seconds" << endl;
        logfile << "Number of frames are " << NFRAMES << endl;
        Err = doCalib(NFRAMES, 0);
        logfile.close();

        cout << "Starting Y Calibration... " << endl;
        logfile.open(string(SAVEPATH) + "\\Xlog.txt");
        logfile << "Y-Calibration Log" << endl;
        logfile << "Exposure time is set to : " << ExpTime << "seconds" << endl;
        logfile << "Number of frames are " << NFRAMES << endl;
        Err = doCalib(NFRAMES, 1);
        logfile.close();

        // Option to continue acquisition
        cout << "Press 1 to continue calibration... ";
        cin >> Loop;
        cout.flush();

    }
    stopCamera();
    fftw_destroy_plan(PlanForward);
    fftw_destroy_plan(PlanInverse);
    fftw_cleanup();
    Err = stopDev();
    Err = closeXYSerialPorts();
    cout << "Press a key and then enter to exit." << endl;
    char c;
    cin >> c;
    return 0;
}

int doCalib(int NFRAMES, int AXIS){
    chrono::high_resolution_clock::time_point t0, t1;
    chrono::duration<double> dt;
    int i, j, k;
    unsigned char Current = 0;
    string fname;
    ofstream ImOut;
    string Command;
    string Axis;
    uint16_t* Image = NULL;
    if (AXIS==0) Axis="X" ;
    else Axis="Y" ;

    // DAQ
    Err = initDAQ();
    XShift = 0.0;
    YShift = 0.0;
    setVoltagesXY();

    // Set calibration range
    string ShiftFileName = Axis + "VoltageOnly.csv";
    shiftsfile.open(string(SAVEPATH) + "\\" + ShiftFileName);
    double VSTART = -10.0;
    cout << "Enter starting voltage for calibration (-20.0V to 110.0V)... ";
    cin >> VSTART;
    cout.flush();
    double VRANGE = 20.0;
    cout << "Enter voltage range for calibration (-130.0V to 130.0V)... ";
    cin >> VRANGE;
    cout.flush();
    double VOTHER = 0.0;
    cout << "Enter voltage for stationary axis... ";
    cin >> VOTHER;
    cout.flush();

    // Go to beginning of the range
    if (AXIS == 0){
        XShift = VSTART;
        YShift = VOTHER;
    }
    else {
        XShift = VOTHER;
        YShift = VSTART;
    }
    FRAMENUMBER = 1;
    setVoltagesXY();
    Sleep(LONG_DELAY);

    // Start acquisition

    t0 = chrono::high_resolution_clock::now();
    i = 0;

    // Reference Image
    Image = (uint16_t*)fli->getRawImage();
    fname = string(SAVEPATH) + "\\" + Axis + "_frame_" + string(6 - std::to_string(i+1).length(), '0') + to_string(i+1) + "_Ref.dat";
    logfile << "Image acquired : " << fname << endl;

    // Shift the image
    shiftsfile << XShift << "," << YShift << "," << 0 << "," << 0 << endl;
    // shiftsfile << ", XVoltage-YVoltage-XPixShift-YPixShift in that order" << endl;
    if (AXIS == 0) XShift += VRANGE/NFRAMES;
    else YShift += VRANGE/NFRAMES;
    Err = setVoltagesXY();

    // Process the reference image
    Err = processReferenceImage(Image);
    ImOut.open(fname);
    ImOut.write((char*)Image, 2*NPIX);
    ImOut.close();
    Current++;

    // Subsequent images
    for (i = 1; i < NFRAMES; i++) {

        // Wait for next image
        Image = (uint16_t*)fli->getRawImage();
        fname = string(SAVEPATH) + "\\" + Axis + "_frame_" + string(6 - std::to_string(i+1).length(), '0') + to_string(i+1) + "_Cur.dat";
        logfile << "Image acquired : " << fname << endl;

        // Apply shifts
        shiftsfile << XShift << "," << YShift << ",";
        if (AXIS == 0) XShift += VRANGE/NFRAMES;
        else YShift += VRANGE/NFRAMES;
        FRAMENUMBER++;
        Err = setVoltagesXY();

        // Status bar
        cout << " ";
        for (k=0; k<i*50/NFRAMES; k++) cout << '=';
        cout << "  " << i*100/NFRAMES << "%   \r";
        cout.flush();

        // Process the current image
        XYIND = getImageShift(Image);
        shiftsfile << get<0>(XYIND) << "," << get<1>(XYIND) << endl;
        ImOut.open(fname);
        ImOut.write((char*)Image, 2*NPIX);
        ImOut.close();
        Current++;
    }
    cout << " ";
    for (k=0; k<50; k++) cout << '=';
    cout << "  100%\r" << endl;
    cout.flush();
    shiftsfile.close();
    t1 = chrono::high_resolution_clock::now();
    dt = chrono::duration_cast<chrono::duration<double>>(t1 - t0);
    cout << "Number of frames: " << NFRAMES << ", In seconds: " << dt.count() << endl;
    cout << "Frame rate: " << NFRAMES/dt.count() << endl;
    logfile << "Number of frames: " << NFRAMES << ", In seconds: " << dt.count() << endl;
    logfile << "Frame rate: " << NFRAMES/dt.count() << endl;

    // Copy the shifts file to current directory for immediate use
    ifstream  SOURCE(string(SAVEPATH) + "\\" + ShiftFileName, ios::binary);
    ofstream  DESTINY(ShiftFileName, ios::binary);
    DESTINY << SOURCE.rdbuf();
    SOURCE.close();
    DESTINY.close();
    Err = closeDAQ();
    return 0;
}


