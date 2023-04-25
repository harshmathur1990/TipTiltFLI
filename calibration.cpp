#include "firstlight.h"
#include "imageprocess.h"
#include "controls.h"
#include "utilheaders.h"

using namespace std;

// Global variables : imageprocess
tuple<double, double> XYIND;
double CM[4];
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
int (*loggingfunc) (std::string) = NULL;
int (*shiftfunc) (std::string) = NULL;
// Global variables : serialcom
int FRAMENUMBER;
// Global variables : daqboard
// Global variables : general
auto TNow = chrono::system_clock::now();
//char SAVEPATH[64];
extern FliSdk* fli;
char logString[100000];
extern int Err;

int doCalib(int NFRAMES, int NUM_FRAME_PER_CALIB_POS, int AXIS);

int main () {
    setupLogging(1);
    std::sprintf(logString, "Allocating memory for various parameters");
    log(logString);
    // Memory allocation for external variables
    // Other variables
    int Loop=1, NFRAMES=10, NUM_FRAME_PER_CALIB_POS=1;
    double ExpTime = 0.0001;
    COMMAND[14] = 13; // CR
    COMMAND[15] = 10; // LF

    // Initialization for high speed
    std::sprintf(logString, "Initializing devices and acquiring one-time data");
    log(logString);
    initDev(1);
//    Err = getDarkFlat();
//    for(int i=0; i<NPIX; i++) MasterFlat[i] = 1.0;
    Err = getHammingWindow();
    Err = initializeFFT();
    if (Err != 0) {
        Err = stopDev();
        return -1;
    }
    Err = openXYSerialPorts();
    loggingfunc = log;
    sendCommand(XPort, "set,0, -20.0");
    sendCommand(YPort, "set,0, -20.0");

    startCamera();

    fli->imageProcessing()->enableAutoClip(true);
    while (Loop == 1) {
        Loop = 0;

        // Input: AOI (TRUE/FALSE), Global Shutter (TRUE/FALSE), Exposure time (Seconds)
//        cout << "Enter exposure time in seconds : ";
//        cin >> ExpTime;
//        cout.flush();
        // Acquisition and computation
        cout << "Number of frames : ";
        cin >> NFRAMES;
        cout.flush();
        cout << "Number of frames per calib position (default: 1): ";
        cin >> NUM_FRAME_PER_CALIB_POS;
        cout.flush();

        // Auto-save dir

        cout << "Starting X Calibration... " << endl;
        sprintf(logString, "X-Calibration Log");
        xlog(logString);
        sprintf(logString, "Number of frames are %d", NFRAMES);
        xlog(logString);
        sprintf(logString, "Number of frames per calib position are %d", NUM_FRAME_PER_CALIB_POS);
        xlog(logString);
        Err = doCalib(NFRAMES, NUM_FRAME_PER_CALIB_POS, 0);

        cout << "Starting Y Calibration... " << endl;
        sprintf(logString, "Y-Calibration Log");
        ylog(logString);
        sprintf(logString, "Number of frames are ", NFRAMES);
        ylog(logString);
        sprintf(logString, "Number of frames per calib position are %d", NUM_FRAME_PER_CALIB_POS);
        ylog(logString);
        Err = doCalib(NFRAMES, NUM_FRAME_PER_CALIB_POS, 1);

        // Option to continue acquisition
        cout << "Press 1 to continue calibration... ";
        cin >> Loop;
        cout.flush();

    }
    stopCamera();
//    fftw_destroy_plan(PlanForward);
//    fftw_destroy_plan(PlanInverse);
//    fftw_cleanup();
    Err = stopDev();
    Err = closeXYSerialPorts();
    cout << "Press a key and then enter to exit." << endl;
    char c;
    cin >> c;
    return 0;
}

int doCalib(int NFRAMES, int NUM_FRAME_PER_CALIB_POS, int AXIS){
    chrono::high_resolution_clock::time_point t0, t1;
    chrono::duration<double> dt;
    int i, j, k, ll;
    unsigned char Current = 0;
    string fname;
    FILE *fp;
    string Command;
    string Axis;
    uint16_t* Image = NULL;
    uint16_t* binnedImage = (uint16_t*)calloc(NPIX, sizeof(uint16_t));
    char tempLogString[10000];
    double shift_x=0, shift_y=0;
    if (AXIS==0) {
        Axis = "X";
        loggingfunc = &xlog;
        shiftfunc = &xvoltagefilelog;
    }
    else {
        Axis = "Y";
        loggingfunc = &ylog;
        shiftfunc = &yvoltagefilelog;
    }

    // DAQ
    Err = initDAQ();
    XShift = 0.0;
    YShift = 0.0;
    setVoltagesXY(XShift, YShift);

    // Set calibration range
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
    setVoltagesXY(XShift, YShift);
    Sleep(LONG_DELAY);

    // Start acquisition

    t0 = chrono::high_resolution_clock::now();
    i = 0;

    // Reference Image
    Image = (uint16_t*)fli->getRawImage();
    bin_separately(Image, binnedImage);
    fname = string(SAVEPATH) + "\\" + Axis + "_frame_" + string(6 - std::to_string(i+1).length(), '0') + to_string(i+1) + "_Ref.dat";
    sprintf(logString, "Image acquired : %s", fname.c_str());
    loggingfunc(logString);

    // Shift the image
    sprintf(logString, "%lf,%lf,0,0", XShift, YShift);
    shiftfunc(logString);

    if (AXIS == 0) XShift += VRANGE/NFRAMES;
    else YShift += VRANGE/NFRAMES;
    Err = setVoltagesXY(XShift, YShift);


    // Process the reference image
    Err = processReferenceImage(binnedImage, 0);
    fp = fopen(fname.c_str(), "wb");
    fwrite(binnedImage, sizeof(binnedImage), NPIX, fp);
    fclose(fp);
    Current++;
    double *CurrentImage;
    fftw_complex* CurrentImageFT;
    fftw_complex* CorrelatedImageFT;
    double* CorrelatedImage;
    if (MODE == INTEL_FFT) {
        CurrentImage = (double*) mkl_malloc(sizeof(double) * NX * NY, 64);
        CurrentImageFT = (fftw_complex* ) mkl_malloc(sizeof(fftw_complex) * NPIXFT, 64);
        CorrelatedImageFT = (fftw_complex*)mkl_malloc(sizeof(fftw_complex) * NPIXFT, 64);
        CorrelatedImage = (double*) mkl_malloc(sizeof(double ) * NX * NY, 64);
    }
    else {
        CurrentImage = (double*) fftw_malloc(sizeof(double) * NX * NY);
        CurrentImageFT = (fftw_complex* ) fftw_malloc(sizeof(fftw_complex) * NPIXFT);
        CorrelatedImageFT = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * NPIXFT);
        CorrelatedImage = (double*) calloc(sizeof(double ), NX * NY);
    }

    // Subsequent images
    for (i = 1; i < NFRAMES; i++) {
        shift_x=0;
        shift_y=0;
        sprintf(tempLogString, "%lf,%lf,", XShift, YShift);
        for (ll=0;ll<NUM_FRAME_PER_CALIB_POS;ll++) {
            Image = (uint16_t*)fli->getRawImage();
            bin_separately(Image, binnedImage);
            fname = string(SAVEPATH) + "\\" + Axis + "_frame_" + string(6 - std::to_string(i+1).length(), '0') + to_string(i+1) + "_" + string(6 - std::to_string(ll+1).length(), '0') + to_string(ll+1) + "_Cur.dat";
            sprintf(logString, "Image acquired : %s", fname.c_str());
            loggingfunc(logString);
            XYIND = getImageShift(
                    binnedImage, 0, 1,
                    CurrentImage, CurrentImageFT, CorrelatedImageFT, CorrelatedImage, ll);
            shift_x += get<0>(XYIND);
            shift_y += get<1>(XYIND);
            fp = fopen(fname.c_str(), "wb");
            fwrite(binnedImage, sizeof(*binnedImage), NPIX, fp);
            fclose(fp);
        }

        // Wait for next image
//        Image = (uint16_t*)fli->getRawImage();
//        bin_image(Image, binnedImage, width, height);
//        fname = string(SAVEPATH) + "\\" + Axis + "_frame_" + string(6 - std::to_string(i+1).length(), '0') + to_string(i+1) + "_Cur.dat";
//        sprintf(logString, "Image acquired : %s", fname.c_str());
//        loggingfunc(logString);

        // Apply shifts

//        sprintf(tempLogString, "%lf,%lf,", XShift, YShift);
        if (AXIS == 0) XShift += VRANGE/NFRAMES;
        else YShift += VRANGE/NFRAMES;
        FRAMENUMBER++;
        Err = setVoltagesXY(XShift, YShift);

        // Status bar
        cout << " ";
        for (k=0; k<i*50/NFRAMES; k++) cout << '=';
        cout << "  " << i*100/NFRAMES << "%   \r";
        cout.flush();

        // Process the current image
//        XYIND = getImageShift(binnedImage);
        sprintf(logString, "%s%lf,%lf", tempLogString, shift_x, shift_y);
        shiftfunc(logString);


        Current++;
    }
    cout << " ";
    for (k=0; k<50; k++) cout << '=';
    cout << "  100%\r" << endl;
    cout.flush();

    t1 = chrono::high_resolution_clock::now();
    dt = chrono::duration_cast<chrono::duration<double>>(t1 - t0);
    cout << "Number of frames: " << NFRAMES << ", In seconds: " << dt.count() << endl;
    cout << "Frame rate: " << NFRAMES/dt.count() << endl;
    sprintf(logString, "Number of frames: %d %s %d", NFRAMES,", In seconds: ", dt.count());
    loggingfunc(logString);
    sprintf(logString, "Frame rate: %d", NFRAMES/dt.count());
    loggingfunc(logString);
    Err = closeDAQ();
    return 0;
}