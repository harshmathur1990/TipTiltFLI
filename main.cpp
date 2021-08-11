#include "common.h"
#include "imageprocess.h"
#include "controls.h"
#include "firstlight.h"
#include "NIDAQmx.h"

using namespace std;

// Acquisition
ofstream logfile;
auto TNow = chrono::system_clock::now(); 
int Err = 1;
char SAVEPATH[64];
// Processing
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
double A00, A01, A10, A11;
double Vxoff, Vyoff, SlewRate;
double Kp, Kd, Ki;
int Ni, Nd;
double XShift, YShift;
// COM port
HANDLE XPort, YPort;
char COMMAND[16];
DWORD dwBytesWritten;
int nWriteBytes;
string XCommand, YCommand;
LPDWORD COMMERROR;
LPCOMSTAT COMMSTATUS;
// DAQ board
TaskHandle XDAQHandle;
TaskHandle YDAQHandle;
float64 XData[1];
float64 YData[1];
// Controls related
deque<double> XPrevCorr;
deque<double> YPrevCorr;
deque<double> IPrevCorr; // Index
double XPErr = 0, YPErr = 0;
double XIErr = 0, YIErr = 0;
double XDErr = 0, YDErr = 0;
double XModVolts[1], YModVolts[1];
extern FliSdk* fli;

int applyShifts(double Dpar1, double Dpar2);


class RawImageReceivedObserver : public IRawImageReceivedObserver
{
public:
    RawImageReceivedObserver(uint16_t width, uint16_t height) :
            _imgSize(width* height),
            _nbImagesReceived(0)
    {
        for (i=0; i<Ni; i++){
            XPrevCorr.push_back(0.00);
            YPrevCorr.push_back(0.00);
            IPrevCorr.push_back(i);
        }
        Dpar1 = -6.0/(Nd*(Nd+1));
        Dpar2 = 12.0/(Nd*(Nd-1)*(Nd+1));
        cout << "Loading the calibration matrix... " << endl;
        Err = getCalibrationMatrix();
        XModVolts[0] = (Vxoff+20.0)/15.0;
        YModVolts[0] = (Vyoff+20.0)/15.0;
        shiftsfile.open("." + string(SAVEPATH) + "_Shifts.csv");
        cout << "Shifts are saved in " << "." + string(SAVEPATH) + "_Shifts.csv" << endl;
        fli->addRawImageReceivedObserver(this);
    };

    virtual void imageReceived(const uint8_t* image) override
    {
        uint16_t *Image = (uint16_t *) image;
        logfile << _nbImagesReceived << " Image acquired : " << fname << endl;
        if (referenceImage == NULL || _nbImagesReceived > NREFREFRESH) {
            NumRefImage += 1;
            fname = string(SAVEPATH) + "\\frame_" + string(6 - std::to_string(_nbImagesReceived+1).length(), '0') + to_string(_nbImagesReceived+1) + "_ref.dat";
            ImOut.open(fname);
            ImOut.write((char*)Image, 2*NPIX);
            ImOut.close();
            processReferenceImage(Image);
            logfile << NumRefImage << " Reference image processed" << endl;
        }
        else {
            XYIND = getImageShift(Image);
            logfile << _nbImagesReceived << " Current image processed" << '\n';
            XTemp = get<0>(XYIND);
            YTemp = get<1>(XYIND);
            XPErr = A00*XTemp + A01*YTemp;
            YPErr = A10*XTemp + A11*YTemp;
            if (abs(XPErr)>NX/2) XPErr=0;
            if (abs(YPErr)>NY/2) YPErr=0;
            applyShifts(Dpar1, Dpar2);
            shiftsfile <<XTemp << "," << YTemp << "," << XPErr << "," << YPErr << endl ;
        }
        Err = setProgressBar(_nbImagesReceived, NFRAMES);
        _nbImagesReceived++;
    }

    virtual uint16_t fpsTrigger() override
    {
        return 0;
    }

    uint32_t getNbImagesReceived()
    {
        return _nbImagesReceived;
    };

private:
    int i;
    uint32_t _imgSize;
    uint32_t _nbImagesReceived;
    uint32_t NumRefImage=0;
    uint16_t* referenceImage=NULL;
    uint64_t NFRAMES, NREFREFRESH;
    ofstream ImOut, shiftsfile;
    string fname;
    double XTemp, YTemp;
    double XPErr = 0, YPErr = 0;
    double Dpar1, Dpar2;
};


int main () {
    logfile.open("log.txt");
    logfile << "Allocating memory for various parameters" << endl;

    // Allocation
    MasterDark = (double*) malloc(sizeof(double)*NPIX);
    MasterFlat = (double*) malloc(sizeof(double)*NPIX);
    HammingWindow = (double*) malloc(sizeof(double)*NPIX);
    CurrentImage = (double*) fftw_malloc(sizeof(double) * NPIX);
    ReferenceImage = (double*) fftw_malloc(sizeof(double) * NPIX);
    CorrelatedImage = (double*) fftw_malloc(sizeof(double) * NPIX);
    CurrentImageFT = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * NPIXFT);
    ReferenceImageFT = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * NPIXFT);
    CorrelatedImageFT = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * NPIXFT);
    COMMAND[14] = 13; // CR
    COMMAND[15] = 10; // LF

    // Other variables
	int Loop=1, ACQMODE = 1, NFRAMES=100, NREFREFRESH=100;
	double ExpTime=0.0001;

	// Initialization for high speed
    logfile << "Initializing devices and acquiring one-time data" << endl;
    cout << "Searching for the cameras... " << endl;
	Err = initDev();
    Err = getDarkFlat();
    cout << "Creating the window function... " << endl;
    Err = getHammingWindow();
    cout << "Creating the Fourier transform plans... " << endl;
    Err = getFFTWPlans();
    cout << "Opening the serial COM ports... " << endl;
    Err = openXYSerialPorts();
    cout << "Loading the calibration matrix... " << endl;
    Err = getCalibrationMatrix();
    logfile.close();

    // Go to offset point
    sendCommand(XPort, "set,0,-20.0");
    sendCommand(YPort, "set,0,-20.0");
    Sleep(LONG_DELAY);
    sendCommand(XPort, "sr,0," + to_string(SlewRate));
    sendCommand(YPort, "sr,0," + to_string(SlewRate));
    cout << "Initializing DAQ..." << endl;
    Err = initDAQ();
    XShift = Vxoff;
    YShift = Vyoff;
    cout << "Actuator going to offset point..." << endl;
    Err = setVoltagesXY();
    Sleep(LONG_DELAY);

    Loop = 0;
    cout << "Enter acquisition mode, 1: with corrections, 2: only shifts-no corrections... ";
    cin >> ACQMODE;
    cout.flush();
    // Acquisition and computation
    cout << "Number of frames : ";
    cin >> NFRAMES;
    cout.flush();
    cout << "Number of frames to refresh reference : ";
    cin >> NREFREFRESH;
    cout.flush();
    // Start
    Err = getDateTimeDirName();
    Err = CreateDirectory(SAVEPATH,NULL);
    logfile.open(string(SAVEPATH) + "\\log.txt");
    logfile << "Number of frames : " << NFRAMES << endl;
    logfile << "Number of frames to refresh reference : " << NREFREFRESH << endl;
    logfile.close();
    RawImageReceivedObserver obs(width, height);
    cout.flush();
    // Clearing data
    XShift = Vxoff;
    YShift = Vyoff;
    cout << "Actuator going to offset point..." << endl;
    Err = setVoltagesXY();
    Err = closeDAQ();
    XShift = 0.0;
    YShift = 0.0;
    sendCommand(XPort, "set,0,0.0");
    sendCommand(YPort, "set,0,0.0");
    fftw_destroy_plan(PlanForward);
    fftw_destroy_plan(PlanInverse);
    fftw_cleanup();
	Err = stopDev();
	cout << "Press a key and then enter to exit." << endl;
	char c;
	cin >> c;
	return 0;
}

int applyShifts(double Dpar1, double Dpar2){
    ofstream shiftsfile;
    shiftsfile.open("." + string(SAVEPATH) + "_Shifts_Uncorrected.csv");
    // Stack up
    XPrevCorr.push_back(XPErr);
    YPrevCorr.push_back(YPErr);
    XPrevCorr.pop_front();
    YPrevCorr.pop_front();
    // // Compute error
    XIErr = accumulate(XPrevCorr.end()-Ni, XPrevCorr.end(), 0.0)/Ni;
    YIErr = accumulate(YPrevCorr.end()-Ni, YPrevCorr.end(), 0.0)/Ni;
    XDErr = Dpar1*accumulate(XPrevCorr.end()-Nd, XPrevCorr.end(), 0.0) + Dpar2*inner_product(XPrevCorr.end()-Nd, XPrevCorr.end(), IPrevCorr.begin(), 0.0);
    YDErr = Dpar1*accumulate(YPrevCorr.end()-Nd, YPrevCorr.end(), 0.0) + Dpar2*inner_product(YPrevCorr.end()-Nd, YPrevCorr.end(), IPrevCorr.begin(), 0.0);
    // Final shift apply
    XModVolts[0] -= (Kp*XPErr + Ki*XIErr + Kd*XDErr)/15.0;
    YModVolts[0] -= (Kp*YPErr + Ki*YIErr + Kd*YDErr)/15.0;
    logfile << "Modulation voltage for X : " << XModVolts[0] << '\n';
    logfile << "Modulation voltage for Y : " << YModVolts[0] << '\n';
    logfile << "Applying voltage for X : " << DAQmxWriteAnalogF64(XDAQHandle, 1, 1, 0.0, DAQmx_Val_GroupByChannel, XModVolts, NULL, NULL) << '\n';
    logfile << "Applying voltage for Y : " << DAQmxWriteAnalogF64(YDAQHandle, 1, 1, 0.0, DAQmx_Val_GroupByChannel, YModVolts, NULL, NULL) << '\n';
    shiftsfile << get<0>(XYIND) << "," << get<1>(XYIND) << "," ;
    shiftsfile << XPErr << "," << YPErr << "," ;
    shiftsfile << XIErr << "," << YIErr << "," ;
    shiftsfile << XDErr << "," << YDErr << endl ;
    return 0;
    return 0;
}



    
