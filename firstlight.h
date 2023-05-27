#include "FliSdk.h"
#define LONG_DELAY 2000
#include <vector>
#include <iostream>
#include <chrono>
#include <deque>
#include <mkl.h>
#include <fftw3.h>
#include "utilheaders.h"
#include "mkl_data_types.h"
#include "imageheaders.h"

using namespace std;

FliSdk* fli;
int nbImages = 0;
int fpsCamera;
char SAVEPATH[64];


class RawImageReceivedObserver : public IRawImageReceivedObserver
{
public:
    RawImageReceivedObserver(
            void (*workFunction)(uint16_t* const Image, uint64_t nbImagesReceived, double* binImgTime, double* imgWriteTime,
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
                                 uint64_t *imageSaveCounter, double *integralVoltX, double * integralVoltY,
                                 bool *updateReference, bool *offloadShiftsToAutoguider, uint32_t* autoGuiderCounter,
                                 double* meanVoltX, double* meanVoltY, double* previousErrorX, double* previousErrorY
            ), uint64_t NREFERESH, double XShift, double YShift,
            int ACQMODE, int AUTOGUIDERMODE,
            int imageSaveAfterSecond):
            workFunction(workFunction),
            NREFRESH(NREFERESH),
            XShift(XShift), YShift(YShift),
            ACQMODE(ACQMODE),
            counter(NREFERESH + 1),
            _nbImagesReceived(0),
            t0(chrono::high_resolution_clock::now()),
            binImgTime(0), imgWriteTime(0), imgShiftTime(0),
            calcCorrectionTime(0), applyCorrectionTime(0),
            integralCounter(0), derivativeCounter(0),
            curr_count(0), ignore_count(0), NumRefImage(0),
            status_count(0), error_no(0),
            breakLoop(false), printStats(true),
            rotatingCounter(0), sumX(0), sumY(0), AUTOGUIDERMODE(AUTOGUIDERMODE),
            imageSaveCounter(0), integralVoltX(0), integralVoltY(0), updateReference(true),
            autoGuiderCounter(fpsCamera), offloadShiftsToAutoguider(false),
            meanVoltX(0), meanVoltY(0), previousErrorX(0), previousErrorY(0)
    {
        if (MODE == INTEL_FFT) {
            int alignment=64;
            CurrentImage = (double*)mkl_malloc(sizeof(double) * NX * NY, alignment);
            CurrentImageFT = (fftw_complex*) mkl_malloc(
                    NPIXFT * sizeof(fftw_complex), alignment);
            CorrelatedImageFT = (fftw_complex*) mkl_malloc(
                    NPIXFT * sizeof(fftw_complex), alignment);
            CorrelatedImage = (double*)mkl_malloc(sizeof(double) * NX * NY, alignment);
            binnedImage = (uint16_t*)mkl_calloc(NX * NY, sizeof(uint16_t), alignment);
        }
        else {
            CurrentImage = (double*)fftw_malloc(sizeof(double) * NX * NY);
            CurrentImageFT = (fftw_complex*) fftw_malloc(
                    NPIXFT * sizeof(fftw_complex));
            CorrelatedImageFT = (fftw_complex*) fftw_malloc(
                    NPIXFT * sizeof(fftw_complex));
            CorrelatedImage = (double*)fftw_malloc(sizeof(double) * NX * NY);
            binnedImage = (uint16_t*)calloc(NX * NY, sizeof(uint16_t));
        }

        fli->addRawImageReceivedObserver(this);
    };

    virtual void imageReceived(const uint8_t* image) override
    {
        if (this->_nbImagesReceived == 0) {
            t0 = chrono::high_resolution_clock::now();
        }
        if (printStats & ((!(this->_nbImagesReceived & ((1 << 12) - 1) )) || breakLoop)) {
            t1 = chrono::high_resolution_clock::now();
            dt = chrono::duration_cast<chrono::duration<double>>(t1 - t0);
            fps = (_nbImagesReceived) / dt.count();
            if (breakLoop == true) {
                printStats = false;
                cout << "error_no: "<< error_no << endl;
            }
            cout << "FPS: "<< fps << " Vx: "<< XShift<< " Vy: "<< YShift<< " _nbImagesReceived :" << _nbImagesReceived << " NumRefImage: "<<NumRefImage<<"\r";
            cout.flush();
        }
        this->imagePointer = (uint16_t *) image;
        this->_nbImagesReceived = this->_nbImagesReceived + 1;
//        cout << "Receiving Images" << this->_nbImagesReceived << "\r" << endl;
        workFunction(
            this->imagePointer, this->_nbImagesReceived, &binImgTime,
            &imgWriteTime, &imgShiftTime, &calcCorrectionTime, &applyCorrectionTime,
            &curr_count, &ignore_count, NREFRESH, &NumRefImage, &integralCounter, &derivativeCounter, &breakLoop,
            &counter, &status_count, &XShift, &YShift, &error_no,
            integralErrorX, integralErrorY, derivativeErrorX, derivativeErrorY,
            CurrentImage, CurrentImageFT, CorrelatedImageFT, CorrelatedImage, fpsCamera,
            binnedImage, ACQMODE, &rotatingCounter, &sumX, &sumY, AUTOGUIDERMODE, &sumVoltageX,
            &sumVoltageY, &imageSaveCounter, &integralVoltX, &integralVoltY, &updateReference,
            &offloadShiftsToAutoguider, &autoGuiderCounter, &meanVoltX, &meanVoltY,
            &previousErrorX, &previousErrorY
        );

    }

    virtual uint16_t fpsTrigger() override
    {
        return 0;
    }

    uint64_t getNbImagesReceived()
    {
        return this->_nbImagesReceived;
    };

    uint16_t* getNextImage() {
        return this->imagePointer;
    };

    double* getCurrentImage() {
        return this->CurrentImage;
    };

    uint64_t* getCurrCount() {
        return &this->curr_count;
    };

    void setACQMODE(int ACQMODE) {
        this->ACQMODE = ACQMODE;
    };

    void setAUTOGUIDERMODE(int AUTOGUIDERMODE) {
        this->AUTOGUIDERMODE = AUTOGUIDERMODE;
    };

private:
    uint64_t _nbImagesReceived, iter;
    uint16_t* imagePointer=NULL;
    int ACQMODE, AUTOGUIDERMODE;
    void (*workFunction)(uint16_t* Image, uint64_t nbImagesReceived, double* binImgTime, double* imgWriteTime,
        double* imgShiftTime, double* calcCorrectionTime, double* applyCorrectionTime,
        uint64_t* curr_count, uint64_t* ignore_count, uint64_t NREFRESH, uint64_t* NumRefImage,
        uint64_t* integralCounter, uint64_t* derivativeCounter, bool* breakLoop, uint64_t* counter,
        uint64_t* status_count, double* XShift, double* YShift, uint64_t* error_no,
        deque<double> &integralErrorX, deque<double> &integralErrorY, deque<double> &derivativeErrorX,
        deque<double> &derivativeErrorY, double *CurrentImage, fftw_complex *CurrentImageFT,
        fftw_complex *CorrelatedImageFT, double *CorrelatedImage, int fpsCamera,
        uint16_t* binnedImage, int ACQMODE, uint64_t* rotatingCounter, double *sumX, double *sumY,
        int AUTOGUIDERMODE, double *sumVoltageX, double *sumVoltageY,
        uint64_t *imageSaveCounter, double *integralVoltX, double * integralVoltY,
        bool *updateReference, bool *offloadShiftsToAutoguider, uint32_t* autoGuiderCounter,
        double* meanVoltX, double* meanVoltY, double* previousErrorX, double* previousErrorY
    ) = NULL;
    chrono::high_resolution_clock::time_point t0, t1;
    chrono::duration<double> dt;
    double fps;
    double binImgTime, imgWriteTime, imgShiftTime, calcCorrectionTime, applyCorrectionTime, sumX, sumY, sumVoltageX, sumVoltageY;
    uint64_t integralCounter, derivativeCounter, NREFRESH, curr_count, ignore_count, NumRefImage, counter, status_count, error_no;
    double XShift, YShift, integralVoltX, integralVoltY;
    bool breakLoop, printStats, updateReference, offloadShiftsToAutoguider;
    deque<double> integralErrorX;
    deque<double> integralErrorY;
    deque<double> derivativeErrorX;
    deque<double> derivativeErrorY;
    double *CurrentImage;
    fftw_complex *CurrentImageFT;
    fftw_complex *CorrelatedImageFT;
    double *CorrelatedImage;
    uint16_t *binnedImage;
    uint64_t rotatingCounter;
    uint64_t imageSaveCounter;
    uint32_t autoGuiderCounter;
    double meanVoltX, meanVoltY;
    double previousErrorX, previousErrorY;
};

int initDev(double askfps=0, double asktemp=0, string NucMode="BiasFlat") {

    std::string cameraName;
    fli = new FliSdk();
    vector<string> listOfGrabbers = fli->detectGrabbers();

    if(listOfGrabbers.size() == 0)
    {
        cout << "No grabber detected, exit." << endl;
        return -1;
    }

    vector<string> listOfCameras = fli->detectCameras();

    if(listOfCameras.size() == 0)
    {
        cout << "No camera detected, exit." << endl;
        return -1;
    }

    int selectedCamera = 0;

    cameraName = listOfCameras[selectedCamera];
    cout << "Setting camera " << cameraName << endl;

    fli->setCamera(cameraName);

    cout << "Setting mode Full." << endl;
    //set full mode
    fli->setMode(FliSdk::Mode::GrabOnly);
    cout << "Done Set mode Full." << endl;
    fli->update();
    fli->setImageDimension(WIDTH, HEIGHT);
    fli->update();
    cout << "Done update." << endl;

    if(!fli->serialCamera() && !fli->cblueOne())
    {
        cout << "Fatal error." << endl;
        return -1;
    }



    double temp = fli->cblueOne()->DeviceTemperature->getValue();
    cout << "Temp read: " << temp << endl;

    fli->imageProcessing()->enableAutoExposure(false);
    fli->update();
    fli->cblueOne()->ExposureMode->setValue(FliSfncCameraEnum::ExposureModeEnum::Timed);
//    fli->cblueOne()->E



    uint16_t offsetX=250, offsetY=0;

    offsetX = fli->cblueOne()->OffsetX->getValue();
    offsetY = fli->cblueOne()->OffsetY->getValue();

    cout << "OffsetX: "<< offsetX <<endl;
    cout << "OffsetY: "<< offsetY <<endl;

//    cout <<"Enter offseX: "<<endl;
//    cin>>offsetX;
//    cout <<"Enter offseY: "<<endl;
//    cin>>offsetY;
    fli->cblueOne()->Width->setValue(WIDTH);
    fli->cblueOne()->Height->setValue(HEIGHT);
    fli->cblueOne()->OffsetX->setValue(offsetX);
    fli->cblueOne()->OffsetY->setValue(offsetY);

    cout << "Width: "<< fli->cblueOne()->Width->getValue()<<endl;
    cout << "Height: "<< fli->cblueOne()->Height->getValue()<<endl;
    cout << "OffsetX: "<< fli->cblueOne()->OffsetX->getValue()<<endl;
    cout << "OffsetY: "<< fli->cblueOne()->OffsetY->getValue()<<endl;

    fli->cblueOne()->setStringFeature("UserSetSelector", "UserSet0");

    double fps=0;
    if (askfps > 0) {
        fps = fli->cblueOne()->AcquisitionFrameRate->getValue();
        double input = -1;
        cout << "Fps read: " << fps << endl;
        while(input < 0) {
            cout << "Enter fps (<="<< fps <<"): " << endl;
            cin>>input;
        }
        fli->cblueOne()->AcquisitionFrameRate->setValue(input);
        fps = fli->cblueOne()->AcquisitionFrameRate->getValue();
        cout << "Fps read: " << fps << endl;
        input = -1;
    }
    else {
        fps = fli->cblueOne()->AcquisitionFrameRate->getMax();
        fli->cblueOne()->AcquisitionFrameRate->setValue(fps);
        fps = fli->cblueOne()->AcquisitionFrameRate->getValue();
        cout << "Fps read: " << fps << endl;
    }

    double exp = fli->cblueOne()->ExposureTime->getValue();
    cout << "Exposure Time read: " << exp << endl;
    cout << "Enter Exposure Time: ";
    cin >>exp;
    fli->cblueOne()->ExposureTime->setValue(exp);
    fli->imageProcessing()->updateAutoExposureParam();
    exp = fli->cblueOne()->ExposureTime->getValue();
    cout << "Exposure Time read: " << exp << endl;

    fli->cblueOne()->setStringFeature("NucMode", NucMode);

    string nucmode="something";

    cout << "Width: "<< fli->cblueOne()->Width->getValue()<<endl;
    cout << "Height: "<< fli->cblueOne()->Height->getValue()<<endl;
    cout << "OffsetX: "<< fli->cblueOne()->OffsetX->getValue()<<endl;
    cout << "OffsetY: "<< fli->cblueOne()->OffsetY->getValue()<<endl;
    exp = fli->cblueOne()->ExposureTime->getValue();
    cout << "Exposure Time read: " << exp << endl;
    fps = fli->cblueOne()->AcquisitionFrameRate->getValue();
    cout << "Fps read: " << fps << endl;
    fli->cblueOne()->getStringFeature("NucMode", nucmode);
    cout << "NucMode : "<< nucmode <<endl;
    fli->cblueOne()->getStringFeature("UserSetSelector", nucmode);
    cout << "UserSetSelector : "<< nucmode <<endl;
    fpsCamera = int(fps) + 1;
    return 0;
}

int stopDev() {
    delete fli;
    return 0;
}

int startCamera() {
    fli->start();
    return 0;
}

int stopCamera() {
    fli->stop();
    return 0;
}