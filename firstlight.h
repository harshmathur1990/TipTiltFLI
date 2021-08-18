#include "FliSdk.h"
#define LONG_DELAY 2000
#include <vector>
#include <iostream>

using namespace std;

FliSdk* fli;
uint16_t width=512, height=512;
int nbImages = 0;

class RawImageReceivedObserver : public IRawImageReceivedObserver
{
public:
    RawImageReceivedObserver(uint16_t width, uint16_t height) :
            _imgSize(width* height),
            _nbImagesReceived(0)
    {
        fli->addRawImageReceivedObserver(this);
    };

    virtual void imageReceived(const uint8_t* image) override
    {
        *imagePointer = (uint16_t *) image;
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

    uint16_t* getNextImage() {
        if (bufferStatus == 0) {
            imagePointer = &imageBuffer2;
            bufferStatus = 1;
            return imageBuffer1;
        }
        else {
            imagePointer = &imageBuffer1;
            bufferStatus = 0;
            return imageBuffer2;
        }
    };

private:
    uint32_t _imgSize;
    uint32_t _nbImagesReceived;
    uint16_t *imageBuffer1=NULL, *imageBuffer2=NULL;
    uint16_t **imagePointer=NULL;
    int bufferStatus=0;
};

int initDev(double askfps=0, double asktemp=0) {

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
    fli->setMode(FliSdk::Mode::Full);

    fli->update();

    if(!fli->serialCamera() && !fli->cblueSfnc())
    {
        cout << "Fatal error." << endl;
        return -1;
    }

    double fps = fli->cblueSfnc()->AcquisitionFrameRate->getValue();
    double input=-1;
    cout << "Fps read: " << fps << endl;
    if (askfps > 0) {
        while(input < 0 || input > fps) {
            cout << "Enter fps (<="<< fps <<"): " << endl;
            cin>>input;
        }
        fli->cblueSfnc()->AcquisitionFrameRate->setValue(input);
        fps = fli->cblueSfnc()->AcquisitionFrameRate->getValue();
        cout << "Fps read: " << fps << endl;
        input = -1;
    }

    double temp = fli->cblueSfnc()->DeviceTemperature->getValue();
    cout << "Temp read: " << temp << endl;
    if (asktemp > 0) {
        while(input < 0 || input > 30) {
            cout << "Enter temp (<="<< 30 <<"): " << endl;
            cin>>input;
        }
        fli->cblueSfnc()->DeviceTemperature->setValue(input);
        temp = fli->cblueSfnc()->DeviceTemperature->getValue();
        cout << "Temp read: " << temp << endl;
    }
    else {
        cout << "Setting Temperature 0"<<endl;
        fli->cblueSfnc()->DeviceTemperature->setValue(0);
        temp = fli->cblueSfnc()->DeviceTemperature->getValue();
        cout << "Temp read: " << temp << endl;
    }


    fli->cblueSfnc()->ExposureMode->setValue(FliCblueSfncEnum::ExposureModeEnum::Timed);

    double exp = fli->cblueSfnc()->ExposureTime->getValue();
    cout << "Exposure Time read: " << exp << endl;
    cout << "Enter Exposure Time: ";
    cin >>exp;
    fli->cblueSfnc()->ExposureTime->setValue(exp);
    exp = fli->cblueSfnc()->ExposureTime->getValue();
    cout << "Exposure Time read: " << exp << endl;

    fli->cblueSfnc()->Width->setValue(width);
    fli->cblueSfnc()->Height->setValue(height);

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