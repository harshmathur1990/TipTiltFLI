#include "FliSdk.h"
#define LONG_DELAY 2000
#include <vector>
#include <iostream>

using namespace std;

FliSdk* fli;
uint16_t width=512, height=512;
int nbImages = 0;

int initDev() {

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
    cout << "Fps read: " << fps << endl;
    double temp = fli->cblueSfnc()->DeviceTemperature->getValue();
    cout << "Temp read: " << temp << endl;
    cout << "Setting Temperature 0"<<endl;
    fli->cblueSfnc()->DeviceTemperature->setValue(0);
    temp = fli->cblueSfnc()->DeviceTemperature->getValue();
    cout << "Temp read: " << temp << endl;

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