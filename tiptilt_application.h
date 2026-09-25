#ifndef TIPTILT_APPLICATION_H
#define TIPTILT_APPLICATION_H

#include <cstdint>
#include <vector>

#include "firstlight.h"

struct TipTiltUiSettings {
    CameraConfiguration camera;
    int xComPort = 0;
    int yComPort = 0;
    int autoguiderComPort = 1;
    int autoguiderMotorFrequency = 350;
    int acquisitionMode = 1;
    int referenceRefreshFrames = 100;
    int autoguiderMode = 0;
};

struct DarkFlatUiSettings {
    int framesPerPosition = 100;
    int xStart = -20;
    int xEnd = 130;
    int yStart = -20;
    int yEnd = 130;
    int step = 5;
    int settleTimeMs = 50;
    std::string outputDirectory = "Flats";
    std::string filePrefix = "MeanFlat";
};

struct CalibrationUiSettings {
    int frames = 100;
    int framesPerPosition = 1;
    int mode = 0;
    int axis = -1;
    double voltageStart = -10.0;
    double voltageRange = 20.0;
    double stationaryVoltage = 0.0;
    int motorControllerComPort = 1;
    int motorFrequency = 350;
};

int InitializeCameraSessionForUi(const TipTiltUiSettings& settings);
int InitializeTipTiltHardwareForUi(const TipTiltUiSettings& settings);
int InitializeAutoguiderHardwareForUi(const TipTiltUiSettings& settings);
int StartClosedLoopForUi(const TipTiltUiSettings& settings);
int ApplyManualTipTiltVoltagesForUi(double xVoltage, double yVoltage);
int RunDarkFlatCaptureForUi(const DarkFlatUiSettings& settings);
int RunCalibrationForUi(const CalibrationUiSettings& settings);
void ShutdownTipTiltRuntimeForUi();
bool CopyLatestPreviewFrameForUi(std::vector<uint16_t>& frame);
bool IsClosedLoopRunningForUi();
bool IsAutoguiderInitializedForUi();

#endif
