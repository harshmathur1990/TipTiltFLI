#ifndef TIPTILT_CONFIG_H
#define TIPTILT_CONFIG_H

#include <string>

struct TipTiltConfig {
    int tipTiltXComPort = 0;
    int tipTiltYComPort = 0;
    int autoguiderComPort = 1;
    int autoguiderMotorFrequency = 350;

    double cameraTargetFrameRate = 40.0;
    double cameraExposureTime = 0.0;
    std::string cameraNucMode = "BiasFlat";

    double correctionA00 = 0.0;
    double correctionA10 = 0.0;
    double correctionA01 = 0.0;
    double correctionA11 = 0.0;

    double calibrationClM00 = 0.0;
    double calibrationClM10 = 0.0;
    double calibrationClM01 = 0.0;
    double calibrationClM11 = 0.0;

    double vxOffset = 60.0;
    double vyOffset = 60.0;
    double slewRate = 200.0;

    double tipTiltKp = 0.2;
    double tipTiltKd = 0.0;
    double tipTiltKi = 0.0;

    int integralWindow = -1;
    int derivativeWindow = -1;

    double autoguiderA00 = 0.0;
    double autoguiderA10 = 0.0;
    double autoguiderA01 = 0.0;
    double autoguiderA11 = 0.0;

    double sampleTime = 0.0015;
    double outputMin = -10.0;
    double outputMax = 10.0;
    double integratorMin = -10.0;
    double integratorMax = 10.0;
    double derivativeTau = 0.00152;

    double autoguiderKp = 0.5;
    double autoguiderKd = 0.0;
    double autoguiderKi = 0.0;

    int autoguiderCorrectionTime = 15;
    int imageSaveAfterSeconds = 60;

    double autoguiderOffloadLimitX = 5.0;
    double autoguiderOffloadLimitY = 10.0;

    bool showLiveView = true;
    bool useCameraFlat = true;
    bool autoPMode = false;

    double minAutoKp = 0.2;
    double maxAutoKp = 0.4;
    double minAutoKd = 0.001;
    double maxAutoKd = 0.01;

    double derivativeCutoffFrequency = 110.0;
};

TipTiltConfig LoadTipTiltConfig(const std::string& path);
bool SaveTipTiltConfig(const TipTiltConfig& config, const std::string& path, std::string* errorMessage = nullptr);
void ApplyTipTiltConfigToGlobals(const TipTiltConfig& config);
std::string GetDefaultTipTiltConfigPath();

#endif
