#ifndef TIPTILT_QT_CONTROLLER_H
#define TIPTILT_QT_CONTROLLER_H

#include <string>

#include <QImage>

#include "tiptilt_application.h"

class TipTiltQtController {
public:
    TipTiltQtController();
    ~TipTiltQtController();

    TipTiltUiSettings& settings();
    const TipTiltUiSettings& settings() const;

    bool startCameraSession(std::string* errorMessage = nullptr);
    bool initializeTipTilt(std::string* errorMessage = nullptr);
    bool initializeAutoguider(std::string* errorMessage = nullptr);
    bool startClosedLoop(std::string* errorMessage = nullptr);
    bool applyManualTipTiltVoltages(double xVoltage, double yVoltage, std::string* errorMessage = nullptr);
    bool runDarkFlatCapture(const DarkFlatUiSettings& settings, std::string* errorMessage = nullptr);
    bool runCalibration(const CalibrationUiSettings& settings, std::string* errorMessage = nullptr);
    void shutdown();

    QImage capturePreviewImage() const;
    std::string runImageBinningBenchmark() const;
    std::string runMklBenchmark(int iterations) const;
    std::string runVoltageShiftMatrixFit(int skipValues, int sampleCount) const;
    std::string runSerialDebug(int comPort, int baudRate, const std::string& payload, int bytesToRead) const;
    std::string connectMotorController(int comPort) const;
    std::string disconnectMotorController() const;
    std::string setMotorFrequency(int motorNum, int frequency) const;
    std::string moveMotor(int motorNum, int direction, int counts) const;
    std::string setMotorEnabled(int motorNum, bool enabled) const;
    std::string exitMotor(int motorNum) const;
    bool isAutoguiderInitialized() const;
    bool isClosedLoopRunning() const;

private:
    static bool checkStatus(int status, const std::string& step, std::string* errorMessage);
    static QImage makePreviewImage(const uint16_t* rawFrame);

    TipTiltUiSettings settings_;
};

#endif
