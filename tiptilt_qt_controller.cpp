#include "tiptilt_qt_controller.h"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <vector>

#include "imageheaders.h"
#include "tiptilt_config.h"
#include "tiptilt_tools.h"

TipTiltQtController::TipTiltQtController() {
    settings_.camera.promptForFrameRate = false;
    settings_.camera.promptForExposure = false;

    try {
        const TipTiltConfig config = LoadTipTiltConfig(GetDefaultTipTiltConfigPath());
        settings_.xComPort = config.tipTiltXComPort;
        settings_.yComPort = config.tipTiltYComPort;
        settings_.autoguiderComPort = config.autoguiderComPort;
        settings_.autoguiderMotorFrequency = config.autoguiderMotorFrequency;
        settings_.camera.targetFrameRate = config.cameraTargetFrameRate;
        settings_.camera.exposureTime = config.cameraExposureTime;
        settings_.camera.nucMode = config.cameraNucMode;
    } catch (...) {
        settings_.camera.targetFrameRate = 40.0;
        settings_.camera.nucMode = "BiasFlat";
    }
}

TipTiltQtController::~TipTiltQtController() {
    shutdown();
}

TipTiltUiSettings& TipTiltQtController::settings() {
    return settings_;
}

const TipTiltUiSettings& TipTiltQtController::settings() const {
    return settings_;
}

bool TipTiltQtController::startCameraSession(std::string* errorMessage) {
    return checkStatus(InitializeCameraSessionForUi(settings_), "starting camera session", errorMessage);
}

bool TipTiltQtController::initializeTipTilt(std::string* errorMessage) {
    return checkStatus(InitializeTipTiltHardwareForUi(settings_), "initializing tip/tilt hardware", errorMessage);
}

bool TipTiltQtController::initializeAutoguider(std::string* errorMessage) {
    return checkStatus(InitializeAutoguiderHardwareForUi(settings_), "initializing autoguider hardware", errorMessage);
}

bool TipTiltQtController::startClosedLoop(std::string* errorMessage) {
    return checkStatus(StartClosedLoopForUi(settings_), "starting the closed loop", errorMessage);
}

bool TipTiltQtController::applyManualTipTiltVoltages(double xVoltage, double yVoltage, std::string* errorMessage) {
    return checkStatus(
        ApplyManualTipTiltVoltagesForUi(xVoltage, yVoltage),
        "applying manual tip/tilt voltages",
        errorMessage
    );
}

bool TipTiltQtController::runDarkFlatCapture(const DarkFlatUiSettings& settings, std::string* errorMessage) {
    return checkStatus(
        RunDarkFlatCaptureForUi(settings),
        "running dark/flat capture",
        errorMessage
    );
}

bool TipTiltQtController::runCalibration(const CalibrationUiSettings& settings, std::string* errorMessage) {
    return checkStatus(
        RunCalibrationForUi(settings),
        "running calibration capture",
        errorMessage
    );
}

void TipTiltQtController::shutdown() {
    ShutdownTipTiltRuntimeForUi();
}

QImage TipTiltQtController::capturePreviewImage() const {
    std::vector<uint16_t> rawFrame;
    if (!CopyLatestPreviewFrameForUi(rawFrame)) {
        return {};
    }
    return makePreviewImage(rawFrame.data());
}

std::string TipTiltQtController::runImageBinningBenchmark() const {
    return RunImageBinningBenchmarkTool().message;
}

std::string TipTiltQtController::runMklBenchmark(int iterations) const {
    return RunMklBenchmarkTool(iterations).message;
}

std::string TipTiltQtController::runVoltageShiftMatrixFit(int skipValues, int sampleCount) const {
    return RunVoltageShiftMatrixTool(skipValues, sampleCount).message;
}

std::string TipTiltQtController::runSerialDebug(int comPort, int baudRate, const std::string& payload, int bytesToRead) const {
    return RunSerialDebugTool(comPort, baudRate, payload, bytesToRead).message;
}

std::string TipTiltQtController::connectMotorController(int comPort) const {
    return ConnectMotorControllerTool(comPort).message;
}

std::string TipTiltQtController::disconnectMotorController() const {
    return DisconnectMotorControllerTool().message;
}

std::string TipTiltQtController::setMotorFrequency(int motorNum, int frequency) const {
    return SetMotorFrequencyTool(motorNum, frequency).message;
}

std::string TipTiltQtController::moveMotor(int motorNum, int direction, int counts) const {
    return MoveMotorTool(motorNum, direction, counts).message;
}

std::string TipTiltQtController::setMotorEnabled(int motorNum, bool enabled) const {
    return SetMotorEnabledTool(motorNum, enabled).message;
}

std::string TipTiltQtController::exitMotor(int motorNum) const {
    return ExitMotorTool(motorNum).message;
}

bool TipTiltQtController::isAutoguiderInitialized() const {
    return IsAutoguiderInitializedForUi();
}

bool TipTiltQtController::isClosedLoopRunning() const {
    return IsClosedLoopRunningForUi();
}

bool TipTiltQtController::checkStatus(int status, const std::string& step, std::string* errorMessage) {
    if (status == 0) {
        if (errorMessage != nullptr) {
            errorMessage->clear();
        }
        return true;
    }

    if (errorMessage != nullptr) {
        *errorMessage = step + " failed with status " + std::to_string(status);
    }
    return false;
}

QImage TipTiltQtController::makePreviewImage(const uint16_t* rawFrame) {
    if (rawFrame == nullptr) {
        return {};
    }

    const auto [minIt, maxIt] = std::minmax_element(rawFrame, rawFrame + (WIDTH * HEIGHT));
    const uint16_t minValue = *minIt;
    const uint16_t maxValue = *maxIt;
    const uint16_t span = maxValue > minValue ? static_cast<uint16_t>(maxValue - minValue) : 1;

    std::vector<std::uint8_t> pixels(WIDTH * HEIGHT);
    for (unsigned index = 0; index < WIDTH * HEIGHT; ++index) {
        const uint16_t value = rawFrame[index];
        pixels[index] = static_cast<std::uint8_t>(
            (255u * static_cast<unsigned>(value - minValue)) / span
        );
    }

    QImage image(pixels.data(), static_cast<int>(WIDTH), static_cast<int>(HEIGHT), static_cast<int>(WIDTH), QImage::Format_Grayscale8);
    return image.copy();
}
