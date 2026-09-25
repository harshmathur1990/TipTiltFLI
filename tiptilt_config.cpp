#include "tiptilt_config.h"

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "controls.h"

namespace {

std::vector<double> parseLine(const std::string& line) {
    std::vector<double> values;
    std::stringstream row(line);
    std::string token;
    while (std::getline(row, token, ',')) {
        if (!token.empty()) {
            values.push_back(std::stod(token));
        }
    }
    return values;
}

bool asBool(double value) {
    return std::fabs(value) > 0.5;
}

void assignIfPresent(const std::vector<double>& values, int index, double& target) {
    if (index < static_cast<int>(values.size())) {
        target = values[index];
    }
}

void assignIfPresent(const std::vector<double>& values, int index, int& target) {
    if (index < static_cast<int>(values.size())) {
        target = static_cast<int>(std::lround(values[index]));
    }
}

} // namespace

std::string GetDefaultTipTiltConfigPath() {
    return "CalibrationMatrix.csv";
}

TipTiltConfig LoadTipTiltConfig(const std::string& path) {
    TipTiltConfig config;
    std::ifstream input(path);
    if (!input.is_open()) {
        throw std::runtime_error("Unable to open config file: " + path);
    }

    std::vector<std::vector<double>> rows;
    std::unordered_map<std::string, std::string> properties;
    std::string line;
    while (std::getline(input, line)) {
        if (!line.empty()) {
            if (line.find('=') != std::string::npos) {
                const std::size_t separator = line.find('=');
                properties[line.substr(0, separator)] = line.substr(separator + 1);
            } else {
                rows.push_back(parseLine(line));
            }
        }
    }

    if (const auto it = properties.find("tiptilt_x_com_port"); it != properties.end()) {
        config.tipTiltXComPort = std::stoi(it->second);
    }
    if (const auto it = properties.find("tiptilt_y_com_port"); it != properties.end()) {
        config.tipTiltYComPort = std::stoi(it->second);
    }
    if (const auto it = properties.find("autoguider_com_port"); it != properties.end()) {
        config.autoguiderComPort = std::stoi(it->second);
    }
    if (const auto it = properties.find("autoguider_motor_frequency"); it != properties.end()) {
        config.autoguiderMotorFrequency = std::stoi(it->second);
    }
    if (const auto it = properties.find("camera_target_frame_rate"); it != properties.end()) {
        config.cameraTargetFrameRate = std::stod(it->second);
    }
    if (const auto it = properties.find("camera_exposure_time"); it != properties.end()) {
        config.cameraExposureTime = std::stod(it->second);
    }
    if (const auto it = properties.find("camera_nuc_mode"); it != properties.end()) {
        config.cameraNucMode = it->second;
    }

    if (rows.size() > 0) {
        assignIfPresent(rows[0], 0, config.correctionA00);
        assignIfPresent(rows[0], 1, config.correctionA10);
        assignIfPresent(rows[0], 2, config.correctionA01);
        assignIfPresent(rows[0], 3, config.correctionA11);
    }
    if (rows.size() > 1) {
        assignIfPresent(rows[1], 0, config.calibrationClM00);
        assignIfPresent(rows[1], 1, config.calibrationClM10);
        assignIfPresent(rows[1], 2, config.calibrationClM01);
        assignIfPresent(rows[1], 3, config.calibrationClM11);
    }
    if (rows.size() > 2) {
        assignIfPresent(rows[2], 0, config.vxOffset);
        assignIfPresent(rows[2], 1, config.vyOffset);
        assignIfPresent(rows[2], 2, config.slewRate);
    }
    if (rows.size() > 3) {
        assignIfPresent(rows[3], 0, config.tipTiltKp);
        assignIfPresent(rows[3], 1, config.tipTiltKd);
        assignIfPresent(rows[3], 2, config.tipTiltKi);
    }
    if (rows.size() <= 6) {
        if (rows.size() > 4) {
            assignIfPresent(rows[4], 0, config.autoguiderCorrectionTime);
            assignIfPresent(rows[4], 1, config.imageSaveAfterSeconds);
        }
        if (rows.size() > 5) {
            assignIfPresent(rows[5], 0, config.autoguiderOffloadLimitX);
            assignIfPresent(rows[5], 1, config.autoguiderOffloadLimitY);
        }
        return config;
    }

    if (rows.size() > 4) {
        assignIfPresent(rows[4], 0, config.integralWindow);
        assignIfPresent(rows[4], 1, config.derivativeWindow);
    }
    if (rows.size() > 5) {
        assignIfPresent(rows[5], 0, config.autoguiderA00);
        assignIfPresent(rows[5], 1, config.autoguiderA10);
        assignIfPresent(rows[5], 2, config.autoguiderA01);
        assignIfPresent(rows[5], 3, config.autoguiderA11);
    }
    if (rows.size() > 6) {
        assignIfPresent(rows[6], 0, config.sampleTime);
        assignIfPresent(rows[6], 1, config.outputMin);
        assignIfPresent(rows[6], 2, config.outputMax);
        assignIfPresent(rows[6], 3, config.integratorMin);
        assignIfPresent(rows[6], 4, config.integratorMax);
        assignIfPresent(rows[6], 5, config.derivativeTau);
    }
    if (rows.size() > 7) {
        assignIfPresent(rows[7], 0, config.autoguiderKp);
        assignIfPresent(rows[7], 1, config.autoguiderKd);
        assignIfPresent(rows[7], 2, config.autoguiderKi);
    }
    if (rows.size() > 8) {
        assignIfPresent(rows[8], 0, config.autoguiderCorrectionTime);
    }
    if (rows.size() > 9) {
        assignIfPresent(rows[9], 0, config.imageSaveAfterSeconds);
    }
    if (rows.size() > 10) {
        assignIfPresent(rows[10], 0, config.autoguiderOffloadLimitX);
        assignIfPresent(rows[10], 1, config.autoguiderOffloadLimitY);
    }
    if (rows.size() > 11) {
        config.showLiveView = asBool(rows[11].empty() ? 0.0 : rows[11][0]);
    }
    if (rows.size() > 12) {
        config.useCameraFlat = asBool(rows[12].empty() ? 0.0 : rows[12][0]);
    }
    if (rows.size() > 13) {
        config.autoPMode = asBool(rows[13].empty() ? 0.0 : rows[13][0]);
    }
    if (rows.size() > 14) {
        assignIfPresent(rows[14], 0, config.minAutoKp);
        assignIfPresent(rows[14], 1, config.maxAutoKp);
        assignIfPresent(rows[14], 2, config.minAutoKd);
        assignIfPresent(rows[14], 3, config.maxAutoKd);
    }
    if (rows.size() > 15) {
        assignIfPresent(rows[15], 0, config.derivativeCutoffFrequency);
    }

    return config;
}

bool SaveTipTiltConfig(const TipTiltConfig& config, const std::string& path, std::string* errorMessage) {
    std::ofstream output(path, std::ios::trunc);
    if (!output.is_open()) {
        if (errorMessage != nullptr) {
            *errorMessage = "Unable to open config file for writing: " + path;
        }
        return false;
    }

    output << config.correctionA00 << ", " << config.correctionA10 << ", " << config.correctionA01 << ", " << config.correctionA11 << '\n';
    output << config.calibrationClM00 << ", " << config.calibrationClM10 << ", " << config.calibrationClM01 << ", " << config.calibrationClM11 << '\n';
    output << config.vxOffset << ", " << config.vyOffset << ", " << config.slewRate << '\n';
    output << config.tipTiltKp << ", " << config.tipTiltKd << ", " << config.tipTiltKi << '\n';
    output << config.integralWindow << ", " << config.derivativeWindow << '\n';
    output << config.autoguiderA00 << ", " << config.autoguiderA10 << ", " << config.autoguiderA01 << ", " << config.autoguiderA11 << '\n';
    output << config.sampleTime << ", " << config.outputMin << ", " << config.outputMax << ", " << config.integratorMin << ", " << config.integratorMax << ", " << config.derivativeTau << '\n';
    output << config.autoguiderKp << ", " << config.autoguiderKd << ", " << config.autoguiderKi << '\n';
    output << config.autoguiderCorrectionTime << '\n';
    output << config.imageSaveAfterSeconds << '\n';
    output << config.autoguiderOffloadLimitX << ", " << config.autoguiderOffloadLimitY << '\n';
    output << (config.showLiveView ? 1 : 0) << '\n';
    output << (config.useCameraFlat ? 1 : 0) << '\n';
    output << (config.autoPMode ? 1 : 0) << '\n';
    output << config.minAutoKp << ", " << config.maxAutoKp << ", " << config.minAutoKd << ", " << config.maxAutoKd << '\n';
    output << config.derivativeCutoffFrequency << '\n';
    output << "tiptilt_x_com_port=" << config.tipTiltXComPort << '\n';
    output << "tiptilt_y_com_port=" << config.tipTiltYComPort << '\n';
    output << "autoguider_com_port=" << config.autoguiderComPort << '\n';
    output << "autoguider_motor_frequency=" << config.autoguiderMotorFrequency << '\n';
    output << "camera_target_frame_rate=" << config.cameraTargetFrameRate << '\n';
    output << "camera_exposure_time=" << config.cameraExposureTime << '\n';
    output << "camera_nuc_mode=" << config.cameraNucMode << '\n';

    if (!output.good()) {
        if (errorMessage != nullptr) {
            *errorMessage = "Failed while writing config file: " + path;
        }
        return false;
    }

    if (errorMessage != nullptr) {
        errorMessage->clear();
    }
    return true;
}

void ApplyTipTiltConfigToGlobals(const TipTiltConfig& config) {
    A00 = config.correctionA00;
    A10 = config.correctionA10;
    A01 = config.correctionA01;
    A11 = config.correctionA11;

    ClM00 = config.calibrationClM00;
    ClM10 = config.calibrationClM10;
    ClM01 = config.calibrationClM01;
    ClM11 = config.calibrationClM11;

    Vxoff = config.vxOffset;
    Vyoff = config.vyOffset;
    SlewRate = config.slewRate;

    Kp = config.tipTiltKp;
    Kd = config.tipTiltKd;
    Ki = config.tipTiltKi;

    Ni = config.integralWindow;
    Nd = config.derivativeWindow;

    AA00 = config.autoguiderA00;
    AA10 = config.autoguiderA10;
    AA01 = config.autoguiderA01;
    AA11 = config.autoguiderA11;

    sampleTime = config.sampleTime;
    limMin = config.outputMin;
    limMax = config.outputMax;
    limMinInt = config.integratorMin;
    limMaxInt = config.integratorMax;
    tau = config.derivativeTau;

    Akp = config.autoguiderKp;
    Akd = config.autoguiderKd;
    Aki = config.autoguiderKi;

    autoGuiderCorrectionTime = config.autoguiderCorrectionTime;
    imageSaveAfterSecond = config.imageSaveAfterSeconds;
    autoGuiderOffloadLimitX = config.autoguiderOffloadLimitX;
    autoGuiderOffloadLimitY = config.autoguiderOffloadLimitY;

    liveView = config.showLiveView ? 1 : 0;
    useCameraFlat = config.useCameraFlat;
    autoPMode = config.autoPMode;
    minKp = config.minAutoKp;
    maxKp = config.maxAutoKp;
    minKd = config.minAutoKd;
    maxKd = config.maxAutoKd;
    cutOffFrequencyOfDerivativeError = config.derivativeCutoffFrequency;

    mp = 0.0;
    cp = minKp;
    md = 0.0;
    cd = minKd;
}
