#include "tiptilt_tools.h"

#include <chrono>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <mkl.h>

#include "imageheaders.h"
#include "imageprocess.h"
#include "motorcontrols.h"
#include "serialconnection.h"
#include "tiptilt_config.h"

namespace {

std::vector<double> readSignalColumn(const std::string& path, int skipValues, int sampleCount, int valueIndex) {
    std::ifstream input(path);
    if (!input.is_open()) {
        throw std::runtime_error("Unable to open " + path);
    }

    std::vector<double> values;
    std::string line;
    while (skipValues-- > 0 && std::getline(input, line)) {
    }

    while (sampleCount-- > 0 && std::getline(input, line)) {
        std::stringstream row(line);
        std::string token;
        std::vector<double> parsed;
        while (std::getline(row, token, ',')) {
            if (!token.empty()) {
                parsed.push_back(std::stod(token));
            }
        }
        if (valueIndex < static_cast<int>(parsed.size())) {
            values.push_back(parsed[valueIndex]);
        }
    }

    return values;
}

double computeSlope(const std::vector<double>& voltage, const std::vector<double>& shift) {
    const int n = static_cast<int>(std::min(voltage.size(), shift.size()));
    if (n < 2) {
        throw std::runtime_error("Not enough samples to compute slope.");
    }

    double sumVoltage = 0.0;
    double sumVoltageSquared = 0.0;
    double sumShift = 0.0;
    double sumShiftVoltage = 0.0;
    for (int index = 0; index < n; ++index) {
        sumVoltage += voltage[index];
        sumVoltageSquared += voltage[index] * voltage[index];
        sumShift += shift[index];
        sumShiftVoltage += shift[index] * voltage[index];
    }

    const double denominator = sumVoltage * sumVoltage - n * sumVoltageSquared;
    if (std::fabs(denominator) < std::numeric_limits<double>::epsilon()) {
        throw std::runtime_error("Degenerate samples; slope denominator is zero.");
    }
    return (sumShift * sumVoltage - n * sumShiftVoltage) / denominator;
}

ToolRunResult makeError(const std::string& message) {
    ToolRunResult result;
    result.success = false;
    result.message = message;
    return result;
}

} // namespace

ToolRunResult RunImageBinningBenchmarkTool() {
    ToolRunResult result;
    std::vector<uint16_t> image(WIDTH * HEIGHT, 4095);
    std::vector<uint16_t> binnedImage(NX * NY, 0);

    const auto start = std::chrono::high_resolution_clock::now();
    bin_separately(image.data(), binnedImage.data());
    const auto end = std::chrono::high_resolution_clock::now();
    const double seconds = std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count();

    std::ofstream output("binnedFlat.dat", std::ios::binary | std::ios::trunc);
    output.write(reinterpret_cast<const char*>(binnedImage.data()), static_cast<std::streamsize>(binnedImage.size() * sizeof(uint16_t)));

    std::ostringstream message;
    message << "Image binning benchmark finished in " << seconds << " s and wrote binnedFlat.dat";
    result.success = true;
    result.message = message.str();
    return result;
}

ToolRunResult RunMklBenchmarkTool(int iterations) {
    if (iterations <= 0) {
        return makeError("Iteration count must be greater than zero.");
    }

    constexpr MKL_LONG kSide = 256;
    constexpr MKL_LONG kComplexWidth = (kSide / 2) + 1;
    constexpr MKL_LONG kFrequencySize = kSide * kComplexWidth;

    DFTI_DESCRIPTOR_HANDLE descriptor = nullptr;
    MKL_LONG lengths[2] = {kSide, kSide};
    if (DftiCreateDescriptor(&descriptor, DFTI_DOUBLE, DFTI_REAL, 2, lengths) != 0) {
        return makeError("DftiCreateDescriptor failed.");
    }
    DftiSetValue(descriptor, DFTI_PLACEMENT, DFTI_NOT_INPLACE);
    DftiSetValue(descriptor, DFTI_THREAD_LIMIT, 1);
    DftiSetValue(descriptor, DFTI_CONJUGATE_EVEN_STORAGE, DFTI_COMPLEX_COMPLEX);

    MKL_LONG inputStrides[3] = {0, kSide, 1};
    MKL_LONG outputStrides[3] = {0, kComplexWidth, 1};
    DftiSetValue(descriptor, DFTI_INPUT_STRIDES, inputStrides);
    DftiSetValue(descriptor, DFTI_OUTPUT_STRIDES, outputStrides);
    if (DftiCommitDescriptor(descriptor) != 0) {
        DftiFreeDescriptor(&descriptor);
        return makeError("DftiCommitDescriptor failed.");
    }

    std::vector<double> image(kSide * kSide);
    std::vector<double> recoveredImage(kSide * kSide);
    std::vector<MKL_Complex16> imageFt(kFrequencySize);
    for (int index = 0; index < static_cast<int>(image.size()); ++index) {
        image[index] = static_cast<double>(index * index + index * 2 + 1);
    }

    const auto start = std::chrono::high_resolution_clock::now();
    for (int iteration = 0; iteration < iterations; ++iteration) {
        if (DftiComputeForward(descriptor, image.data(), imageFt.data()) != 0) {
            DftiFreeDescriptor(&descriptor);
            return makeError("DftiComputeForward failed.");
        }
        if (DftiComputeBackward(descriptor, imageFt.data(), recoveredImage.data()) != 0) {
            DftiFreeDescriptor(&descriptor);
            return makeError("DftiComputeBackward failed.");
        }
    }
    const auto end = std::chrono::high_resolution_clock::now();
    DftiFreeDescriptor(&descriptor);

    const double totalMs = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end - start).count();
    std::ostringstream message;
    message << "MKL FFT benchmark finished in " << (totalMs / iterations) << " ms per iteration";

    ToolRunResult result;
    result.success = true;
    result.message = message.str();
    return result;
}

ToolRunResult RunVoltageShiftMatrixTool(int skipValues, int sampleCount) {
    try {
        const auto xVoltage = readSignalColumn("XVoltageOnly.csv", skipValues, sampleCount, 0);
        const auto xShiftFromX = readSignalColumn("XVoltageOnly.csv", skipValues, sampleCount, 2);
        const auto yShiftFromX = readSignalColumn("XVoltageOnly.csv", skipValues, sampleCount, 3);

        const auto yVoltage = readSignalColumn("YVoltageOnly.csv", skipValues, sampleCount, 1);
        const auto xShiftFromY = readSignalColumn("YVoltageOnly.csv", skipValues, sampleCount, 2);
        const auto yShiftFromY = readSignalColumn("YVoltageOnly.csv", skipValues, sampleCount, 3);

        const double slopeXVx = computeSlope(xVoltage, xShiftFromX);
        const double slopeYVx = computeSlope(xVoltage, yShiftFromX);
        const double slopeXVy = computeSlope(yVoltage, xShiftFromY);
        const double slopeYVy = computeSlope(yVoltage, yShiftFromY);

        const double determinant = slopeXVx * slopeYVy - slopeXVy * slopeYVx;
        if (std::fabs(determinant) < std::numeric_limits<double>::epsilon()) {
            return makeError("Calibration matrix determinant is zero.");
        }

        TipTiltConfig config = LoadTipTiltConfig(GetDefaultTipTiltConfigPath());
        config.correctionA00 = slopeYVy / determinant;
        config.correctionA01 = -slopeXVy / determinant;
        config.correctionA10 = -slopeYVx / determinant;
        config.correctionA11 = slopeXVx / determinant;
        config.calibrationClM00 = slopeXVx;
        config.calibrationClM10 = slopeYVx;
        config.calibrationClM01 = slopeXVy;
        config.calibrationClM11 = slopeYVy;

        std::string errorMessage;
        if (!SaveTipTiltConfig(config, GetDefaultTipTiltConfigPath(), &errorMessage)) {
            return makeError(errorMessage);
        }

        std::ostringstream message;
        message << "Updated CalibrationMatrix.csv with A=["
                << config.correctionA00 << ", "
                << config.correctionA01 << "; "
                << config.correctionA10 << ", "
                << config.correctionA11 << "]";

        ToolRunResult result;
        result.success = true;
        result.message = message.str();
        return result;
    } catch (const std::exception& ex) {
        return makeError(ex.what());
    }
}

ToolRunResult RunSerialDebugTool(int comPort, int baudRate, const std::string& payload, int bytesToRead) {
    if (comPort <= 0) {
        return makeError("COM port must be greater than zero.");
    }
    if (baudRate <= 0) {
        return makeError("Baud rate must be greater than zero.");
    }

    const int connectStatus = createSerialConnection(comPort, baudRate, 8, ONESTOPBIT, NOPARITY);
    if (connectStatus != 0) {
        return makeError("Failed to open serial connection.");
    }

    ToolRunResult result;
    if (writeToPort(payload) != 0) {
        closeSerialConnection();
        return makeError("Failed to write serial payload.");
    }

    std::ostringstream message;
    message << "Sent serial payload on COM" << comPort;

    if (bytesToRead > 0) {
        std::vector<char> buffer(static_cast<size_t>(bytesToRead) + 1, '\0');
        DWORD bytesRead = 0;
        if (readFromPort(buffer.data(), static_cast<DWORD>(bytesToRead), &bytesRead) != 0) {
            closeSerialConnection();
            return makeError("Failed to read serial response.");
        }

        buffer[static_cast<size_t>(bytesRead)] = '\0';
        message << "; read " << bytesRead << " bytes: " << buffer.data();
    }

    closeSerialConnection();
    result.success = true;
    result.message = message.str();
    return result;
}

ToolRunResult ConnectMotorControllerTool(int comPort) {
    if (comPort <= 0) {
        return makeError("Motor controller COM port must be greater than zero.");
    }
    if (CreateControllerConnection(comPort) != 0) {
        return makeError("Failed to connect motor controller.");
    }

    ToolRunResult result;
    result.success = true;
    result.message = "Connected motor controller on COM" + std::to_string(comPort);
    return result;
}

ToolRunResult DisconnectMotorControllerTool() {
    if (closeControllerConnection() != 0) {
        return makeError("Failed to close motor controller connection.");
    }

    ToolRunResult result;
    result.success = true;
    result.message = "Motor controller connection closed.";
    return result;
}

ToolRunResult SetMotorFrequencyTool(int motorNum, int frequency) {
    if (setMotorFrequency(motorNum, frequency) != 0) {
        return makeError("Failed to set motor frequency.");
    }

    ToolRunResult result;
    result.success = true;
    result.message = "Set motor " + std::to_string(motorNum) + " frequency to " + std::to_string(frequency);
    return result;
}

ToolRunResult MoveMotorTool(int motorNum, int direction, int counts) {
    if (setMotorCount(motorNum, direction, counts) != 0) {
        return makeError("Failed to move motor.");
    }

    ToolRunResult result;
    result.success = true;
    result.message = "Moved motor " + std::to_string(motorNum) + " by " + std::to_string(counts) + " counts.";
    return result;
}

ToolRunResult SetMotorEnabledTool(int motorNum, bool enabled) {
    const int status = enabled ? enableMotor(motorNum) : disableMotor(motorNum);
    if (status != 0) {
        return makeError(enabled ? "Failed to enable motor." : "Failed to disable motor.");
    }

    ToolRunResult result;
    result.success = true;
    result.message = std::string(enabled ? "Enabled" : "Disabled") + " motor " + std::to_string(motorNum);
    return result;
}

ToolRunResult ExitMotorTool(int motorNum) {
    if (exitMotor(motorNum) != 0) {
        return makeError("Failed to exit motor.");
    }

    ToolRunResult result;
    result.success = true;
    result.message = "Exited motor " + std::to_string(motorNum);
    return result;
}
