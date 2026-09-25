#ifndef TIPTILT_TOOLS_H
#define TIPTILT_TOOLS_H

#include <string>

struct ToolRunResult {
    bool success = false;
    std::string message;
};

ToolRunResult RunImageBinningBenchmarkTool();
ToolRunResult RunMklBenchmarkTool(int iterations);
ToolRunResult RunVoltageShiftMatrixTool(int skipValues, int sampleCount);
ToolRunResult RunSerialDebugTool(int comPort, int baudRate, const std::string& payload, int bytesToRead);
ToolRunResult ConnectMotorControllerTool(int comPort);
ToolRunResult DisconnectMotorControllerTool();
ToolRunResult SetMotorFrequencyTool(int motorNum, int frequency);
ToolRunResult MoveMotorTool(int motorNum, int direction, int counts);
ToolRunResult SetMotorEnabledTool(int motorNum, bool enabled);
ToolRunResult ExitMotorTool(int motorNum);

#endif
