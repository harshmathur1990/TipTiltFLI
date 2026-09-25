#include "controls.h"

#include <cstdio>
#include <cstring>
#include <exception>

#include "tiptilt_config.h"
#include "utilheaders.h"

std::string XCommand;
std::string YCommand;
HANDLE XPort = nullptr;
HANDLE YPort = nullptr;
char COMMAND[16] = {};
DWORD dwBytesWritten = 0;
int nWriteBytes = 0;
int Err = 0;
DWORD commErrorStorage = 0;
COMSTAT commStatusStorage = {};
LPDWORD COMMERROR = &commErrorStorage;
LPCOMSTAT COMMSTATUS = &commStatusStorage;
TaskHandle XDAQHandle = nullptr;
double XData[2] = {};
double YData[1] = {};

int openXYSerialPorts(int xComPort, int yComPort) {
    std::string xComPortName = std::string("\\\\.\\COM") + std::to_string(xComPort);
    log_info("Using " + xComPortName + " for actuator X control");
    XPort = getSerialHandle(xComPortName.c_str());
    if (XPort == nullptr) {
        log_error("Failed opening COM port " + xComPortName + " for actuator X control");
        return -1;
    }

    std::string yComPortName = std::string("\\\\.\\COM") + std::to_string(yComPort);
    log_info("Using " + yComPortName + " for actuator Y control");
    YPort = getSerialHandle(yComPortName.c_str());
    if (YPort == nullptr) {
        log_error("Failed opening COM port " + yComPortName + " for actuator Y control");
        return -2;
    }

    return 0;
}

int closeXYSerialPorts() {
    Err = CloseHandle(XPort);
    Err = CloseHandle(YPort);
    return 0;
}

HANDLE getSerialHandle(const char* comPort) {
    COMMTIMEOUTS timeout = {0};
    HANDLE serialHandle = CreateFile(
        comPort,
        GENERIC_READ | GENERIC_WRITE,
        0,
        0,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        0
    );
    log_info(std::string("Opening serial port: ") + comPort);

    if (serialHandle == INVALID_HANDLE_VALUE) {
        return nullptr;
    }

    DCB serialParams = {0};
    serialParams.DCBlength = sizeof(serialParams);
    GetCommState(serialHandle, &serialParams);
    serialParams.BaudRate = 115200;
    serialParams.ByteSize = 8;
    serialParams.StopBits = 1;
    serialParams.Parity = 'N';
    serialParams.fOutX = TRUE;
    serialParams.fInX = TRUE;
    SetCommState(serialHandle, &serialParams);

    timeout.ReadIntervalTimeout = 50;
    timeout.ReadTotalTimeoutConstant = 50;
    timeout.ReadTotalTimeoutMultiplier = 50;
    timeout.WriteTotalTimeoutConstant = 50;
    timeout.WriteTotalTimeoutMultiplier = 10;
    SetCommTimeouts(serialHandle, &timeout);

    return serialHandle;
}

int sendCommand(HANDLE serialHandle, std::string command) {
    strncpy_s(COMMAND, 16, command.c_str(), 14);
    COMMAND[14] = 13;
    COMMAND[15] = 10;
    log_debug("Command: " + command);
    PurgeComm(serialHandle, PURGE_RXABORT | PURGE_RXCLEAR | PURGE_TXABORT | PURGE_TXCLEAR);
    ClearCommError(serialHandle, COMMERROR, COMMSTATUS);
    const int writeResult = WriteFile(serialHandle, COMMAND, 16, &dwBytesWritten, NULL);
    log_debug("Writing to serial port returned: " + std::to_string(writeResult));
    return 0;
}

int initDAQ() {
    log_info("Creating DAQ handle for X: " + std::to_string(DAQmxCreateTask("", &XDAQHandle)));

    log_info(
        "Creating Analog Out for X: " + std::to_string(
            DAQmxCreateAOVoltageChan(XDAQHandle, "Dev1/ao0:1", "", 0, 10, DAQmx_Val_Volts, "")
        )
    );

    log_info("Initializing the DAQ handle for X: " + std::to_string(DAQmxStartTask(XDAQHandle)));

    log_info("Enabling modulation input for X actuator");
    Err = sendCommand(XPort, "modon,0,1");

    log_info("Enabling modulation input for Y actuator");
    Err = sendCommand(YPort, "modon,0,1");
    return 0;
}

int setVoltagesXY(double xShift, double yShift) {
    XData[0] = (xShift + 20.0) / 15.0;
    XData[1] = (yShift + 20.0) / 15.0;
    DAQmxWriteAnalogF64(
        XDAQHandle,
        1,
        0,
        0.0,
        DAQmx_Val_GroupByChannel,
        XData,
        NULL,
        NULL
    );
    return 0;
}

int closeDAQ() {
    log_info("Disabling modulation input for X actuator");
    Err = sendCommand(XPort, "modon,0,0");

    log_info("Disabling modulation input for Y actuator");
    Err = sendCommand(YPort, "modon,0,0");

    log_info("Stopping the DAQ handle for X: " + std::to_string(DAQmxStopTask(XDAQHandle)));
    log_info("Clearing the DAQ handle for X: " + std::to_string(DAQmxClearTask(XDAQHandle)));
    return 0;
}

int getCalibrationMatrix() {
    try {
        const TipTiltConfig config = LoadTipTiltConfig(GetDefaultTipTiltConfigPath());
        ApplyTipTiltConfigToGlobals(config);
        return 0;
    } catch (const std::exception& ex) {
        log_error("Failed loading calibration config: " + std::string(ex.what()));
        return -1;
    }
}
