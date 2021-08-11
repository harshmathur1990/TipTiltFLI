#ifndef CONTROLS_H
#define CONTROLS_H

//#include <unistd.h>
#include <sstream>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <cstdint>
#include <chrono>
#include <string>
#include <tuple>
#include <Windows.h>
#include <ctime>
#include <queue>
#include "NIDAQmx.h"

using namespace std;

// External variables
extern tuple<double, double> XYIND;
extern double XShift, YShift;
extern string XCommand, YCommand;
extern HANDLE XPort, YPort;
extern double A00, A01, A10, A11;
extern char COMMAND[16];
extern DWORD dwBytesWritten;
extern int nWriteBytes;
extern int Err;
extern char SAVEPATH[64];
extern ofstream logfile;
extern LPDWORD COMMERROR;
extern LPCOMSTAT COMMSTATUS;
extern TaskHandle XDAQHandle;
extern TaskHandle YDAQHandle;
extern float64 XData[1];
extern float64 YData[1];
extern double A00, A01, A10, A11;
extern double Vxoff, Vyoff, SlewRate;
extern double Kp, Kd, Ki;
extern int Nd, Ni;


int openXYSerialPorts();
int closeXYSerialPorts();
HANDLE getSerialHandle(const char* COMPort);
int sendCommand(HANDLE SerialHandle, string Command);
int moveXY(tuple<double, double> XYIND);
int sendVXY(tuple<double, double> VoltageXY);
int initDAQ();
int closeDAQ();
int setVoltagesXY();
int getCalibrationMatrix();

int openXYSerialPorts(){
    char COMPort[] = "COM1";
    // Read from file
    ifstream config("CalibrationMatrix.csv");
    string Line, Value;
    stringstream Row;
    int COMX, COMY;
    for (int i=0; i<6; i++) getline(config, Line, '\n');
    Row = stringstream(Line);
    getline(Row, Value, ',');
    COMX = stoi(Value);
    getline(Row, Value, ',');
    COMY = stoi(Value);
    // Port for X
    // cout << "Enter COM port for X-actuator control : ";
    // cin >> COMPort;
    COMPort[3] = 48 + COMX;
    cout << "Using " << COMPort << " for actuator X control" << endl;
    XPort = getSerialHandle(COMPort);
    // // Port for Y
    // cout << "Enter COM port for Y-actuator control : ";
    // cin >> COMPort;
    COMPort[3] = 48 + COMY;
    cout << "Using " << COMPort << " for actuator Y control" << endl;
    YPort = getSerialHandle(COMPort);
    return 0;
}

int closeXYSerialPorts(){
    Err = CloseHandle(XPort);
    Err = CloseHandle(YPort);
    return 0;
}

HANDLE getSerialHandle(const char* COMPort){
    // Parameters
    HANDLE SerialHandle;
    COMMTIMEOUTS Timeout = { 0 };

    SerialHandle = CreateFile(COMPort , GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
    cout <<  "Opening serial port : " << COMPort << endl;

    // COM port settings
    DCB SerialParams = { 0 };
    SerialParams.DCBlength = sizeof(SerialParams);
    GetCommState(SerialHandle, &SerialParams);
    SerialParams.BaudRate = 115200;
    SerialParams.ByteSize = 8;
    SerialParams.StopBits = 1;
    SerialParams.Parity = 'N';
    SerialParams.fOutX = TRUE;
    SerialParams.fInX = TRUE;
    SetCommState(SerialHandle, &SerialParams);

    // Set timeouts
    Timeout.ReadIntervalTimeout = 50;
    Timeout.ReadTotalTimeoutConstant = 50;
    Timeout.ReadTotalTimeoutMultiplier = 50;
    Timeout.WriteTotalTimeoutConstant = 50;
    Timeout.WriteTotalTimeoutMultiplier = 10;
    SetCommTimeouts(SerialHandle, &Timeout);

    return SerialHandle;
}

int moveXY(tuple<double, double> XYIND){
    // logfile << A00 << A01 << A10 << A11 << endl;
    XShift = A00*get<0>(XYIND) + A01*get<1>(XYIND);
    YShift = A10*get<0>(XYIND) + A11*get<1>(XYIND);
    logfile << "X-shift measured : " << get<0>(XYIND) << endl;
    logfile << "Y-shift measured : " << get<1>(XYIND) << endl;
    logfile << "X-voltage applied : " << XShift << endl;
    logfile << "Y-voltage applied : " << YShift << endl;
    XCommand = "set,0," + std::to_string(XShift);
    YCommand = "set,0," + std::to_string(YShift);
    sendCommand(XPort, XCommand);
    sendCommand(YPort, YCommand);
    return 0;
}

int sendVXY(){
    logfile << "X-voltage applied : " << XShift << endl;
    logfile << "Y-voltage applied : " << YShift << endl;
    XCommand = "set,0," + std::to_string(XShift);
    YCommand = "set,0," + std::to_string(YShift);
    sendCommand(XPort, XCommand);
    sendCommand(YPort, YCommand);
    return 0;
}

int sendCommand(HANDLE SerialHandle, string Command){
    strncpy(COMMAND, Command.c_str(), 14);
    logfile << "Command : " << Command << endl;
    PurgeComm(SerialHandle, PURGE_RXABORT|PURGE_RXCLEAR|PURGE_TXABORT|PURGE_TXCLEAR);
    ClearCommError(SerialHandle, COMMERROR, COMMSTATUS);
    logfile << "Writing to SerialPort : " << WriteFile(SerialHandle, COMMAND, 16, &dwBytesWritten, NULL) << endl;
    return 0;
}

int initDAQ(){
    logfile << "Creating DAQ handle for X : " << DAQmxCreateTask("", &XDAQHandle) << endl;
    logfile << "Creating Analog Out for X : " << DAQmxCreateAOVoltageChan(XDAQHandle, "Dev1/ao0", "", 
                                                                0.3, 2.3, DAQmx_Val_Volts, "") << endl;
    logfile << "Initializing the DAQ handle for X : " << DAQmxStartTask(XDAQHandle) << endl;

    logfile << "Creating DAQ handle for Y : " << DAQmxCreateTask("", &YDAQHandle) << endl;
    logfile << "Creating Analog Out for Y : " << DAQmxCreateAOVoltageChan(YDAQHandle, "Dev1/ao1", "", 
                                                                0.3, 2.3, DAQmx_Val_Volts, "") << endl;
    logfile << "Initializing the DAQ handle for Y : " << DAQmxStartTask(YDAQHandle) << endl;

    logfile << "Enabling modulation input for X actuator " << endl;
    Err = sendCommand(XPort, "modon,0,1");
    logfile << "Enabling modulation input for Y actuator " << endl;
    Err = sendCommand(YPort, "modon,0,1");
    return 0;
}

int setVoltagesXY(){
    logfile << "Actuator voltage for X : " << XShift << endl;
    logfile << "Actuator voltage for Y : " << YShift << endl;
    XData[0] = (XShift+20.0)/15.0;
    YData[0] = (YShift+20.0)/15.0;
    logfile << "Modulation voltage for X : " << XData[0] << endl;
    logfile << "Modulation voltage for Y : " << YData[0] << endl;
    logfile << "Applying voltage for X : " << DAQmxWriteAnalogF64(XDAQHandle, 1, 1, 0.0, DAQmx_Val_GroupByChannel, XData, NULL, NULL) << endl;
    logfile << "Applying voltage for Y : " << DAQmxWriteAnalogF64(YDAQHandle, 1, 1, 0.0, DAQmx_Val_GroupByChannel, YData, NULL, NULL) << endl;
    return 0; 
}

int closeDAQ(){
    logfile << "Disabling modulation input for X actuator " << endl;
    Err = sendCommand(XPort, "modon,0,0");
    logfile << "Disabling modulation input for Y actuator " << endl;
    Err = sendCommand(YPort, "modon,0,0");
    logfile << "Stopping the DAQ handle for X : " << DAQmxStopTask(XDAQHandle) << endl;
    logfile << "Clearing the DAQ handle for X : " << DAQmxClearTask(XDAQHandle) << endl;
    logfile << "Stopping the DAQ handle for Y : " << DAQmxStopTask(YDAQHandle) << endl;
    logfile << "Clearing the DAQ handle for Y : " << DAQmxClearTask(YDAQHandle) << endl;
    return 0;
}

int getCalibrationMatrix(){
    double CM[4];
    ifstream calmat("CalibrationMatrix.csv");
    string Line, Value;
    stringstream Row;
    // Line 1
    getline(calmat, Line, '\n');
    Row = stringstream(Line);
    getline(Row, Value, ',');
    CM[0] = stod(Value);
    getline(Row, Value, ',');
    CM[1] = stod(Value);
    getline(Row, Value, ',');
    CM[2] = stod(Value);
    getline(Row, Value, ',');
    CM[3] = stod(Value);
    A00 = CM[0];
    A01 = CM[1];
    A10 = CM[2];
    A11 = CM[3];
    cout << "Inverse calibration matrix is : " << A00 << ", " << A01 << ", " << A10 << ", " << A11 << endl;
    logfile << "Inverse calibration matrix is : " << A00 << ", " << A01 << ", " << A10 << ", " << A11 << endl;
    // Line 2
    getline(calmat, Line, '\n');
    Row = stringstream(Line);
    getline(Row, Value, ',');
    CM[0] = stod(Value);
    getline(Row, Value, ',');
    CM[1] = stod(Value);
    getline(Row, Value, ',');
    CM[2] = stod(Value);
    getline(Row, Value, ',');
    CM[3] = stod(Value);
    cout << "Calibration matrix is : " << CM[0] << ", " << CM[1] << ", " << CM[2] << ", " << CM[3] << endl;
    logfile << "Calibration matrix is : " << CM[0] << ", " << CM[1] << ", " << CM[2] << ", " << CM[3] << endl;
    // Line 3
    getline(calmat, Line, '\n');
    Row = stringstream(Line);
    getline(Row, Value, ',');
    Vxoff = stod(Value);
    getline(Row, Value, ',');
    Vyoff = stod(Value);
    getline(Row, Value, ',');
    SlewRate = stod(Value);
    cout << "Vx offset : " << Vxoff << endl;
    cout << "Vy offset : " << Vyoff << endl;
    cout << "Slew rate is : " << SlewRate << endl;
    logfile << "Vx offset : " << Vxoff << endl;
    logfile << "Vy offset : " << Vyoff << endl;
    logfile << "Slew rate is : " << SlewRate << endl;
    // Line 4
    getline(calmat, Line, '\n');
    Row = stringstream(Line);
    getline(Row, Value, ',');
    Kp = stod(Value);
    getline(Row, Value, ',');
    Kd = stod(Value);
    getline(Row, Value, ',');
    Ki = stod(Value);
    cout << "Proportional constant : " << Kp << endl;
    cout << "Differential constant : " << Kd << endl;
    cout << "Integral constant : " << Ki << endl;
    logfile << "Proportional constant : " << Kp << endl;
    logfile << "Differential constant : " << Kd << endl;
    logfile << "Integral constant : " << Ki << endl;
    // Line 5
    getline(calmat, Line, '\n');
    Row = stringstream(Line);
    getline(Row, Value, ',');
    Ni = stoi(Value);
    getline(Row, Value, ',');
    Nd = stoi(Value);
    cout << "Numer of points to compute integral error : " << Ni << endl;
    cout << "Numer of points to compute differential error : " << Nd << endl;
    logfile << "Numer of points to compute integral error : " << Ni << endl;
    logfile << "Numer of points to compute differential error : " << Nd << endl;
    calmat.close();
    return 0;
}

#endif
