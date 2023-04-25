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
#include "utilheaders.h"

using namespace std;

// External variables
//extern double XShift, YShift;
string XCommand, YCommand;
HANDLE XPort, YPort;
extern double A00, A01, A10, A11;
char COMMAND[16];
DWORD dwBytesWritten;
int nWriteBytes;
int Err;
extern char SAVEPATH[64];
//extern ofstream logfile;
LPDWORD COMMERROR;
LPCOMSTAT COMMSTATUS;
TaskHandle XDAQHandle;
//extern float64 XData[1];
//extern float64 YData[1];
extern double Vxoff, Vyoff, SlewRate;
extern double Kp, Kd, Ki;
extern int Nd, Ni;
extern int (*loggingfunc) (std::string);
extern char logString[100000];

int openXYSerialPorts();
int closeXYSerialPorts();
HANDLE getSerialHandle(const char* COMPort);
int sendCommand(HANDLE SerialHandle, string Command);
int initDAQ();
int closeDAQ();
inline int setVoltagesXY(float, float);
int getCalibrationMatrix();
double XData[2];
double YData[1];
int openXYSerialPorts(){
    char COMPort[] = "COM1";
    // Read from file
//    ifstream config("CalibrationMatrix.csv");
//    string Line, Value;
//    stringstream Row;
    int COMX, COMY;
//    for (int i=0; i<6; i++) getline(config, Line, '\n');
//    Row = stringstream(Line);
//    getline(Row, Value, ',');
//    COMX = stoi(Value);
//    getline(Row, Value, ',');
//    COMY = stoi(Value);
//     Port for X
     cout << "Enter COM port for X-actuator control : ";
//     cin >> COMPort;
     cin >> COMX;
    COMPort[3] = 48 + COMX;
    cout << "Using " << COMPort << " for actuator X control" << endl;
    XPort = getSerialHandle(COMPort);
    if (XPort == NULL) {
        cout << "Failed opening COMPort " << COMPort << " for actuator X control" << endl;
        return -1;
    }
    // // Port for Y
     cout << "Enter COM port for Y-actuator control : ";
//     cin >> COMPort;
    cin >> COMY;
    COMPort[3] = 48 + COMY;
    cout << "Using " << COMPort << " for actuator Y control" << endl;
    YPort = getSerialHandle(COMPort);
    if (YPort == NULL) {
        cout << "Failed opening COMPort " << COMPort << " for actuator Y control" << endl;
        return -2;
    }
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

    if (SerialHandle == INVALID_HANDLE_VALUE) {
        return NULL;
    }
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

int sendCommand(HANDLE SerialHandle, string Command){
    strncpy(COMMAND, Command.c_str(), 14);
    sprintf(logString, "Command : %s", Command.c_str());
    loggingfunc(logString);
    PurgeComm(SerialHandle, PURGE_RXABORT|PURGE_RXCLEAR|PURGE_TXABORT|PURGE_TXCLEAR);
    ClearCommError(SerialHandle, COMMERROR, COMMSTATUS);
    sprintf(logString, "Writing to SerialPort : %d", WriteFile(SerialHandle, COMMAND, 16, &dwBytesWritten, NULL));
    loggingfunc(logString);
    return 0;
}

int initDAQ(){
    sprintf(logString,  "Creating DAQ handle for X : %d", DAQmxCreateTask("", &XDAQHandle));
    loggingfunc(logString);

    sprintf(logString, "Creating Analog Out for X : %ld", DAQmxCreateAOVoltageChan(XDAQHandle, "Dev1/ao0:1", "",
                                                                                   0, 10, DAQmx_Val_Volts, ""));
    loggingfunc(logString);

//    sprintf(logString, "Configuring Hardware Timed Output for X/Y : %d", DAQmxCfgOutputBuffer(XDAQHandle, 0));
//    loggingfunc(logString);

    sprintf(logString, "Initializing the DAQ handle for X : %d", DAQmxStartTask(XDAQHandle));
    loggingfunc(logString);


//    sprintf(logString, "Creating DAQ handle for Y : %d", DAQmxCreateTask("", &YDAQHandle));
//    loggingfunc(logString);
//
//    sprintf(logString, "Creating Analog Out for Y : %ld", DAQmxCreateAOVoltageChan(YDAQHandle, "Dev1/ao1", "",
//                                                                0, 10, DAQmx_Val_Volts, ""));
//    loggingfunc(logString);

//    sprintf(logString, "Initializing the DAQ handle for Y : %d", DAQmxStartTask(YDAQHandle));
//    loggingfunc(logString);

    sprintf(logString, "Enabling modulation input for X actuator ");
    loggingfunc(logString);

//    Err = sendCommand(XPort, "stat");
//    Err = sendCommand(YPort, "stat");

    Err = sendCommand(XPort, "modon,0,1");
    sprintf(logString, "Enabling modulation input for Y actuator ");
    log(logString);
    Err = sendCommand(YPort, "modon,0,1");
    return 0;
}

inline int setVoltagesXY(float XShift, float YShift){
    int32 sampsPerChanWritten=0;

//    float64 YData[1];
//    sprintf(logString, "Actuator voltage for X : %.2f", XShift);
//    loggingfunc(logString);
//    sprintf(logString, "Actuator voltage for Y : %.2f", YShift);
//    loggingfunc(logString);

    XData[0] = (XShift+20.0)/15.0;
    XData[1] = (YShift+20.0)/15.0;
//    sprintf(logString, "Modulation voltage for X : %.2f", XData[0]);
//    loggingfunc(logString);
//    sprintf(logString, "Modulation voltage for Y : %.2f", YData[0]);
//    loggingfunc(logString);
//    t0 = chrono::high_resolution_clock::now();
    DAQmxWriteAnalogF64(
            XDAQHandle, 1, 1, 0.0, DAQmx_Val_GroupByChannel, XData, NULL, NULL);
//    DAQmxWriteAnalogF64(
//            YDAQHandle, 1, 1, 0.0, DAQmx_Val_GroupByScanNumber, YData, NULL, NULL);
//    t1 = chrono::high_resolution_clock::now();
//    dt = chrono::duration_cast<chrono::duration<double>>(t1 - t0);
//    cout << "   Time to apply: "<<dt.count() << endl;
    //    status = DAQmxWriteRaw(XDAQHandle, 1, 0, 0, XData, &sampsPerChanWritten, NULL);
    //    sprintf(logString, "Applying voltage for X : %d", status);
//    loggingfunc(logString);
//    status = DAQmxWriteAnalogF64(YDAQHandle, 1, 0, 0.0, DAQmx_Val_GroupByScanNumber, YData, NULL, NULL);
//    sprintf(logString, "Applying voltage for Y : %d", status);
//    loggingfunc(logString);
    return 0;
}

int closeDAQ(){
    sprintf(logString, "Disabling modulation input for X actuator ");
    loggingfunc(logString);
    Err = sendCommand(XPort, "modon,0,0");
    sprintf(logString, "Disabling modulation input for Y actuator ");
    loggingfunc(logString);
    Err = sendCommand(YPort, "modon,0,0");
    sprintf(logString, "Stopping the DAQ handle for X : %d", DAQmxStopTask(XDAQHandle));
    loggingfunc(logString);
    sprintf(logString, "Clearing the DAQ handle for X : %d", DAQmxClearTask(XDAQHandle));
    loggingfunc(logString);
//    sprintf(logString, "Stopping the DAQ handle for Y : %d", DAQmxStopTask(YDAQHandle));
//    loggingfunc(logString);
//    sprintf(logString, "Clearing the DAQ handle for Y : %d", DAQmxClearTask(YDAQHandle));
//    loggingfunc(logString);
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
//    cout << "Inverse calibration matrix is : " << A00 << ", " << A01 << ", " << A10 << ", " << A11 << endl;
    sprintf(logString, "Inverse calibration matrix is : %.2f, %.2f, %.2f, %.2f",A00, A01, A10, A11);
    log(logString);
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
//    cout << "Calibration matrix is : " << CM[0] << ", " << CM[1] << ", " << CM[2] << ", " << CM[3] << endl;
    sprintf(logString, "Calibration matrix is : %.2f, %.2f, %.2f, %.2f", CM[0], CM[1], CM[2], CM[3]);
    log(logString);
    // Line 3
    getline(calmat, Line, '\n');
    Row = stringstream(Line);
    getline(Row, Value, ',');
    Vxoff = stod(Value);
    getline(Row, Value, ',');
    Vyoff = stod(Value);
    getline(Row, Value, ',');
    SlewRate = stod(Value);
//    cout << "Vx offset : " << Vxoff << endl;
//    cout << "Vy offset : " << Vyoff << endl;
//    cout << "Slew rate is : " << SlewRate << endl;
    sprintf(logString, "Vx offset : %.2f", Vxoff);
    log(logString);
    sprintf(logString, "Vy offset : %.2f", Vyoff);
    log(logString);
    sprintf(logString, "Slew rate is : %.2f", SlewRate);
    log(logString);
    // Line 4
    getline(calmat, Line, '\n');
    Row = stringstream(Line);
    getline(Row, Value, ',');
    Kp = stod(Value);
    getline(Row, Value, ',');
    Kd = stod(Value);
    getline(Row, Value, ',');
    Ki = stod(Value);
//    cout << "Proportional constant : " << Kp << endl;
//    cout << "Differential constant : " << Kd << endl;
//    cout << "Integral constant : " << Ki << endl;
    sprintf(logString, "Proportional constant : %.2f", Kp);
    log(logString);
    sprintf(logString, "Differential constant : %.2f", Kd);
    log(logString);
    sprintf(logString, "Integral constant : %.2f", Ki);
    log(logString);
    // Line 5
    getline(calmat, Line, '\n');
    Row = stringstream(Line);
    getline(Row, Value, ',');
    Ni = stoi(Value);
    getline(Row, Value, ',');
    Nd = stoi(Value);
//    cout << "Numer of points to compute integral error : " << Ni << endl;
//    cout << "Numer of points to compute differential error : " << Nd << endl;
    sprintf(logString, "Numer of points to compute integral error : %d", Ni);
    log(logString);
    sprintf(logString, "Numer of points to compute differential error : %d",Nd);
    log(logString);
    calmat.close();
    return 0;
}

#endif
