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
extern double ClM00, ClM01, ClM10, ClM11;
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
extern double AA00, AA01, AA10, AA11; // Autoguider control matrix
extern double tau;
extern double Akp, Aki, Akd;
extern int autoGuiderCorrectionTime;
extern double autoGuiderOffloadLimitX, autoGuiderOffloadLimitY;
extern int imageSaveAfterSecond;
/* Output limits */
extern double limMin;
extern double limMax;

/* Integrator limits */
extern double limMinInt;
extern double limMaxInt;

extern int liveView;

extern bool useCameraFlat;
extern string NucMode;

extern bool autoPMode;
extern double minKp, maxKp, minKd, maxKd;
extern double mp, cp, md, cd;

extern double cutOffFrequencyOfDerivativeError;

/* Sample time (in seconds) */
extern double sampleTime;
extern int (*loggingfunc) (std::string);
extern char logString[100000];

int openXYSerialPorts();
int closeXYSerialPorts();
HANDLE getSerialHandle(const char* COMPort);
int sendCommand(HANDLE SerialHandle, string Command);
int initDAQ();
int closeDAQ();
inline int setVoltagesXY(double, double);
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
//    strncpy(COMMAND, Command.c_str(), 14);
    strncpy_s(COMMAND, 16, Command.c_str(), 14);
    COMMAND[14] = 13; // CR
    COMMAND[15] = 10; // LF
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

//    sprintf(logString, "Configuring Buffer as 1 for X/Y : %d", DAQmxCfgOutputBuffer(XDAQHandle, 1));
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
    Err = sendCommand(YPort, "modon,0,1")
            ;
    return 0;
}

inline int setVoltagesXY(double XShift, double YShift){
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
            XDAQHandle, 1, 0, 0.0, DAQmx_Val_GroupByChannel, XData, NULL, NULL);
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
    ClM00 = CM[0];
    ClM01 = CM[1];
    ClM10 = CM[2];
    ClM11 = CM[3];
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

    getline(calmat, Line, '\n');
    Row = stringstream(Line);
    getline(Row, Value, ',');
    AA00 = stoi(Value);
    getline(Row, Value, ',');
    AA01 = stoi(Value);
    getline(Row, Value, ',');
    AA10 = stoi(Value);
    getline(Row, Value, ',');
    AA11 = stoi(Value);

    sprintf(logString, "Autoguider control matrix is : %.2f, %.2f, %.2f, %.2f", AA00, AA01, AA10, AA11);
    log(logString);

    getline(calmat, Line, '\n');
    Row = stringstream(Line);
    getline(Row, Value, ',');
    tau = stod(Value);
    getline(Row, Value, ',');
    limMin = stod(Value);
    getline(Row, Value, ',');
    limMax = stod(Value);
    getline(Row, Value, ',');
    limMinInt = stod(Value);
    getline(Row, Value, ',');
    limMaxInt = stod(Value);
    getline(Row, Value, ',');
    sampleTime = stod(Value);

    sprintf(logString, "tau, limMin, limMax, limMinInt, limMaxInt, sampleTime : %.4f, %.4f, %.4f, %.4f, %.4f, %.4f", tau, limMin, limMax, limMinInt, limMaxInt, sampleTime);
    log(logString);

    getline(calmat, Line, '\n');
    Row = stringstream(Line);
    getline(Row, Value, ',');
    Akp = stod(Value);
    getline(Row, Value, ',');
    Akd = stod(Value);
    getline(Row, Value, ',');
    Aki = stod(Value);

    sprintf(logString, "Akp, Akd, Aki : %.4f, %.4f, %.4f", Akp, Akd, Aki);
    log(logString);

    getline(calmat, Line, '\n');
    Row = stringstream(Line);
    getline(Row, Value, ',');
    autoGuiderCorrectionTime = stoi(Value);

    sprintf(logString, "autoGuiderCorrectionTime: %d", autoGuiderCorrectionTime);
    log(logString);

    getline(calmat, Line, '\n');
    Row = stringstream(Line);
    getline(Row, Value, ',');
    imageSaveAfterSecond = stoi(Value);

    sprintf(logString, "imageSaveAfterSecond: %d", imageSaveAfterSecond);
    log(logString);

    getline(calmat, Line, '\n');
    Row = stringstream(Line);
    getline(Row, Value, ',');
    autoGuiderOffloadLimitX = stod(Value);
    getline(Row, Value, ',');
    autoGuiderOffloadLimitY = stod(Value);

    sprintf(logString, "autoGuiderOffloadLimitX, autoGuiderOffloadLimitY: %.4f, %.4f", autoGuiderOffloadLimitX, autoGuiderOffloadLimitY);
    log(logString);

    getline(calmat, Line, '\n');
    Row = stringstream(Line);
    getline(Row, Value, ',');
    liveView = stoi(Value);

    sprintf(logString, "liveView: %d", liveView);
    log(logString);

    getline(calmat, Line, '\n');
    Row = stringstream(Line);
    getline(Row, Value, ',');
    int x = stoi(Value);

    if (x) useCameraFlat = true;
    else useCameraFlat = false;

    if (useCameraFlat) NucMode = "BiasFlat";
    else NucMode = "Bias";

    sprintf(logString, "useCameraFlat, NucMode: %d, %s", useCameraFlat, NucMode.c_str());
    log(logString);

    getline(calmat, Line, '\n');
    Row = stringstream(Line);
    getline(Row, Value, ',');
    int hpmode = stoi(Value);

    if (hpmode) autoPMode = true;
    else autoPMode = false;

    sprintf(logString, "hpmode, autoPMode: %d, %d", hpmode, autoPMode);
    log(logString);

    getline(calmat, Line, '\n');
    Row = stringstream(Line);
    getline(Row, Value, ',');
    minKp = stod(Value);
    getline(Row, Value, ',');
    maxKp = stod(Value);
    getline(Row, Value, ',');
    minKd = stod(Value);
    getline(Row, Value, ',');
    maxKd = stod(Value);

    sprintf(logString, "minKp, maxKp, minKd, maxKd: %lf, %lf, %lf, %lf", minKp, maxKp, minKd, maxKd);
    log(logString);

    mp = (maxKp - minKp) / (20 - 0.01);
    cp = maxKp - mp * 20;

    md = (maxKd - minKd) / (maxKp - minKp);
    cd = maxKd - md * maxKp;

    sprintf(logString, "mp, cp, md, cd: %lf, %lf, %lf, %lf", mp, cp, md, cd);
    log(logString);

    getline(calmat, Line, '\n');
    Row = stringstream(Line);
    getline(Row, Value, ',');
    cutOffFrequencyOfDerivativeError = stod(Value);

    sprintf(logString, "cutOffFrequencyOfDerivativeError: %lf", cutOffFrequencyOfDerivativeError);
    log(logString);

    calmat.close();
    return 0;
}

#endif
