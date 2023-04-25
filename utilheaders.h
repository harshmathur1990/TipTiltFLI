//
// Created by IIAP-IPC on 03/02/2021.
//

#ifndef TIPTILT_UTILHEADERS_H
#define TIPTILT_UTILHEADERS_H
#include <iostream>
#include <fstream>
#include <ctime>
#include <string>
#include <vector>
#include <stdlib.h>
#include <cstdint>
#include <chrono>
#include <tuple>
#include <Windows.h>

extern std::string loggingFilename;
extern std::ofstream outfile;
extern std::string xLogCalibrationFilename;
extern std::ofstream outfileXLogCalibrationFilename;
extern std::string yLogCalibrationFilename;
extern std::ofstream outfileYLogCalibrationFilename;
extern std::string xVoltageOnlyFilename;
extern std::ofstream outfileXVoltageOnlyFilename;
extern std::string yVoltageOnlyFilename;
extern std::ofstream outfileYVoltageOnlyFilename;
extern std::string currentDirectoryXVoltageOnlyFilename;
extern std::ofstream currentDirectoryOutfileXVoltageOnlyFilename;
extern std::string currentDirectoryYVoltageOnlyFilename;
extern std::ofstream currentDirectoryOutfileYVoltageOnlyFilename;
extern std::ofstream shiftUncorrected;
extern std::ofstream shift;
extern std::ofstream opLog;
extern char logString[100000];
extern char SAVEPATH[64];

int log(std::string logString);

int setupLogging(int mode);
int getDateTimeDirName();
int setProgressBar(int i, int NFRAMES);
int xlog(std::string logString);
int ylog(std::string logString);
int xvoltagefilelog(std::string logString);
int yvoltagefilelog(std::string logString);
int shift_log(std::string logString);
int shift_uncorected_log(std::string logString);
int oplog_write(std::string logString);

#endif //TIPTILT_UTILHEADERS_H
