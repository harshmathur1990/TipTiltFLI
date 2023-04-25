//
// Created by IIAP-IPC on 03/02/2021.
//

#include "utilheaders.h"

std::string loggingFilename;
std::ofstream outfile;
std::string xLogCalibrationFilename;
std::ofstream outfileXLogCalibrationFilename;
std::string yLogCalibrationFilename;
std::ofstream outfileYLogCalibrationFilename;
std::string xVoltageOnlyFilename;
std::ofstream outfileXVoltageOnlyFilename;
std::string yVoltageOnlyFilename;
std::ofstream outfileYVoltageOnlyFilename;
std::string currentDirectoryXVoltageOnlyFilename;
std::ofstream currentDirectoryOutfileXVoltageOnlyFilename;
std::string currentDirectoryYVoltageOnlyFilename;
std::ofstream currentDirectoryOutfileYVoltageOnlyFilename;
std::ofstream shiftUncorrected;
std::ofstream shift;
std::ofstream opLog;

int log(std::string logString) {
    time_t curr_time;
    tm * curr_tm;
    time(&curr_time);
    curr_tm = localtime(&curr_time);
    char timeString[100];
    std::strftime(timeString, 100, "%Y_%m_%dT%H_%M_%S:: ", curr_tm);
    std::cout << logString << std::endl;
    outfile << timeString << logString << std::endl;
    return 0;
}


int xlog(std::string logString) {
    outfileXLogCalibrationFilename << logString << std::endl;
    return 0;
}

int ylog(std::string logString) {
    outfileYLogCalibrationFilename << logString << std::endl;
    return 0;
}

int xvoltagefilelog(std::string logString) {
    outfileXVoltageOnlyFilename << logString << std::endl;
    currentDirectoryOutfileXVoltageOnlyFilename << logString << std::endl;
    return 0;
}

int yvoltagefilelog(std::string logString) {
    outfileYVoltageOnlyFilename << logString << std::endl;
    currentDirectoryOutfileYVoltageOnlyFilename << logString << std::endl;
    return 0;
}

int shift_log(std::string logString) {
    shift << logString << std::endl;
    return 0;
}

int shift_uncorected_log(std::string logString) {
    shiftUncorrected << logString << std::endl;
    return 0;
}

int oplog_write(std::string logString) {
    opLog << logString << std::endl;
    return 0;
}

int setupLogging(int mode){
    getDateTimeDirName();
    CreateDirectory(SAVEPATH,NULL);
    loggingFilename = std::string(SAVEPATH) + "\\log.txt";
    outfile.open(loggingFilename.c_str());
    if (mode == 1){
        // extra calibration log files
        xLogCalibrationFilename = std::string(SAVEPATH) + "\\Xlog.txt";
        yLogCalibrationFilename = std::string(SAVEPATH) + "\\Ylog.txt";
        outfileXLogCalibrationFilename.open(xLogCalibrationFilename.c_str());
        outfileYLogCalibrationFilename.open(yLogCalibrationFilename.c_str());
        xVoltageOnlyFilename = std::string(SAVEPATH) + "\\XVoltageOnly.csv";
        yVoltageOnlyFilename = std::string(SAVEPATH) + "\\YVoltageOnly.csv";
        outfileXVoltageOnlyFilename.open(xVoltageOnlyFilename.c_str());
        outfileYVoltageOnlyFilename.open(yVoltageOnlyFilename.c_str());
        currentDirectoryXVoltageOnlyFilename = "XVoltageOnly.csv";
        currentDirectoryYVoltageOnlyFilename = "YVoltageOnly.csv";
        currentDirectoryOutfileXVoltageOnlyFilename.open(currentDirectoryXVoltageOnlyFilename.c_str());
        currentDirectoryOutfileYVoltageOnlyFilename.open(currentDirectoryYVoltageOnlyFilename.c_str());
    }
    else if (mode == 2) {
        shiftUncorrected.open(std::string(std::string(SAVEPATH) + "\\Shifts_Uncorrected.csv").c_str());
        shift.open(std::string(std::string(SAVEPATH) + "\\Shifts.csv").c_str());
        opLog.open(std::string(std::string(SAVEPATH) + "\\oplog.txt").c_str());
    }
    return 0;
}

int getDateTimeDirName(){
    std::time_t TIME = std::time(NULL);
    std::strftime(SAVEPATH, sizeof(SAVEPATH), "F:\\tiptilt\\%Y%m%d_%H%M%S", std::localtime(&TIME));
    std::cout << "Auto-save directory : " << SAVEPATH << std::endl;
    return 0;
}

int setProgressBar(int i, int NFRAMES){
    // Progress bar
    std::cout << " ";
    for (int k=0; k<i*50/NFRAMES; k++) std::cout << '=';
    std::cout << "  " << i*100/NFRAMES << "%   \r";
    std::cout.flush();
    return 0;
}