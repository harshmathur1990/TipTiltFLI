#ifndef COMMON_H
#define COMMON_H

//#include <unistd.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <cstdint>
#include <chrono>
#include <string>
#include <tuple>
#include <Windows.h>
#include <ctime>

using namespace std;

extern ofstream logfile;
extern char SAVEPATH[64];

int getDateTimeDirName();
int setProgressBar(int i, int NFRAMES);

int getDateTimeDirName(){
    std::time_t TIME = std::time(NULL);
    std::strftime(SAVEPATH, sizeof(SAVEPATH), "\\%Y%m%d_%H%M%S", std::localtime(&TIME));
    cout << "Auto-save directory : " << SAVEPATH << endl;
    // CreateDirectory(SAVEPATH ,NULL);
    logfile << "Creating auto-save directory : " << string(SAVEPATH) << endl;
    return 0;
}

int setProgressBar(int i, int NFRAMES){
    // Progress bar
    cout << " ";
    for (int k=0; k<i*50/NFRAMES; k++) cout << '=';
    cout << "  " << i*100/NFRAMES << "%   \r";
    cout.flush();
    return 0;
}

#endif