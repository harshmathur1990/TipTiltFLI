#ifndef DETECTOR_H
#define DETECTOR_H

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <fftw3.h>
#include <fstream>
#include <iostream>
#include <math.h>
#include <numeric>
#include <stdlib.h>
#include <string>
#include <tuple>
#include <Windows.h>
#include <vector>
#include "FliSdk.h"

#define DELAY 10
#define LONG_DELAY 2000
#define DEVNUM 0
#define NBUFF 256

using namespace std;

extern AT_H Dev;
extern ofstream logfile;

int initDev();
int setTemp();
int setDev();
int setAcq(int AOI, bool GLOBAL, double ExpTime);
int stopDev();
int getDevProps();

int initDev() {
	int Err = 1;
	AT_WC CameraModel[128];
	char Model[128];
	// AT_H Dev = AT_HANDLE_UNINITIALISED;
	AT_64 DeviceCount = 0;

	// Start core library
	logfile << "Error from AT_Initialise : " << AT_InitialiseLibrary() << endl;
	logfile << "Error from AT_InitialiseUtilityLibrary : " << AT_InitialiseUtilityLibrary() << endl;
	logfile << "Error from AT_GetInt('Device Count') : " << AT_GetInt(AT_HANDLE_SYSTEM, L"Device Count", &DeviceCount) << endl;
	logfile << "Found " << DeviceCount << " Devices." << endl;
	cout << "Found " << DeviceCount << " Devices." << endl;

	// Open hardware camera
	cout << "Device " << DEVNUM << " : ";
	logfile << "Opening camera : " << AT_Open(DEVNUM, &Dev) << endl;

	Err = AT_GetString(Dev, L"Camera Model", CameraModel, 128);
	if (Err == AT_SUCCESS) {
		wcstombs(Model, CameraModel, 64);
		cout << Model << endl;
	}
	else {
		cout << "Error getting Camera Model : " << Err << endl;
		logfile << "Error closing camera() : " << AT_Close(Dev) << endl;
	}
	return Dev;
}

int setDev() {
	logfile << "Setting TriggerMode : " << AT_SetEnumeratedString(Dev, L"TriggerMode", L"Internal") << endl;
	Sleep(DELAY);
	logfile << "Setting PixelReadoutRate : " << AT_SetEnumeratedString(Dev, L"PixelReadoutRate", L"280 MHz") << endl;
	Sleep(DELAY);
	logfile << "Setting SimplePreAmpGainControl : " << AT_SetEnumeratedString(Dev, L"SimplePreAmpGainControl", L"16-bit (low noise & high well capacity)") << endl;
	Sleep(DELAY);
	logfile << "Setting PixelEncoding : " << AT_SetEnumeratedString(Dev, L"PixelEncoding", L"Mono16") << endl;
	Sleep(DELAY);
	logfile << "Setting StaticBlemishCorrection : " << AT_SetBool(Dev, L"StaticBlemishCorrection", AT_TRUE) << endl;
	Sleep(DELAY);
	logfile << "Setting SpuriousNoiseFilter : " << AT_SetBool(Dev, L"SpuriousNoiseFilter", AT_TRUE) << endl;
	Sleep(DELAY);
	logfile << "Setting ElectronicShutteringMode : " << AT_SetEnumeratedString(Dev, L"ElectronicShutteringMode", L"Rolling") << endl;
	Sleep(DELAY);
	return 0;
}

int setAcq(int AOI, bool GLOBAL, double ExpTime) {
    // Region of interest
	int TEMP;
	double TEMP2;

	if (AOI==128) {
		logfile << "Setting AOIWidth : " << AT_SetInt(Dev, L"AOIWidth", 128) << endl;
		logfile << "Setting AOILeft : " << AT_SetInt(Dev, L"AOILeft", 1217) << endl;
		logfile << "Setting AOIHeight : " << AT_SetInt(Dev, L"AOIHeight", 128) << endl;
		logfile << "Setting VerticallyCentreAOI : " << AT_SetBool(Dev, L"VerticallyCentreAOI", AT_TRUE) << endl;
	}
	Sleep(DELAY);
	// Exposure
	logfile << "Setting exposure : " << AT_SetFloat(Dev, L"ExposureTime", ExpTime) << endl;
	Sleep(DELAY);
    // Shutter mode
	if (GLOBAL) logfile << "Setting ElectronicShutteringMode : " << AT_SetEnumeratedString(Dev, L"ElectronicShutteringMode", L"Global") << endl;
	else logfile << "Setting ElectronicShutteringMode : " << AT_SetEnumeratedString(Dev, L"ElectronicShutteringMode", L"Rolling") << endl;
	Sleep(DELAY);
	// Acquisition mode
	logfile << "Setting CycleMode Continuous: " << AT_SetEnumeratedString(Dev, L"CycleMode", L"Continuous") << endl;
	Sleep(DELAY);
	return 0;
}

int stopDev() {
	logfile << "Flushing the device : " << AT_Flush(Dev) << endl;
	logfile << "Closing the device : " << AT_Close(Dev) << endl;
	logfile << "Finalising Utility Library : " << AT_FinaliseUtilityLibrary() << endl;
	logfile << "Finalising Core Library : " << AT_FinaliseLibrary() << endl;
	return 0;
}

int setTemp() {
	int Err = 1;
    wchar_t* tempStatus[256];
	int tempStaInd = 0;
	cout << "Setting SensorCooling : " << AT_SetBool(Dev, L"SensorCooling", AT_TRUE) << endl;
	cout << "Setting Temperature Control : " << AT_SetEnumeratedString(Dev, L"TemperatureControl", L"-30.00") << endl;
	cout << "Error getting  temperature status  : " << AT_GetEnumIndex(Dev, L"TemperatureStatus", &tempStaInd) << endl;

    // Wait for the temperature to stabilize
	while (tempStaInd != 1) {
		Err = AT_GetEnumIndex(Dev, L"TemperatureStatus", &tempStaInd);
		if (Err != AT_SUCCESS) { cout << "Error getting  temperature status  : " << Err << endl; }
		if (tempStaInd == 2) { cout << "Temperature status : " << "cooling" << endl; }
		else if (tempStaInd != 1) { cout << "Temperature status : " << tempStaInd << endl; }
		Sleep(LONG_DELAY);
	}
	cout << "Temperature status : " << " cooled to -30 C" << endl;
	return 0;
}

int getDevProps() {
	int Err = 1;
	wchar_t Par[256];
	char Pars[256];
	int iPar = 0;

	Err = AT_GetEnumeratedString(Dev, L"TriggerMode", iPar, Par, 256);
	if (Err != AT_SUCCESS) { cout << "Error getting TriggerMode : " << Err << endl; }
	wcstombs(Pars, Par, 256);
	cout << "TriggerMode: " << Pars << endl;


	Err = AT_GetEnumeratedString(Dev, L"PixelReadoutRate", iPar, Par, 64);
	if (Err != AT_SUCCESS) { cout << "Error getting PixelReadoutRate : " << Err << endl; }
	wcstombs(Pars, Par, 256);
	cout << "PixelReadoutRate: " << Pars << endl;

	Err = AT_GetEnumeratedString(Dev, L"SimplePreAmpGainControl", iPar, Par, 64);
	if (Err != AT_SUCCESS) { cout << "Error getting SimplePreAmpGainControl : " << Err << endl; }
	wcstombs(Pars, Par, 256);
	cout << "SimplePreAmpGainControl: " << Pars << endl;

	Err = AT_GetEnumeratedString(Dev, L"PixelEncoding", iPar, Par, 64);
	if (Err != AT_SUCCESS) { cout << "Error getting PixelEncoding : " << Err << endl; }
	wcstombs(Pars, Par, 256);
	cout << "PixelEncoding: " << Pars << endl;

	Err = AT_GetEnumeratedString(Dev, L"ElectronicShutteringMode", iPar, Par, 64);
	if (Err != AT_SUCCESS) { cout << "Error getting ElectronicShutteringMode : " << Err << endl; }
	wcstombs(Pars, Par, 256);
	cout << "ElectronicShutteringMode: " << Pars << endl;

	return 0;

}
#endif