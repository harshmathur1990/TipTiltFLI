#ifndef IMAGEPROCESS_H
#define IMAGEPROCESS_H

// Required libraries
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
//#include <bits/stdc++.h>
#include <Windows.h>

// External variables
extern double *MasterDark;
extern double *MasterFlat;
extern double *HammingWindow;
extern double *CurrentImage;
extern double *ReferenceImage;
extern double *CorrelatedImage;
extern fftw_plan PlanForward;
extern fftw_plan PlanInverse;
extern fftw_complex *CurrentImageFT;
extern fftw_complex *ReferenceImageFT;
extern fftw_complex *CorrelatedImageFT;
extern int XIND, YIND, MAXIND;
extern double W0, WX, WY, WXY, ImageMean;
extern tuple<double, double, double, double> COEFF;
extern tuple<double, double> XYIND;
extern tuple<fftw_plan, fftw_plan> FFTWPlans;
extern ofstream logfile;

// Constants
#define PI 3.14159265359
#define NITER 1000
#define NX 512
#define NY 512
#define NPIX NX*NY
#define NPIXFT NX*(1+NY/2)
#define NORMSCALE 1.0/(NPIX*NPIXFT)
#define A0 0.54348
#define C00 9.53984e-4
#define C01 -1.12233e-5
#define C02 -1.12233e-5
#define C03 1.32039e-7
#define C10 -1.12233e-5
#define C11 1.76745e-7 
#define C12 1.32039e-7 
#define C13 -2.07936e-9
#define C20 -1.12233e-05
#define C21 1.32039e-7
#define C22 1.76745e-7
#define C23 -2.07936e-9
#define C30 1.32039e-7
#define C31 -2.07936e-9
#define C32 -2.07936e-9
#define C33 3.27458e-11 

using namespace std;

// Functions
int getHammingWindow();
int getFFTWPlans();
int getDarkFlat();
tuple<double,double> getSubPixelShift(double *s, int ind);
tuple<double,double,double,double> getGradientSurfaceFit(double *s);
tuple<double, double> getImageShift(uint16_t *Image);

int getDarkFlat(){
    logfile << "Reading dark and flats file" <<  endl;
    ifstream DarkFile("MeanDark.dat",  ios::binary);
    ifstream FlatFile("MeanFlat.dat",  ios::binary);
    uint16_t TEMP;
    for (int i=0; i<NPIX; i++){
        DarkFile.read((char*) &TEMP, sizeof(uint16_t));
        MasterDark[i] = TEMP;
        FlatFile.read((char*) &TEMP, sizeof(uint16_t));
        MasterFlat[i] = 1.0/(TEMP-MasterDark[i]);
    }
    DarkFile.close();
    FlatFile.close();
    logfile << "Master dark and master flat are loaded" << endl;
    return 0;
}

int getHammingWindow() {
    double *Hamm1d;
    Hamm1d = (double*) malloc(sizeof(double)*NX);
    // Create 1D Hamming window
    for (int i=0; i<NX; i++){
        Hamm1d[i] = A0 + (A0-1)*cos(2*PI*i/(NX-1));
    }
    // Flatten it from +16 to -16 pixels
    for (int i=16; i<NX-16; i++){
        Hamm1d[i] = Hamm1d[15];
    }
    // Normalize the whole window
    double maxHamm = 1.0/ *max_element(Hamm1d, Hamm1d+NX);
    for (int i=16; i<NX-16; i++){
        Hamm1d[i] *= maxHamm;
    }
    // Convert to 2D mask
    for (int i=0; i<NPIX; i++){
        int xin = i%NX;
        int yin = i/NX;
        HammingWindow[i] = Hamm1d[xin]*Hamm1d[yin];
    }
    return 0;
}

int getFFTWPlans(){

    // Forward plan
    double *forIN;
    forIN = new double[NPIX];
    fftw_complex *forOUT;
    forOUT = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * NPIXFT);
    PlanForward = fftw_plan_dft_r2c_2d(NX, NY, forIN, forOUT, FFTW_MEASURE);

    // Inverse plan
    fftw_complex *invIN;
    invIN = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * NPIXFT);
    double *invOUT;
    invOUT = new double[NPIX];
    PlanInverse = fftw_plan_dft_c2r_2d(NX, NY, invIN, invOUT, FFTW_MEASURE);
    return 0;
}

tuple<double,double,double,double> getGradientSurfaceFit(double *s) {
    double Sz=0, Szx=0, Szy=0, Szxy=0;
    tuple<double,double,double,double> COEFF;
    int i=0, XIND=0, YIND=0;
    for (i=0; i<NPIX; i++){
        XIND = i%NX;
        YIND = i/NX;
        Sz += s[i];
        Szx += XIND*s[i];
        Szy += YIND*s[i];
        Szxy += XIND*YIND*s[i];
    }
    get<0>(COEFF) = C00*Sz + C01*Szx + C02*Szy + C03*Szxy;
    get<1>(COEFF) = C10*Sz + C11*Szx + C12*Szy + C13*Szxy;
    get<2>(COEFF) = C20*Sz + C21*Szx + C22*Szy + C23*Szxy;
    get<3>(COEFF) = C30*Sz + C31*Szx + C32*Szy + C33*Szxy;
   return COEFF; 
}

tuple<double,double> getSubPixelShift(double *s, int ind) {
    int XIND = ind%NY, YIND = ind/NX;
    double nn = 0, nz = 0, np = 0, zn = 0, zz = 0, zp = 0, pn = 0, pz = 0, pp = 0;
    double a2 = 0, a3 = 0, a4 = 0, a5 = 0, a6 = 0;
    // cout << XIND << " " << YIND << endl;
    if (YIND == 0){
        if (XIND == 0){
            nn = s[16383];
            nz = s[16256];
            np = s[16257];
            zn = s[127];
            zz = s[0];
            zp = s[1];
            pn = s[255];
            pz = s[128];
            pp = s[129];
        }
        else if (XIND == 127) {
            nn = s[16382];
            nz = s[16383];
            np = s[16256];
            zn = s[126];
            zz = s[127];
            zp = s[128];
            pn = s[254];
            pz = s[255];
            pp = s[256];
        }
        else{
            nn = s[ind+16255];
            nz = s[ind+16256];
            np = s[ind+16257];
            zn = s[ind-1];
            zz = s[ind];
            zp = s[ind+1];
            pn = s[ind+127];
            pz = s[ind+128];
            pp = s[ind+129];
        }
    }
    else if (YIND == 127){
        if (XIND == 0){
            nn = s[16255];
            nz = s[16128];
            np = s[16129];
            zn = s[16383];
            zz = s[16256];
            zp = s[16257];
            pn = s[127];
            pz = s[0];
            pp = s[1];
        }
        else if (XIND == 127){
            nn = s[16254];
            nz = s[16255];
            np = s[16128];
            zn = s[16382];
            zz = s[16383];
            zp = s[0];
            pn = s[126];
            pz = s[127];
            pp = s[16256];
        }
        else{
            nn = s[ind-129];
            nz = s[ind-128];
            np = s[ind-127];
            zn = s[ind-1];
            zz = s[ind];
            zp = s[ind+1];
            pn = s[ind-16257];
            pz = s[ind-16256];
            pp = s[ind-16255];
        }
    }
    else{
        nn = s[ind-129];
        nz = s[ind-128];
        np = s[ind-127];
        zn = s[ind-1];
        zz = s[ind];
        zp = s[ind+1];
        pn = s[ind+127];
        pz = s[ind+128];
        pp = s[ind+129];
    }
    double xshf = XIND, yshf = YIND;
    if (XIND>63) { xshf = XIND - 128; }
    if (YIND>63) { yshf = YIND - 128; }
    // cout << xshf << " " << yshf << endl;
    a2 = 0.5 * (zp - zn);
    a3 = 0.5 * (zp - 2*zz + zn);
    a4 = 0.5 * (pz - nz);
    a5 = 0.5 * (pz - 2*zz + nz);
    a6 = 0.25 * (pp + nn - pn - np);
    // cout<< zz << " " << nn << " " << pp << endl;
    // cout<< a2 << " " << a3 << " " << a4 << " " << a5 << " " << a6 << endl;
    get<0>(XYIND) = xshf + (a4*a6-2*a2*a5)/(4*a3*a5-pow(a6,2));
    get<1>(XYIND) = yshf + (a2*a6-2*a3*a4)/(4*a3*a5-pow(a6,2));

    return XYIND;
}

int processReferenceImage(uint16_t *Image){
    int i=0;
    ImageMean=0;
    // Dark, flat and normalize
    for (i=0; i<NPIX; i++){ 
            // ReferenceImage[i] = (Image[i]-MasterDark[i])*MasterFlat[i];
            ReferenceImage[i] = (Image[i]-80.00)*1.00;
            ImageMean += ReferenceImage[i];
    }
    ImageMean = NPIX/ImageMean;
    for (i=0; i<NPIX; i++) ReferenceImage[i] *= ImageMean; 
    // Windowing and gradient correction
    COEFF = getGradientSurfaceFit(ReferenceImage);
    W0 = get<0>(COEFF);
    WX = get<1>(COEFF);
    WY = get<2>(COEFF);
    WXY = get<3>(COEFF);
    for (i=0; i<NPIX; i++){ 
        XIND = i%NX;
        YIND = i/NX;
        ReferenceImage[i] -= W0 + WX*XIND + WY*YIND + WXY*XIND*YIND;
        ReferenceImage[i] *= HammingWindow[i];
    }
    fftw_execute_dft_r2c( PlanForward, ReferenceImage, ReferenceImageFT); // Forward FT of reference image
    return 0;
}

tuple<double, double> getImageShift(uint16_t *Image) {
    int i=0;
    ImageMean = 0;
    // Reduction
    for (i=0; i<NPIX; i++){ // Dark & flat
        // CurrentImage[i] = (Image[i]-MasterDark[i])*MasterFlat[i];
        CurrentImage[i] = (Image[i]-80.00)*1.00;
        ImageMean += CurrentImage[i];
    }
    ImageMean = NPIX/ImageMean;
    for (i=0; i<NPIX; i++){ // Normalize the image
        CurrentImage[i] *= ImageMean;
    }
    // Gradient removal and windowing
    COEFF = getGradientSurfaceFit(CurrentImage);
    W0 = get<0>(COEFF);
    WX = get<1>(COEFF);
    WY = get<2>(COEFF);
    WXY = get<3>(COEFF);
    for (i=0; i<NPIX; i++){ // Surface fit & window
        XIND = i%NX;
        YIND = i/NX;
        CurrentImage[i] -= W0 + WX*XIND + WY*YIND + WXY*XIND*YIND;
        CurrentImage[i] *= HammingWindow[i];
    }

    // Correlation
    fftw_execute_dft_r2c( PlanForward, CurrentImage, CurrentImageFT); // Current image FT
    for (i=0; i<NPIXFT; i++){
        CorrelatedImageFT[i][0] = CurrentImageFT[i][0]*ReferenceImageFT[i][0] + CurrentImageFT[i][1]*ReferenceImageFT[i][1];
        CorrelatedImageFT[i][1] = CurrentImageFT[i][1]*ReferenceImageFT[i][0] - CurrentImageFT[i][0]*ReferenceImageFT[i][1];
    }
    fftw_execute_dft_c2r( PlanInverse, CorrelatedImageFT, CorrelatedImage);

    // Image shift
    MAXIND = distance(CorrelatedImage, max_element(CorrelatedImage, CorrelatedImage+NPIX));
    XYIND = getSubPixelShift(CorrelatedImage, MAXIND); // Sub-pixel interpolation
    return XYIND;
}

#endif