#ifndef IMAGEPROCESS_H
#define IMAGEPROCESS_H

// Required libraries
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <math.h>
#include <numeric>
#include <fftw3.h>
#include <stdlib.h>
#include <string>
#include <tuple>
#include <mkl.h>
#include <mkl_dfti.h>
//#include <bits/stdc++.h>
#include <Windows.h>
#include "utilheaders.h"
#include "mkl_data_types.h"
#include "imageheaders.h"
#include <type_traits>
#include <memory>

// Constants
#define PI 3.14159265359
#define A0 0.54348
//Cij in order for 512, 256, 128
#define C00  /* 6.06787473e-05  */ 2.41299051e-04   // 9.53984e-4
#define C01  /* -1.77943541e-07 */ -1.41662848e-06  // -1.12233e-5
#define C02  /* -1.77943541e-07 */ -1.41662848e-06  // -1.12233e-5
#define C03  /* 5.21828565e-10  */ 8.31680126e-09   // 1.32039e-7
#define C10  /* -1.77943541e-07 */ -1.41662848e-06  // -1.12233e-5
#define C11  /* 6.96452214e-10  */ 1.11108116e-08   // 1.76745e-7
#define C12  /* 5.21828565e-10  */ 8.31680126e-09   // 1.32039e-7
#define C13  /* -2.04238186e-12 */ -6.52298138e-11  // -2.07936e-9
#define C20  /* -1.77943541e-07 */ -1.41662848e-06  // -1.12233e-05
#define C21  /* 5.21828565e-10  */ 8.31680126e-09   // 1.32039e-7
#define C22  /* 6.96452214e-10  */ 1.11108116e-08   // 1.76745e-7
#define C23  /* -2.04238186e-12 */ -6.52298138e-11  // -2.07936e-9
#define C30  /* 5.21828565e-10  */ 8.31680126e-09   // 1.32039e-7
#define C31  /* -2.04238186e-12 */ -6.52298138e-11  // -2.07936e-9
#define C32  /* -2.04238186e-12 */ -6.52298138e-11  // -2.07936e-9
#define C33  /* 7.99366676e-15  */ 5.11606383e-13   // 3.27458e-11
using namespace std;

// Functions

int getHammingWindow();
int getIntelFFTPlans();
int getFFTWPlans();

inline tuple<double,double> getSubPixelShift(double *s, int ind);
inline tuple<double,double,double,double> getGradientSurfaceFit(double *s);
inline tuple<double, double> getImageShift(
        uint16_t *Image, int this_indice, int fpsCamera,
        double *CurrentImage, fftw_complex *CurrentImageFT,
        fftw_complex *CorrelatedImageFT, double *CorrelatedImage, uint64_t curr_count);
inline void bin_separately(uint16_t* image, uint16_t* binned_image);
void ComputeForward(double* image, fftw_complex* imageFT);
void ComputeBackward(fftw_complex* imageFT, double* image);
int initializeFFT();

double *HammingWindow;
double *ReferenceImage;
fftw_complex* ReferenceImageFT;
DFTI_DESCRIPTOR_HANDLE descHandle;
fftw_plan PlanForward;
fftw_plan PlanInverse;

int getHammingWindow() {
    HammingWindow = (double*) malloc(sizeof(double)*NPIX);
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

int getIntelFFTPlans(){

    ReferenceImage = (double*) mkl_malloc(sizeof(double) * NX * NY, 64);
    ReferenceImageFT = (fftw_complex *) mkl_malloc(
            NPIXFT * sizeof(fftw_complex),64);

    MKL_LONG lengths[2];
    lengths[0] = NX;
    lengths[1] = NY;
    MKL_LONG status = DftiCreateDescriptor(&descHandle, DFTI_DOUBLE, DFTI_REAL, 2, lengths);

    if (status != 0) {
        cout << "DftiCreateDescriptor failed : " << status << endl;
        return -1;
    }
    status = DftiSetValue(descHandle, DFTI_PLACEMENT, DFTI_NOT_INPLACE);
    if (status != 0) {
        cout << "DftiSetValue DFTI_PLACEMENT failed : " << status << endl;
        return -2;
    }

    status = DftiSetValue(descHandle, DFTI_CONJUGATE_EVEN_STORAGE, DFTI_COMPLEX_COMPLEX);
    if (status != 0) {
        cout << "DftiSetValue DFTI_CONJUGATE_EVEN_STORAGE failed : " << status << endl;
        return -4;
    }

    status = DftiSetValue(descHandle, DFTI_PACKED_FORMAT, DFTI_CCE_FORMAT);
    if (status != 0) {
        cout << "DftiSetValue DFTI_PACKED_FORMAT failed : " << status << endl;
        return -5;
    }

    MKL_LONG strides[3];
    strides[0] = 0; // first stride is displacement from the first element (0 beause we are doing only one transform)
    strides[1] = 1; // distance between consecutive elements in the first dimension
    strides[2] = NX; // distance between consecutive elements in the secone dimension

    status = DftiSetValue(descHandle, DFTI_INPUT_STRIDES, strides);
    if (status != 0) {
        cout << "DftiSetValue DFTI_INPUT_STRIDES failed : " << status << endl;
        return -6;
    }

    status = DftiSetValue(descHandle, DFTI_OUTPUT_STRIDES, strides);
    if (status != 0) {
        cout << "DftiSetValue DFTI_OUTPUT_STRIDES failed : " << status << endl;
        return -7;
    }

    MKL_LONG format;
    status = DftiGetValue(descHandle, DFTI_PACKED_FORMAT, &format);
    if (status != 0) {
        cout << "DftiGetValue DFTI_PACKED_FORMAT failed : " << status << endl;
        return -8;
    }
    cout << "DftiGetValue DFTI_PACKED_FORMAT : " << format << endl;

    MKL_LONG s_strides[3];
    status = DftiGetValue(descHandle, DFTI_INPUT_STRIDES, s_strides);
    if (status != 0) {
        cout << "DftiGetValue DFTI_INPUT_STRIDES failed : " << status << endl;
        return -9;
    }
    cout << "DftiGetValue DFTI_INPUT_STRIDES : " << s_strides[0] <<"  "<< s_strides[1] << "  "<< s_strides[2] <<endl;

    status = DftiGetValue(descHandle, DFTI_OUTPUT_STRIDES, s_strides);
    if (status != 0) {
        cout << "DftiGetValue DFTI_OUTPUT_STRIDES failed : " << status << endl;
        return -10;
    }
    cout << "DftiGetValue DFTI_OUTPUT_STRIDES : " << s_strides[0] <<"  "<< s_strides[1] << endl;

    status = DftiGetValue(descHandle, DFTI_CONJUGATE_EVEN_STORAGE, &format);
    if (status != 0) {
        cout << "DftiGetValue DFTI_CONJUGATE_EVEN_STORAGE failed : " << status << endl;
        return -11;
    }
    cout << "DftiGetValue DFTI_CONJUGATE_EVEN_STORAGE : " << format << endl;

    status = DftiCommitDescriptor(descHandle);
    if (status != 0) {
        cout << "DftiCommitDescriptor failed : " << status << endl;
        return -12;
    }

    return status;

}

int getFFTWPlans() {
    ReferenceImage = (double*) fftw_malloc(sizeof(double) * NX * NY);
    ReferenceImageFT = (fftw_complex *) fftw_malloc(
            NPIXFT * sizeof(fftw_complex));

    double *forIN;
    forIN = new double[NX * NY];
    fftw_complex *forOUT;
    forOUT = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * NPIXFT);
    PlanForward = fftw_plan_dft_r2c_2d(NX, NY, forIN, forOUT, FFTW_MEASURE);

    // Inverse plan
    fftw_complex *invIN;
    invIN = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * NPIXFT);
    double *invOUT;
    invOUT = new double[NPIXFT];
    PlanInverse = fftw_plan_dft_c2r_2d(NX, NY, invIN, invOUT, FFTW_MEASURE);
    return 0;
}

int initializeFFT() {
    if (MODE == INTEL_FFT) {
        return getIntelFFTPlans();
    }
    return getFFTWPlans();
}

inline tuple<double,double,double,double> getGradientSurfaceFit(double *s) {
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

inline int getLinearArrayIndice(int i, int j) {
    return j * NX + i;
}

inline tuple<double,double> getSubPixelShift(double *s, int ind) {
    std::tuple<double, double> XYIND;
    int XIND = ind%NX, YIND = ind/NX;
    double nn = 0, nz = 0, np = 0, zn = 0, zz = 0, zp = 0, pn = 0, pz = 0, pp = 0;
    double a2 = 0, a3 = 0, a4 = 0, a5 = 0, a6 = 0;
    int i, j, nei_x, nei_y;
    double coeff[9];
    int k = 0;
    for (j=-1;j<2;j++) {
        for (i=-1;i<2;i++) {
            nei_y = YIND + j;
            nei_x = XIND + i;
            if (nei_x < 0) nei_x += NX;
            else if (nei_x >= NX) nei_x -= NX;
            if (nei_y < 0) nei_y += NY;
            else if (nei_y >= NY) nei_y -= NY;
            coeff[k] = s[getLinearArrayIndice(nei_x, nei_y)];
            k += 1;
        }
    }
    nn = coeff[0];
    nz = coeff[1];
    np = coeff[2];
    zn = coeff[3];
    zz = coeff[4];
    zp = coeff[5];
    pn = coeff[6];
    pz = coeff[7];
    pp = coeff[8];
    double xshf = XIND, yshf = YIND;

    // We are doing this because 0, 0 is in the corner,
    // not in the center, one can do fftshift and
    // then subtract center coordinates from this,
    // but doing this way is also same
    if (XIND>(NX/2) - 1) { xshf = XIND - NX; }
    if (YIND>(NY/2) -  1) { yshf = YIND - NY; }

    a2 = 0.5 * (zp - zn);
    a3 = 0.5 * (zp - 2*zz + zn);
    a4 = 0.5 * (pz - nz);
    a5 = 0.5 * (pz - 2*zz + nz);
    a6 = 0.25 * (pp + nn - pn - np);
    get<0>(XYIND) = xshf + (a4*a6-2*a2*a5)/(4*a3*a5-pow(a6,2));
    get<1>(XYIND) = yshf + (a2*a6-2*a3*a4)/(4*a3*a5-pow(a6,2));
    return XYIND;
}

inline int processReferenceImage(uint16_t *Image, int indice){
    int XIND, YIND;
    int i=0;
    double ImageMean=0;
    std::tuple<double, double, double, double> COEFF;
    // Dark, flat and normalize
    for (i=0; i<NPIX; i++){ 
             ReferenceImage[i] = Image[i];
            ImageMean += ReferenceImage[i];
    }
    ImageMean = NPIX/ImageMean;
    for (i=0; i<NPIX; i++) ReferenceImage[i] *= ImageMean;
    // Windowing and gradient correction
    COEFF = getGradientSurfaceFit(ReferenceImage);
    double W0 = get<0>(COEFF);
    double WX = get<1>(COEFF);
    double WY = get<2>(COEFF);
    double WXY = get<3>(COEFF);
    for (i=0; i<NPIX; i++){
        XIND = i%NX;
        YIND = i/NX;
        ReferenceImage[i] -= W0 + WX*XIND + WY*YIND + WXY*XIND*YIND;
        ReferenceImage[i] *= HammingWindow[i];
    }

    ComputeForward(ReferenceImage, ReferenceImageFT);
    return 0;
}

inline tuple<double, double> getImageShift(
        uint16_t *Image, int this_indice, int fpsCamera, double *CurrentImage,
        fftw_complex *CurrentImageFT,
        fftw_complex *CorrelatedImageFT, double *CorrelatedImage, uint64_t curr_count
        ) {

    std::tuple<double, double> XYIND;
    std::tuple<double, double, double, double> COEFF;
    int i=0, XIND, YIND, MAXIND;
    double ImageMean = 0;
    string fname;
    // Reduction
    for (i=0; i<NPIX; i++){
         CurrentImage[i] = Image[i];
        ImageMean += CurrentImage[i];
    }
    ImageMean = NPIX/ImageMean;
    for (i=0; i<NPIX; i++){
        CurrentImage[i] *= ImageMean;
    }
    // Gradient removal and windowing

    COEFF = getGradientSurfaceFit(CurrentImage);
    double W0 = get<0>(COEFF);
    double WX = get<1>(COEFF);
    double WY = get<2>(COEFF);
    double WXY = get<3>(COEFF);
    for (i=0; i<NPIX; i++){
        XIND = i%NX;
        YIND = i/NX;
        CurrentImage[i] -= W0 + WX*XIND + WY*YIND + WXY*XIND*YIND;
        CurrentImage[i] *= HammingWindow[i];
    }

//    fname = string(SAVEPATH) + "//Curr_" +  to_string(curr_count) + ".dat";

//    FILE *fp;
//    fopen_s(&fp, fname.c_str(), "wb");
//    fwrite(CurrentImage, sizeof(CurrentImage), NX * NY, fp);
//    fclose(fp);

    ComputeForward(CurrentImage, CurrentImageFT);

    for (i=0; i<NPIXFT; i++){
        CorrelatedImageFT[i][0] = CurrentImageFT[i][0]*ReferenceImageFT[i][0] + CurrentImageFT[i][1]*ReferenceImageFT[i][1];
        CorrelatedImageFT[i][1] = CurrentImageFT[i][1]*ReferenceImageFT[i][0] - CurrentImageFT[i][0]*ReferenceImageFT[i][1];
    }
    ComputeBackward(CorrelatedImageFT, CorrelatedImage);

    MAXIND = distance(CorrelatedImage, max_element(CorrelatedImage, CorrelatedImage+NPIX));

    XYIND = getSubPixelShift(CorrelatedImage, MAXIND); // Sub-pixel interpolation

    return XYIND;
}

inline void bin_separately(uint16_t* const image, uint16_t* const binnedImage)
{
    // just some stuff for static_assert
    using PixelValueType = std::remove_cvref_t<decltype(*image)>;

    constexpr PixelValueType AllOnes = ~static_cast<PixelValueType>(0);
    constexpr unsigned BitCount = 12;
    constexpr uint64_t PixelInValueMax = static_cast<PixelValueType>(~(AllOnes << BitCount)); // use 64 bit to prevent overflow issues
    constexpr uint64_t PixelTypeMax = (std::numeric_limits<PixelValueType>::max)();
    // end static_assert stuff


    {
        // compress horizontally
        static_assert(PixelInValueMax * BINFACTORWIDTH <= PixelTypeMax,
                      "cannot compress horizontally without risking overflow");

        auto out = image;
        for (auto inPos = image, end = image + WIDTH * HEIGHT; inPos != end;)
        {
            uint_fast16_t sum = 0;
            for (unsigned i = 0; i != BINFACTORWIDTH; ++i)
            {
                sum += *(inPos++);
            }
            *(out++) = sum;
        }
    }

    {
        // compress vertically, divide and write to out

        //read pointers
        uint16_t* inPoss[BINFACTORHEIGHT];
        for (unsigned i = 0; i != BINFACTORHEIGHT; ++i)
        {
            inPoss[i] = image + (NX * i);
        }

        for (auto out = binnedImage, end = binnedImage + NX * NY; out != end;) // for all output rows
        {
            for (auto const rowEnd = out + NX; out != rowEnd;)
            {
                uint_fast32_t sum = 0;

                static_assert(PixelInValueMax * BINFACTORWIDTH * BINFACTORHEIGHT <= (std::numeric_limits<decltype(sum)>::max)(),
                              "type of sum needs replacement, since it cannot hold the result of adding up all source pixels for one target pixel");

                for (unsigned i = 0; i != BINFACTORHEIGHT; ++i)
                {
                    sum += *(inPoss[i]++);
                }
                *(out++) = sum / DIVIDEFACTOR;
            }

            // we advanced each pointer by one row -> advance by (BINNINGFACTORHEIGHT - 1) more
            for (unsigned i = 0; i != BINFACTORHEIGHT; ++i)
            {
                inPoss[i] += NX * (BINFACTORHEIGHT - 1);
            }
        }
    }

}

void ComputeForward(double* image, fftw_complex* imageFT) {
    if (MODE == INTEL_FFT) {
        DftiComputeForward(descHandle, image, imageFT);
        return;
    }
    fftw_execute_dft_r2c( PlanForward, image, imageFT);
}

void ComputeBackward(fftw_complex* imageFT, double* image){
    if (MODE == INTEL_FFT) {
        DftiComputeBackward(descHandle, imageFT, image);
        return;
    }
    fftw_execute_dft_c2r( PlanInverse, imageFT, image);
}
#endif