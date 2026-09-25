#include "imageprocess.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <string>

#include "utilheaders.h"
#include <opencv2/opencv.hpp>

using namespace std;

double *HammingWindow = nullptr;
double *ReferenceImage = nullptr;
fftw_complex* ReferenceImageFT = nullptr;
DFTI_DESCRIPTOR_HANDLE descHandle = nullptr;
fftw_plan PlanForward = nullptr;
fftw_plan PlanInverse = nullptr;
double **MasterFlat = nullptr;

namespace {
int xflatarr[LENGTHFLATARR] = {-20, -15, -10,  -5,   0,   5,  10,  15,  20,  25,  30,  35,  40,
                               45,  50,  55,  60,  65,  70,  75,  80,  85,  90,  95, 100, 105,
                               110, 115, 120, 125, 130};
int yflatarr[LENGTHFLATARR] = {-20, -15, -10,  -5,   0,   5,  10,  15,  20,  25,  30,  35,  40,
                               45,  50,  55,  60,  65,  70,  75,  80,  85,  90,  95, 100, 105,
                               110, 115, 120, 125, 130};

int getLinearArrayIndice(int i, int j) {
    return j * NX + i;
}
}

int getDarkFlat() {
    string flatFilename = "MeanFlat";
    MasterFlat = (double**)malloc(sizeof(double*) * LENGTHFLATARR * LENGTHFLATARR);
    for (unsigned i = 0; i < LENGTHFLATARR * LENGTHFLATARR; i++) {
        MasterFlat[i] = (double*)malloc(sizeof(double) * NX * NY);
        int flatYIndex = i / LENGTHFLATARR;
        int flatXIndex = i % LENGTHFLATARR;
        string fileName = string("Flats") + "\\" + flatFilename + "_" + to_string(xflatarr[flatXIndex]) + "_" + to_string(yflatarr[flatYIndex]) + ".dat";
        ifstream flatFile(fileName, ios::binary);
        uint16_t pixel = 0;
        for (unsigned int j = 0; j < NX * NY; j++) {
            flatFile.read((char*) &pixel, sizeof(uint16_t));
            MasterFlat[i][j] = 1.0 / pixel;
        }
        flatFile.close();
    }
    return 0;
}

int get_flat_indice(double xShift, double yShift) {
    constexpr double minShift = -20.0;
    constexpr double step = 5.0;

    int flatXIndex = std::clamp(static_cast<int>(lround((xShift - minShift) / step)), 0, static_cast<int>(LENGTHFLATARR) - 1);
    int flatYIndex = std::clamp(static_cast<int>(lround((yShift - minShift) / step)), 0, static_cast<int>(LENGTHFLATARR) - 1);
    return flatYIndex * LENGTHFLATARR + flatXIndex;
}

int getHammingWindow() {
    HammingWindow = (double*) malloc(sizeof(double) * NPIX);
    double *hamming1d = (double*) malloc(sizeof(double) * NX);

    for (int i = 0; i < static_cast<int>(NX); i++) {
        hamming1d[i] = A0 + (A0 - 1) * cos(2 * PI * i / (NX - 1));
    }
    for (int i = HAMMINGWINDOW_CUT; i < static_cast<int>(NX) - HAMMINGWINDOW_CUT; i++) {
        hamming1d[i] = hamming1d[HAMMINGWINDOW_CUT - 1];
    }

    double maxHamm = 1.0 / *max_element(hamming1d, hamming1d + NX);
    for (int i = HAMMINGWINDOW_CUT; i < static_cast<int>(NX) - HAMMINGWINDOW_CUT; i++) {
        hamming1d[i] *= maxHamm;
    }

    for (int i = 0; i < static_cast<int>(NPIX); i++) {
        int xin = i % NX;
        int yin = i / NX;
        HammingWindow[i] = hamming1d[xin] * hamming1d[yin];
    }
    return 0;
}

int getIntelFFTPlans() {
    ReferenceImage = (double*) mkl_malloc(sizeof(double) * NX * NY, 64);
    ReferenceImageFT = (fftw_complex *) mkl_malloc(NPIXFT * sizeof(fftw_complex), 64);

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
    strides[0] = 0;
    strides[1] = 1;
    strides[2] = NX;

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

    MKL_LONG storedStrides[3];
    status = DftiGetValue(descHandle, DFTI_INPUT_STRIDES, storedStrides);
    if (status != 0) {
        cout << "DftiGetValue DFTI_INPUT_STRIDES failed : " << status << endl;
        return -9;
    }
    cout << "DftiGetValue DFTI_INPUT_STRIDES : " << storedStrides[0] << "  " << storedStrides[1] << "  " << storedStrides[2] << endl;

    status = DftiGetValue(descHandle, DFTI_OUTPUT_STRIDES, storedStrides);
    if (status != 0) {
        cout << "DftiGetValue DFTI_OUTPUT_STRIDES failed : " << status << endl;
        return -10;
    }
    cout << "DftiGetValue DFTI_OUTPUT_STRIDES : " << storedStrides[0] << "  " << storedStrides[1] << endl;

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
    ReferenceImageFT = (fftw_complex *) fftw_malloc(NPIXFT * sizeof(fftw_complex));

    double *forwardInput = new double[NX * NY];
    fftw_complex *forwardOutput = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * NPIXFT);
    PlanForward = fftw_plan_dft_r2c_2d(NX, NY, forwardInput, forwardOutput, FFTW_MEASURE);

    fftw_complex *inverseInput = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * NPIXFT);
    double *inverseOutput = new double[NX * NY];
    PlanInverse = fftw_plan_dft_c2r_2d(NX, NY, inverseInput, inverseOutput, FFTW_MEASURE);
    return 0;
}

int initializeFFT() {
    if (MODE == INTEL_FFT) {
        return getIntelFFTPlans();
    }
    return getFFTWPlans();
}

tuple<double,double,double,double> getGradientSurfaceFit(double* const surface) {
    double sumZ = 0;
    double sumZX = 0;
    double sumZY = 0;
    double sumZXY = 0;
    tuple<double,double,double,double> coefficients;
    for (int i = 0; i < static_cast<int>(NPIX); i++) {
        int xIndex = i % NX;
        int yIndex = i / NX;
        sumZ += surface[i];
        sumZX += xIndex * surface[i];
        sumZY += yIndex * surface[i];
        sumZXY += xIndex * yIndex * surface[i];
    }
    get<0>(coefficients) = C00 * sumZ + C01 * sumZX + C02 * sumZY + C03 * sumZXY;
    get<1>(coefficients) = C10 * sumZ + C11 * sumZX + C12 * sumZY + C13 * sumZXY;
    get<2>(coefficients) = C20 * sumZ + C21 * sumZX + C22 * sumZY + C23 * sumZXY;
    get<3>(coefficients) = C30 * sumZ + C31 * sumZX + C32 * sumZY + C33 * sumZXY;
    return coefficients;
}

tuple<double,double> getSubPixelShift(double* const surface, int index) {
    tuple<double, double> shift;
    unsigned int xIndex = index % NX;
    unsigned int yIndex = index / NX;
    double coeff[9];
    int k = 0;
    for (int j = -1; j < 2; j++) {
        for (int i = -1; i < 2; i++) {
            int neighborY = static_cast<int>(yIndex) + j;
            int neighborX = static_cast<int>(xIndex) + i;
            if (neighborX < 0) neighborX += static_cast<int>(NX);
            else if (neighborX >= static_cast<int>(NX)) neighborX -= static_cast<int>(NX);
            if (neighborY < 0) neighborY += static_cast<int>(NY);
            else if (neighborY >= static_cast<int>(NY)) neighborY -= static_cast<int>(NY);
            coeff[k++] = surface[getLinearArrayIndice(neighborX, neighborY)];
        }
    }

    double xShift = xIndex;
    double yShift = yIndex;
    if (xIndex > (NX / 2) - 1) {
        xShift = static_cast<int>(xIndex) - static_cast<int>(NX);
    }
    if (yIndex > (NY / 2) - 1) {
        yShift = static_cast<int>(yIndex) - static_cast<int>(NY);
    }

    double a2 = 0.5 * (coeff[5] - coeff[3]);
    double a3 = 0.5 * (coeff[5] - 2 * coeff[4] + coeff[3]);
    double a4 = 0.5 * (coeff[7] - coeff[1]);
    double a5 = 0.5 * (coeff[7] - 2 * coeff[4] + coeff[1]);
    double a6 = 0.25 * (coeff[8] + coeff[0] - coeff[6] - coeff[2]);
    get<0>(shift) = xShift + (a4 * a6 - 2 * a2 * a5) / (4 * a3 * a5 - pow(a6, 2));
    get<1>(shift) = yShift + (a2 * a6 - 2 * a3 * a4) / (4 * a3 * a5 - pow(a6, 2));
    return shift;
}

int processReferenceImage(uint16_t* const image, bool useCameraFlat, int flatIndice) {
    double imageMean = 0;
    for (int i = 0; i < static_cast<int>(NPIX); i++) {
        ReferenceImage[i] = image[i];
        if (!useCameraFlat) {
            ReferenceImage[i] *= MasterFlat[flatIndice][i];
        }
        imageMean += ReferenceImage[i];
    }
    imageMean = NPIX / imageMean;
    for (int i = 0; i < static_cast<int>(NPIX); i++) {
        ReferenceImage[i] *= imageMean;
    }

    tuple<double, double, double, double> coefficients = getGradientSurfaceFit(ReferenceImage);
    double w0 = get<0>(coefficients);
    double wx = get<1>(coefficients);
    double wy = get<2>(coefficients);
    double wxy = get<3>(coefficients);
    for (int i = 0; i < static_cast<int>(NPIX); i++) {
        int xIndex = i % NX;
        int yIndex = i / NX;
        ReferenceImage[i] -= w0 + wx * xIndex + wy * yIndex + wxy * xIndex * yIndex;
        ReferenceImage[i] *= HammingWindow[i];
    }

    ComputeForward(ReferenceImage, ReferenceImageFT);
    return 0;
}

tuple<double, double> getImageShift(
        uint16_t* const image,
        double *currentImage,
        fftw_complex *currentImageFT,
        fftw_complex *correlatedImageFT,
        double *correlatedImage,
        uint64_t currCount,
        uint64_t *imageSaveCounter,
        int fpsCamera,
        int imageSaveAfterSecond,
        uint64_t* counter,
        bool* updateReference,
        uint32_t autoGuiderCounter,
        bool autoGuiderHappening,
        uint64_t refreshInterval,
        uint64_t* numRefImage,
        bool useCameraFlat,
        int flatIndice,
        int showLive
        ) {
    tuple<double, double> shift;
    tuple<double, double, double, double> coefficients;
    double imageMean = 0;

    for (unsigned int i = 0; i < NPIX; i++) {
        currentImage[i] = image[i];
        if (!useCameraFlat) {
            currentImage[i] *= MasterFlat[flatIndice][i];
        }
        imageMean += currentImage[i];
    }
    imageMean = NPIX / imageMean;
    for (unsigned int i = 0; i < NPIX; i++) {
        currentImage[i] *= imageMean;
    }

    if (showLive && !(currCount & ((1 << 4) - 1))) {
        unique_lock<mutex> displayLock(displayMutex);
        displayReady = true;
        displayLock.unlock();
        displayConditionalVariable.notify_one();
    }

    if (imageSaveAfterSecond > 0) {
        FILE *filePointer = nullptr;
        if (*imageSaveCounter == static_cast<uint64_t>(imageSaveAfterSecond * fpsCamera)) {
            string fileName = string(SAVEPATH) + "//Curr_" + to_string(currCount) + ".dat";
            fopen_s(&filePointer, fileName.c_str(), "wb");
            fwrite(currentImage, sizeof(*currentImage), NX * NY, filePointer);
            fclose(filePointer);
            *imageSaveCounter = 0;
        }
        *imageSaveCounter += 1;
    }

    coefficients = getGradientSurfaceFit(currentImage);
    double w0 = get<0>(coefficients);
    double wx = get<1>(coefficients);
    double wy = get<2>(coefficients);
    double wxy = get<3>(coefficients);
    for (unsigned int i = 0; i < NPIX; i++) {
        unsigned int xIndex = i % NX;
        unsigned int yIndex = i / NX;
        currentImage[i] -= w0 + wx * xIndex + wy * yIndex + wxy * xIndex * yIndex;
        currentImage[i] *= HammingWindow[i];
    }

    ComputeForward(currentImage, currentImageFT);

    for (unsigned int i = 0; i < NPIXFT; i++) {
        correlatedImageFT[i][0] = currentImageFT[i][0] * ReferenceImageFT[i][0] + currentImageFT[i][1] * ReferenceImageFT[i][1];
        correlatedImageFT[i][1] = currentImageFT[i][1] * ReferenceImageFT[i][0] - currentImageFT[i][0] * ReferenceImageFT[i][1];
    }
    ComputeBackward(correlatedImageFT, correlatedImage);

    unsigned maxIndex = distance(correlatedImage, max_element(correlatedImage, correlatedImage + NPIX));
    shift = getSubPixelShift(correlatedImage, maxIndex);

    if (
        (*counter > refreshInterval || *updateReference) &&
        (autoGuiderCounter == 0 && !autoGuiderHappening) &&
        (fabs(get<0>(shift)) <= 0.03) &&
        (fabs(get<1>(shift)) <= 0.03)
    ) {
        ComputeForward(currentImage, ReferenceImageFT);
        *counter = 0;
        *updateReference = false;
        *numRefImage += 1;
    }
    return shift;
}

void bin_separately(uint16_t* const image, uint16_t* const binnedImage) {
    using PixelValueType = std::remove_cvref_t<decltype(*image)>;

    constexpr PixelValueType AllOnes = ~static_cast<PixelValueType>(0);
    constexpr unsigned BitCount = 12;
    constexpr uint64_t PixelInValueMax = static_cast<PixelValueType>(~(AllOnes << BitCount));
    constexpr uint64_t PixelTypeMax = (std::numeric_limits<PixelValueType>::max)();

    {
        static_assert(PixelInValueMax * BINFACTORWIDTH <= PixelTypeMax,
                      "cannot compress horizontally without risking overflow");

        auto out = image;
        for (auto inPos = image, end = image + WIDTH * HEIGHT; inPos != end;) {
            uint_fast16_t sum = 0;
            for (unsigned i = 0; i != BINFACTORWIDTH; ++i) {
                sum += *(inPos++);
            }
            *(out++) = sum;
        }
    }

    {
        uint16_t* inputRows[BINFACTORHEIGHT];
        for (unsigned i = 0; i != BINFACTORHEIGHT; ++i) {
            inputRows[i] = image + (NX * i);
        }

        for (auto out = binnedImage, end = binnedImage + NX * NY; out != end;) {
            for (auto const rowEnd = out + NX; out != rowEnd;) {
                uint_fast32_t sum = 0;

                static_assert(
                    PixelInValueMax * BINFACTORWIDTH * BINFACTORHEIGHT <= (std::numeric_limits<decltype(sum)>::max)(),
                    "type of sum needs replacement, since it cannot hold the result of adding up all source pixels for one target pixel"
                );

                for (unsigned i = 0; i != BINFACTORHEIGHT; ++i) {
                    sum += *(inputRows[i]++);
                }
                *(out++) = sum / DIVIDEFACTOR;
            }

            for (unsigned i = 0; i != BINFACTORHEIGHT; ++i) {
                inputRows[i] += NX * (BINFACTORHEIGHT - 1);
            }
        }
    }
}

void ComputeForward(double* const image, fftw_complex* const imageFT) {
    if (MODE == INTEL_FFT) {
        DftiComputeForward(descHandle, image, imageFT);
        return;
    }
    fftw_execute_dft_r2c(PlanForward, image, imageFT);
}

void ComputeBackward(fftw_complex* const imageFT, double* const image) {
    if (MODE == INTEL_FFT) {
        DftiComputeBackward(descHandle, imageFT, image);
        return;
    }
    fftw_execute_dft_c2r(PlanInverse, imageFT, image);
}
