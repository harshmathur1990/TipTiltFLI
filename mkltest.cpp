#include <iostream>
#include <fstream>
#include <mkl.h>
#include <complex>
#include <chrono>
#define NN 256
#define NPIXFFT NN * (1 + (NN / 2))
using namespace std;

typedef struct {
    double re;
    double im;
} mkl_double_complex;


int getFFTWPlans(DFTI_DESCRIPTOR_HANDLE *descHandle);
int printFFT(mkl_double_complex *imageFT);


int main() {

    chrono::high_resolution_clock::time_point t0, t1;
    chrono::duration<double> dt;

    int i, flag=0;
    MKL_LONG status, count;
    DFTI_DESCRIPTOR_HANDLE descHandle;
    getFFTWPlans(&descHandle);

    double *image = (double*)malloc(sizeof(double) * NN * NN);
    double *recoveredImage = (double*)malloc(sizeof(double) * NN * NN);
    mkl_double_complex *imageFT = (mkl_double_complex*) mkl_malloc(
            NPIXFFT * sizeof(mkl_double_complex),64);

//    ifstream currentImage("F:\\tiptilt\\20230421_121442\\CurrentImage_286.dat",  ios::binary);

    double TEMP;

    for (int i=0; i<NN * NN; i++){
//        currentImage.read((char*) &TEMP, sizeof(double));
        image[i] = i * i + i * 2 + 1;
    }

    double x;
    count = 0;
    while (1) {
        if (count == 0) {
            t0 = chrono::high_resolution_clock::now();
        }
        status = DftiComputeForward(descHandle, image, imageFT);
        if (status != 0) {
            cout <<"DftiComputeForward Failed: " << status << endl;
            break;
        }
//        if (flag == 0) {
//            printFFT(imageFT);
//            flag = 1;
//        }
        for (i=0; i<NPIXFFT; i++){
            x = imageFT[i].re;
        }
        status = DftiComputeBackward(descHandle, imageFT, recoveredImage);
        if (status != 0) {
            cout <<"DftiComputeBackward Failed: " << status << endl;
            break;
        }
        count += 1;
        if (count == 3000) {
            t1 = chrono::high_resolution_clock::now();
            dt = chrono::duration_cast<chrono::duration<double>>(t1 - t0);
           cout<< "Time (ms): "<<dt.count() * 1000 / (count - 1) <<"\r";
           cout.flush();
        }
    }
    return 0;
}

int getFFTWPlans(DFTI_DESCRIPTOR_HANDLE *descHandle){

    MKL_LONG lengths[2];
    lengths[0] = NN;
    lengths[1] = NN;
    MKL_LONG status = DftiCreateDescriptor(descHandle, DFTI_DOUBLE, DFTI_REAL, 2, lengths);

    if (status != 0) {
        cout << "DftiCreateDescriptor failed : " << status << endl;
        return -1;
    }
    status = DftiSetValue(*descHandle, DFTI_PLACEMENT, DFTI_NOT_INPLACE);
    if (status != 0) {
        cout << "DftiSetValue DFTI_PLACEMENT failed : " << status << endl;
        return -2;
    }

    status = DftiSetValue(*descHandle, DFTI_THREAD_LIMIT, 1);
    if (status != 0) {
        cout << "DftiSetValue DFTI_THREAD_LIMIT failed : " << status << endl;
        return -3;
    }

    status = DftiSetValue(*descHandle, DFTI_CONJUGATE_EVEN_STORAGE, DFTI_COMPLEX_COMPLEX);
    if (status != 0) {
        cout << "DftiSetValue DFTI_CONJUGATE_EVEN_STORAGE failed : " << status << endl;
        return -4;
    }

    status = DftiSetValue(*descHandle, DFTI_PACKED_FORMAT, DFTI_CCE_FORMAT);
    if (status != 0) {
        cout << "DftiSetValue DFTI_PACKED_FORMAT failed : " << status << endl;
        return -5;
    }

    MKL_LONG strides[2];
    strides[0] = NN;
    strides[1] = 1;

    status = DftiSetValue(*descHandle, DFTI_INPUT_STRIDES, strides);
    if (status != 0) {
        cout << "DftiSetValue DFTI_INPUT_STRIDES failed : " << status << endl;
        return -6;
    }

    status = DftiSetValue(*descHandle, DFTI_OUTPUT_STRIDES, strides);
    if (status != 0) {
        cout << "DftiSetValue DFTI_OUTPUT_STRIDES failed : " << status << endl;
        return -7;
    }

    MKL_LONG format;
    status = DftiGetValue(*descHandle, DFTI_PACKED_FORMAT, &format);
    if (status != 0) {
        cout << "DftiGetValue DFTI_PACKED_FORMAT failed : " << status << endl;
        return -8;
    }
    cout << "DftiGetValue DFTI_PACKED_FORMAT : " << format << endl;

    status = DftiCommitDescriptor(*descHandle);
    if (status != 0) {
        cout << "DftiCommitDescriptor failed : " << status << endl;
        return -9;
    }

    return status;
}

int printFFT(mkl_double_complex *imageFT) {
    int i;

    for (i=0;i<NPIXFFT;i++) {
        cout <<i << "  " << imageFT[i].re << "  "<<imageFT[i].im<<endl;
    }

    return 0;
}