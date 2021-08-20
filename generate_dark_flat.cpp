#include <iostream>
#include <cstdlib>
#include <fstream>
#include <string>
#include "firstlight.h"
#define DARK 0
#define FLAT 1

extern FliSdk* fli;
extern uint16_t width, height;

int main() {
    initDev(1);
    int mode = -1, NFRAMES=0, i, j;
    uint16_t *Image;
    uint16_t *MeanImage = (uint16_t*)calloc(width*height, sizeof(uint16_t));
    ofstream ImOut;
    std::string darkFilename = "MeanDark.dat";
    std::string flatFilename = "MeanFlat.dat";

    while(mode != DARK || mode != FLAT) {
        std::cout<<"Enter dark(0) or flat(1)"<<std::endl;
        std::cin>>mode;
    }

    if (mode == DARK) {
        std::cout<<"Please close the shutter of the camera"<<std::endl;
    }
    else {
        std::cout<<"Please move the Image about the disc center"<<std::endl;
    }

    std::cout<<"Please enter the number of frames to be captured"<<std::endl;
    std::cin>>NFRAMES;
    startCamera();

    fli->imageProcessing()->enableAutoClip(true);

    for (i=0;i<NFRAMES;i++) {
        Image = (uint16_t*)fli->getRawImage();
        for (j=0;j < width*height;j++) {
            MeanImage[j] += Image[j];
        }
    }
    stopCamera();
    stopDev();
    for (j=0;j < width*height;j++) {
        MeanImage[j]/= NFRAMES;
    }

    if (mode == DARK) ImOut.open(darkFilename);
    else ImOut.open(flatFilename);
    ImOut.write((char*)MeanImage, 2*width*height);
    ImOut.close();

    if (mode == DARK)
        std::cout<<"Wrote "<<darkFilename<<std::endl;
    else
        std::cout<<"Wrote "<<flatFilename<<std::endl;

    return 0;
}