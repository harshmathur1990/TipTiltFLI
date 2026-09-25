//
// Created by IIAP-IPC on 28/04/2022.
//

#include <iostream>
#include <chrono>
#include <fstream>
#include <string>
#include "imageheaders.h"
#include <cstdlib>
#include <cstdint>
#include <type_traits>
#include <memory>

using namespace std;

inline int bin_image(uint16_t* image, uint16_t *binned_image);
inline int bin_separately(uint16_t* const image, uint16_t* const binned_image);


int main() {
    ifstream myfile;
//    myfile.open("F:\\tiptilt\\20230421_121442\\CurrentImage_3276.dat");
    uint16_t *Image = (uint16_t*)calloc(WIDTH * HEIGHT, sizeof(uint16_t));
    int index = 0;
    for (index=0;index<WIDTH * HEIGHT;index++) {
        Image[index] = 4095;
    }
    int a;
    chrono::high_resolution_clock::time_point t0, t1;
    chrono::duration<double> dt;
    uint16_t *binned_image = (uint16_t*)calloc(NX * NY, sizeof(uint16_t));
    t0 = chrono::high_resolution_clock::now();
    int r = bin_separately(Image, binned_image);
    t1 = chrono::high_resolution_clock::now();
    dt = chrono::duration_cast<chrono::duration<double>>(t1 - t0);
    cout << r<<" Time in seconds: " << dt.count() << endl;
    FILE *fp;
    fopen_s(&fp, "binnedFlat.dat", "wb");
    fwrite(binned_image, sizeof(binned_image), NX * NY, fp);
    fclose(fp);
    cout<<"Press any key to exit"<<endl;
    cin>>a;
    return 0;
}

inline int bin_image(uint16_t* image, uint16_t *binned_image) {
    if (image == NULL) {
        return 1;
    }

    int i, j, k, l;
    for (i = 0; i < NY; i++) {
        for (j = 0; j < NX; j++) {
            binned_image[j * NX + i] = 0;
            for (k = i * BINFACTORHEIGHT; k < i * BINFACTORHEIGHT + BINFACTORHEIGHT; k++) {
                for (l = j * BINFACTORWIDTH; l < j * BINFACTORWIDTH + BINFACTORWIDTH; l++) {
                    binned_image[j * NX + i] += (image[l * HEIGHT + k] / DIVIDEFACTOR) ;
                }
            }
        }
    }
    return 0;
}


inline int bin_separately(uint16_t* const image, uint16_t* const binned_image) {
    using PixelValueType = _Remove_cvref_t<decltype(*image)>;

    constexpr PixelValueType AllOnes = ~static_cast<PixelValueType>(0);
    constexpr unsigned BitCount = 12;
    constexpr uint64_t PixelInValueMax = static_cast<PixelValueType>(~(AllOnes << BitCount));
    constexpr uint64_t PixelTypeMax = (numeric_limits<PixelValueType>::max)();

    {
        static_assert(PixelInValueMax * BINFACTORWIDTH <= PixelTypeMax,
                "Cannot Compress horizontally without risking overflow");

        auto out = image;

        for (auto inPos = image, end=image + WIDTH * HEIGHT; inPos!= end;) {
            uint_fast16_t sum = 0;
            for(unsigned i=0;i != BINFACTORWIDTH; ++i) {
                sum += *(inPos++);
            }
            *(out++) = sum;
        }
    }

    {
        uint16_t* inPoss[BINFACTORHEIGHT];

        for (unsigned i=0; i != BINFACTORHEIGHT; ++i) {
            inPoss[i] = image + NX * i;
        }

        for (auto out= binned_image, end = binned_image + NX * NY; out != end;) {
            for (auto const rowEnd = out + NX; out != rowEnd;) {
                uint_fast16_t sum = 0;

                static_assert(PixelInValueMax * BINFACTORWIDTH * BINFACTORHEIGHT <=(numeric_limits<decltype(sum)>::max)(),
                        "type of sum needs replacement, since it cannot hold the result of adding up all source pixels for one target pixel");

                for (unsigned i = 0; i != BINFACTORHEIGHT; ++i) {
                    sum += *(inPoss[i]++);
                }
                *(out++) = sum / DIVIDEFACTOR;

                for (unsigned i=0; i != BINFACTORHEIGHT; ++i) {
                    inPoss[i] += NX * (BINFACTORHEIGHT - 1);
                }
            }
        }
    }

    return 0;
}