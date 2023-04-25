#ifndef TIPTILTAO_IMAGEHEADERS_H
#define TIPTILTAO_IMAGEHEADERS_H
constexpr unsigned NX = 256;
constexpr unsigned NY = 256;
constexpr unsigned NPIX = NX * NY;
constexpr unsigned NPIXFT = NX * (1 + (NY / 2));
constexpr unsigned WIDTH = 768;
constexpr unsigned HEIGHT = 768;
constexpr unsigned BINFACTORWIDTH = WIDTH / NX;
constexpr unsigned BINFACTORHEIGHT = HEIGHT / NY;
constexpr unsigned DIVIDEFACTOR = BINFACTORWIDTH * BINFACTORHEIGHT;
constexpr unsigned INTEL_FFT = 0;
constexpr unsigned FFTW_FFT = 1;
int MODE=FFTW_FFT;
#endif //TIPTILTAO_IMAGEHEADERS_H
