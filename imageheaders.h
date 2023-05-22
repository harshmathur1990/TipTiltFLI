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
constexpr unsigned INTEL_FFT = 1;
constexpr unsigned FFTW_FFT = 2;
constexpr unsigned MODE=FFTW_FFT;

constexpr double PI = 3.14159265359;
constexpr double A0 = 0.54348;
//Cij in order for 512, 256, 128
constexpr double C00 = /* 6.06787473e-05  */ 2.41299051e-04;   // 9.53984e-4
constexpr double C01 = /* -1.77943541e-07 */ -1.41662848e-06;  // -1.12233e-5
constexpr double C02 = /* -1.77943541e-07 */ -1.41662848e-06;  // -1.12233e-5
constexpr double C03 = /* 5.21828565e-10  */ 8.31680126e-09;  // 1.32039e-7
constexpr double C10 = /* -1.77943541e-07 */ -1.41662848e-06;  // -1.12233e-5
constexpr double C11 = /* 6.96452214e-10  */ 1.11108116e-08;   // 1.76745e-7
constexpr double C12 = /* 5.21828565e-10  */ 8.31680126e-09;   // 1.32039e-7
constexpr double C13 = /* -2.04238186e-12 */ -6.52298138e-11;  // -2.07936e-9
constexpr double C20 = /* -1.77943541e-07 */ -1.41662848e-06;  // -1.12233e-05
constexpr double C21 = /* 5.21828565e-10  */ 8.31680126e-09;   // 1.32039e-7
constexpr double C22 = /* 6.96452214e-10  */ 1.11108116e-08;   // 1.76745e-7
constexpr double C23 = /* -2.04238186e-12 */ -6.52298138e-11;  // -2.07936e-9
constexpr double C30 = /* 5.21828565e-10  */ 8.31680126e-09;   // 1.32039e-7
constexpr double C31 = /* -2.04238186e-12 */ -6.52298138e-11;  // -2.07936e-9
constexpr double C32 = /* -2.04238186e-12 */ -6.52298138e-11;  // -2.07936e-9
constexpr double C33 = /* 7.99366676e-15  */ 5.11606383e-13;   // 3.27458e-11
constexpr unsigned HAMMINGWINDOW_CUT = NX / 8;
#endif
