cmake_minimum_required(VERSION 3.14)
project(TipTiltAO)

set(CMAKE_PREFIX_PATH C:/PROGRA~1/opencv/build)
set(CMAKE_BUILD_TYPE RELEASE)
set(CMAKE_CXX_STANDARD 23)

remove_definitions(-DUNICODE -D_UNICODE)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories($ENV{FLISDK_DIR}/include)

if(ARM_BUILD)
    set(CMAKE_SKIP_RPATH TRUE)
    set(CMAKE_CXX_FLAGS  "-Wl,-rpath,'\$ORIGIN/../../../../lib'")
elseif(LINUX_BUILD)
    set(CMAKE_SKIP_RPATH TRUE)
    set(CMAKE_CXX_FLAGS  "-Wl,-rpath,'\$ORIGIN/../../../../lib/release'")
endif()

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -no-pie")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Ofast -march=native")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Ofast -march=native")

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g")

if(ARM_BUILD)

    link_directories($ENV{FLISDK_DIR}/lib)
else()
    if (CMAKE_BUILD_TYPE MATCHES RELEASE)
        link_directories($ENV{FLISDK_DIR}/lib/release)
    else ()
        link_directories($ENV{FLISDK_DIR}/lib/debug)
    endif ()
endif()

set(FFTW C:/Users/IIAP-IPC/Documents/fftw-3)
set(FFTWLINK ${FFTW}/libfftw3l-3.lib ${FFTW}/libfftw3f-3.lib ${FFTW}/libfftw3-3.lib)
set(INCLUDE_DIR C:/PROGRA~2/NATION~1/Shared/EXTERN~1/C/include)
set(INCLUDE_lIB_DIR C:/PROGRA~2/NATION~1/Shared/EXTERN~1/C/lib64/msvc)
set(INTEL_FFTWLINK C:/PROGRA~2/Intel/oneAPI/mkl/2023.1.0/lib/intel64)
set(INTEL_FFTW_INCLUDE C:/PROGRA~2/Intel/oneAPI/mkl/2023.1.0/include)
set(INTEL_FFTWLINKLIB ${INTEL_FFTWLINK}/mkl_intel_lp64.lib ${INTEL_FFTWLINK}/mkl_core.lib ${INTEL_FFTWLINK}/mkl_sequential.lib)

set(SOURCES main.cpp controls.h firstlight.h imageprocess.h utilheaders.h utils.cpp mkl_data_types.h imageheaders.h serialconnection.h serialconnection.cpp motorcontrols.h motorcontrols.cpp Biquad.h Butterworth.h Biquad.cpp Butterworth.cpp)
add_executable(TipTiltAO ${SOURCES})
target_include_directories(TipTiltAO PRIVATE ${INCLUDE_DIR} ${INCLUDE_lIB_DIR} ${INTEL_FFTW_INCLUDE} ${FFTW} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(TipTiltAO $ENV{FLISDK_DIR}/lib/release/FliSdk.lib ${INCLUDE_lIB_DIR}/NIDAQmx.lib ${INTEL_FFTWLINKLIB} ${FFTWLINK} ${OpenCV_LIBS})

add_executable(CalibrateXY calibration.cpp utilheaders.h utils.cpp imageheaders.h serialconnection.h serialconnection.cpp motorcontrols.h motorcontrols.cpp)
target_include_directories(CalibrateXY PRIVATE ${INCLUDE_DIR} ${INCLUDE_lIB_DIR} ${INTEL_FFTW_INCLUDE} ${FFTW} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(CalibrateXY $ENV{FLISDK_DIR}/lib/release/FliSdk.lib ${INCLUDE_lIB_DIR}/NIDAQmx.lib ${INTEL_FFTWLINKLIB} ${FFTWLINK} ${OpenCV_LIBS})

add_executable(VoltageShiftMatrix make_calibration_matrix.cpp)

add_executable(DarkFlat generate_dark_flat.cpp utilheaders.h utils.cpp imageprocess.h)
target_include_directories(DarkFlat PRIVATE ${INCLUDE_DIR} ${INCLUDE_lIB_DIR}  ${INTEL_FFTW_INCLUDE} ${FFTW} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(DarkFlat $ENV{FLISDK_DIR}/lib/release/FliSdk.lib ${INCLUDE_lIB_DIR}/NIDAQmx.lib  ${INTEL_FFTWLINKLIB} ${FFTWLINK} ${OpenCV_LIBS})

add_executable(VoltageTest voltage_test.cpp utilheaders.h utils.cpp)
target_include_directories(VoltageTest PRIVATE ${INCLUDE_DIR} ${INCLUDE_lIB_DIR} )
target_link_libraries(VoltageTest $ENV{FLISDK_DIR}/lib/release/FliSdk.lib ${INCLUDE_lIB_DIR}/NIDAQmx.lib)

add_executable(Test2  test_2.cpp)

add_executable(MKLTest mkltest.cpp)
target_include_directories(MKLTest PRIVATE ${INTEL_FFTW_INCLUDE} ${FFTW})
target_link_libraries(MKLTest ${INTEL_FFTWLINKLIB} ${FFTWLINK})