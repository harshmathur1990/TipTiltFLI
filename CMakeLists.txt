cmake_minimum_required(VERSION 3.14)
project(TipTiltAO)


set(CMAKE_CXX_STANDARD 14)

remove_definitions(-DUNICODE -D_UNICODE)

include_directories($ENV{FLISDK_DIR}/include)

if(ARM_BUILD)
    set(CMAKE_SKIP_RPATH TRUE)
    set(CMAKE_CXX_FLAGS  "-Wl,-rpath,'\$ORIGIN/../../../../lib'")
elseif(LINUX_BUILD)
    set(CMAKE_SKIP_RPATH TRUE)
    set(CMAKE_CXX_FLAGS  "-Wl,-rpath,'\$ORIGIN/../../../../lib/release'")
endif()

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -no-pie")

if(ARM_BUILD)
    link_directories($ENV{FLISDK_DIR}/lib)
else()
    link_directories($ENV{FLISDK_DIR}/lib/release)
endif()

set(INCLUDE_DIR C:/PROGRA~2/NATION~1/Shared/EXTERN~1/C/include)
set(INCLUDE_WINDIR C:/Windows/System32)
set(INCLUDE_lIB_DIR C:/PROGRA~2/NATION~1/Shared/EXTERN~1/C/lib64/msvc)
set(INCLUDE_FFTW_DIR C:/Users/hp/Documents/fftw3)
set(SOURCES main.cpp common.h controls.h firstlight.h imageprocess.h)
add_executable(TipTiltAO ${SOURCES})
target_include_directories(TipTiltAO PRIVATE ${INCLUDE_DIR} ${INCLUDE_WINDIR} ${INCLUDE_lIB_DIR} ${INCLUDE_FFTW_DIR})
target_link_libraries(TipTiltAO $ENV{FLISDK_DIR}/lib/release/FliSdk.lib ${INCLUDE_lIB_DIR}/NIDAQmx.lib ${INCLUDE_FFTW_DIR}/libfftw3-3.lib)

add_executable(CalibrateXY calibration.cpp)
target_include_directories(CalibrateXY PRIVATE ${INCLUDE_DIR} ${INCLUDE_WINDIR} ${INCLUDE_lIB_DIR} ${INCLUDE_FFTW_DIR})
target_link_libraries(CalibrateXY $ENV{FLISDK_DIR}/lib/release/FliSdk.lib ${INCLUDE_lIB_DIR}/NIDAQmx.lib ${INCLUDE_FFTW_DIR}/libfftw3-3.lib)

add_executable(VoltageShiftMatrix make_calibration_matrix.cpp)

add_executable(DarkFlat generate_dark_flat.cpp)
target_include_directories(DarkFlat PRIVATE ${INCLUDE_DIR} ${INCLUDE_WINDIR} ${INCLUDE_lIB_DIR} ${INCLUDE_FFTW_DIR})
target_link_libraries(DarkFlat $ENV{FLISDK_DIR}/lib/release/FliSdk.lib ${INCLUDE_lIB_DIR}/NIDAQmx.lib ${INCLUDE_FFTW_DIR}/libfftw3-3.lib)