# Tip Tilt Image Stabilization System at Kodaikanal Tower Tunnel Telescope

# Working Principle
It is a closed loop system. Images are acquired from the camera at
fast frame rates of about 655 FPS. Image shift (in pixel units)
is calculated of every frame with respect to a reference frame.
A feedback or correction is given to an actuator which brings back
the image at the reference position.

# Components Used
1. FirstLightImaging C-Blue One Camera
2. Piezojena Actuator
3. NI USB 6351 DAQ Board

# Algorithm
1. The camera works in a callback mode.
2. Once we recieve a frame, either we periodically change the reference frame.
3. or we use the frame to calculate the image shift.
4. Before we do 2 or 3, we preprocess the frame.
5. The image we recieve from camera is of the size 768 x 768, we bin it to 256 x 256.
6. Bias correction happens in camera itself, we only do flat correction.
7. The camera has feature to perform flat correction also, but
8. our actuator is placed in converging beam. Thus we need separate
9. reference frames per actuator position. But if the system is upgraded to
10. use the actuator in collimated beam, camera flats can be used.
11. We multiply the image with a 2D hamming window, to reduce the intensity at the edges.
12. We store the FFT of the reference image in a global variable.
13. FFT of the current frame is performed and multiplied with the conjugate
14. of the FFT of the reference frame (thus performing convolution).
15. The inverse FFT is performed and the a subpixel interpolation is performed
16. on the resultant correlated image to retrieve the image shifts.
17. The imahe shifts are passed through a PID controller and correction to the
18. actuator is calculated. A notification is sent to another thread which wakes up
19. and applies voltages to the actuator. To make sure the actuator works in nominal
20. voltage ranges, periodically, a the offset voltage from the base voltage of the
21. actuator is offloaded to the telescope control system.
22. A live view is also added (in a separate thread), which displays every 16th frame (40 FPS).

# Requirements
1. FFTW3
2. FLI SDK
3. NI DAQMX
4. CMAKE
5. OPENCV
6. CLION
