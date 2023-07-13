# Scripts for running VINS-GPU/ROS with ArduPilot
This repository contains a ROS workspace for using stereo-visual navigation with ROS and ArduPilot.

The end result is the ability to use non-GPS navigation with ArduPilot, using a stereo camera.

# Components

Hardware:
- Stereo Camera. Global shutter preferred (such as https://www.arducam.com/product/arducam-1mp2-wide-angle-stereo-camera-for-raspberry-pi-jetson-nano-and-xavier-nx-dual-ov9281-monochrome-global-shutter-camera-module/)
- Nvidia Jetson Nano
- ArduPilot flight controller (connected to UART on Jetson 40-pin header)

Software:
- ROS Melodic
- VINS-GPU Lite
- MAVROS

# How it works

The VINS_Lite_GPU (https://github.com/KopiSoftware/VINS_Lite_GPU) algorithm is used as a ROS module to take in stereo camera images and generate a pose in 3D space. This pose is sent to ArduPilot via MAVROS and the VISION_POSITION_ESTIMATE message. ArduPilot then uses these messages as a non-GPS navigation source.

Note the VINS_Lite_GPU used here is slightly modified from source - adding a odometry message compatible with MAVROS.

It is a "lighter" version of VINS-GPU (https://github.com/pjrambo/VINS-Fusion-gpu) which allows for better realtime performance on a Jetson Nano.

# Installation

Follow the instructions at https://www.arducam.com/doc-old/camera-for-jetson-nano/multiple-cameras-on-the-jetson/how-to-use-stereo-camera-to-perform-location-with-visual-slam/#ftoc-heading-20, noting the following changes:

- Use `git clone https://github.com/stephendade/Camarray_HAT.git` repo in place of ``git clone -b ov9281_stereo https://github.com/ArduCAM/Camarray_HAT.git``
- Use `git clone https://github.com/stephendade/Nvidia_Jetson_ROS_SLAM_VINS.git` repo in place of ``git clone https://github.com/ArduCAM/Nvidia_Jetson_ROS_SLAM_VINS.git``
- Use this repository as your ROS workspace
- Install MAVROS via ``sudo apt install ros-melodic-mavros ros-melodic-mavros-extras``
- Use ``systemctl stop nvgetty && systemctl disable nvgetty`` on the Jetson and reboot to disable the system console on the UART.

# Configure
- If not using the ArduCAM OV9281 stereo camera, edit ``jetson.launch`` to suit your stereo camera setup
- Edit ``jetson.launch`` to point the to appropriate UART or UDP port for MAVROS.

Ardupilot will need the following parameters:

```
SCHED_LOOP_RATE  250 
VISO_DELAY_MS    150
VISO_ORIENT      0
VISO_POS_M_NSE   0.100000
VISO_POS_X       0.000000
VISO_POS_Y       0.000000
VISO_POS_Z       0.000000
VISO_SCALE       1.000000
VISO_TYPE        2
VISO_VEL_M_NSE   0.010000
VISO_YAW_M_NSE   0.100000
```

# Running
Use ``launch.sh`` to run the ROS instance. If using a remote ROS node (for viewing, debugging, etc) edit the IP addresses in ``launch.sh`` as required.

When connecting a ground station, ensure the streamrate is 20Hz. This is required for the VINS inertial message (/mavros/imu/data) input.


# Lince notes

Following this:

https://www.arducam.com/doc-old/camera-for-jetson-nano/multiple-cameras-on-the-jetson/how-to-use-stereo-camera-to-perform-location-with-visual-slam/#ftoc-heading-20

3.5.1 Install Eigen3.3.90
    git clone https://github.com/eigenteam/eigen-git-mirror
    cd eigen-git-mirror
    mkdir build && cd build
    cmake ..
    make -j4
    sudo make install

3.5.2 Install Ceres2.0.0
    You can refer to here (http://ceres-solver.org/installation.html) to learn more!

        - NOTE, we needed cmake 3.11 minimum on this step, so we need to install that first in case our jetson installation doesn't have a new enough cmake:
            sudo apt remove cmake
            wget https://cmake.org/files/v3.11/cmake-3.11.4.tar.gz
            tar xf cmake-3.11.4.tar.gz
            cd cmake-3.11.4
            ./configure
            sudo make install
            cmake --version

    sudo apt-get install liblapack-dev libsuitesparse-dev libcxsparse3 libgflags-dev libgoogle-glog-dev libgtest-dev
    git clone https://ceres-solver.googlesource.com/ceres-solver
    cd ceres-solver
    mkdir build && cd build
        - NOTE, here we might want do to checkout in 2.0.0
    cmake ..
        - NOTE, on this step it could not find cuda compiler. It seems it was needed to be added to .bashrc or .zshrc. So, add this to .zshrc:
        
            # cuda 10.2
            export CUDA_HOME=/usr/local/cuda
            export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64:/usr/local/cuda/extras/CUPTI/lib64
            export PATH=$PATH:$CUDA_HOME/bin

        and it should work.
        
    sudo make install

3.5.3 Install Opencv3.4.14
    Install OpenCV related dependence
    
    sudo apt-get install build-essential pkg-config
    sudo apt-get install cmake libavcodec-dev libavformat-dev libavutil-dev libglew-dev libgtk2.0-dev libgtk-3-dev libjpeg-dev libpng-dev libpostproc-dev libswscale-dev libtbb-dev libtiff5-dev libv4l-dev libxvidcore-dev libx264-dev qt5-default zlib1g-dev libgl1 libglvnd-dev pkg-config libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev mesa-utils
    
    sudo apt-get install python2.7-dev python3-dev python-numpy python3-numpy

Resolve openGL conflicts
    cd /usr/lib/aarch64-linux-gnu/
    sudo ln -sf libGL.so.1.0.0 libGL.so
    sudo vim /usr/local/cuda/include/cuda_gl_interop.h
        # Comment (line #62~68) of cuda_gl_interop.h 
            //#if defined(__arm__) || defined(__aarch64__)
            //#ifndef GL_VERSION
            //#error Please include the appropriate gl headers before including cuda_gl_interop.h
            //#endif
            //#else
            #include <GL/gl.h>
            //#endif

View the compute capability of your Jetson used
    cd /usr/local/cuda/samples/1_Utilities/deviceQuery
    sudo make
    sudo ./deviceQuery
        The following message appears after execution, the Cuda version installed by Jetson NX is 10.2, and the compute capability version is 7.2, it seems on nano it is like:

            Device 0: "NVIDIA Tegra X1"                                                         
            CUDA Driver Version / Runtime Version          10.2 / 10.2
            CUDA Capability Major/Minor version number:    5.3
            Total amount of global memory:                 3956 MBytes (4148178944 bytes)
            ( 1) Multiprocessors, (128) CUDA Cores/MP:     128 CUDA Cores



Compile and install OpenCV
It is recommended that you install OpenCV in the Home directory.

    Click here to download OpenCV, and unzip in the Home directory.
        wget https://github.com/opencv/opencv/archive/refs/tags/3.4.14.zip
        unzip 3.4.14.zip

    Note: When you execute the command line of cmake, change CUDA_ARCH_BIN to the computing power version of your own platform, NX platform is 7.2. Nano seems to be 5.3

        cd ~/opencv-3.4.14
        mkdir build && cd build
        cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D WITH_CUDA=ON -D CUDA_ARCH_BIN=7.2 -D CUDA_ARCH_PTX="" -D ENABLE_FAST_MATH=ON -D CUDA_FAST_MATH=ON -D WITH_CUBLAS=ON -D WITH_LIBV4L=ON -D WITH_GSTREAMER=ON -D WITH_GSTREAMER_0_10=OFF -D WITH_QT=ON -D WITH_OPENGL=ON -D CUDA_NVCC_FLAGS="--expt-relaxed-constexpr" -D WITH_TBB=ON ..
        
        make -j4


We made it until 3.6. Run the location algorithm, but the stuff is not launching, probably because we need the custom hat camarray shit