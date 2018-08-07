# 3D scanner based on Kinect v2

## Table of Contents

* [License](README.md#about)
* [License](README.md#license)
* [Maintainers](README.md#maintainers)
* [Requirements](README.md#requirements)
* [Installation](README.md#installation)
  * [Linux](README.md#linux)
* [Troubleshooting](README.md#troubleshooting-and-reporting-bugs)

## About

3D scanner software is based on  for Kinect for Windows v2 (K4W2) devices (release and developer preview).
DOI: [![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.50641.svg)](https://doi.org/10.5281/zenodo.50641)

If you use the KDE depth unwrapping algorithm implemented in the library, please also cite this ECCV 2016 [paper](http://users.isy.liu.se/cvl/perfo/abstracts/jaremo16.html).

This software supports:
* Multiple RGBD image processing
* Multiple PCL projection
* Multi-cloud alignating

## Requirements
Software
  *Linux
    *Libfreenect2
    *OpenCV 3.4.2
    *PCL
    *VTK
Hardware
  *PC with USB 3.0 controllers
    *each Kinect v2 require own usb 3.0 expansion card

### Requirements for optional features

## Maintainers

* Jozef Volák <jozef.volak@fel.uniza.sk>

## Installation
### Linux

Note: Ubuntu 12.04 is too old to support. Debian jessie may also be too old, and Debian stretch is implied in the following.

* Download libfreenect2 source
    ```
    git clone https://github.com/OpenKinect/libfreenect2.git
    cd libfreenect2
    ```
* (Ubuntu 14.04 only) Download upgrade deb files
    ```
    cd depends; ./download_debs_trusty.sh
    ```
* Install build tools
    ```
    sudo apt-get install build-essential cmake pkg-config
    ```
* Install libusb. The version must be >= 1.0.20.
    1. (Ubuntu 14.04 only) `sudo dpkg -i debs/libusb*deb`
    2. (Other) `sudo apt-get install libusb-1.0-0-dev`
* Install TurboJPEG
    1. (Ubuntu 14.04 to 16.04) `sudo apt-get install libturbojpeg libjpeg-turbo8-dev`
    2. (Debian/Ubuntu 17.10 and newer) `sudo apt-get install libturbojpeg0-dev`
* Install OpenGL
    1. (Ubuntu 14.04 only) `sudo dpkg -i debs/libglfw3*deb; sudo apt-get install -f`
    2. (Odroid XU4) OpenGL 3.1 is not supported on this platform. Use `cmake -DENABLE_OPENGL=OFF` later.
    3. (Other) `sudo apt-get install libglfw3-dev`
* Install OpenCL (optional)
    - Intel GPU
        1. (Ubuntu 14.04 only) `sudo apt-add-repository ppa:floe/beignet; sudo apt-get update; sudo apt-get install beignet-dev; sudo dpkg -i debs/ocl-icd*deb`
        2. (Other) `sudo apt-get install beignet-dev`
        3. For older kernels, `# echo 0 >/sys/module/i915/parameters/enable_cmd_parser` is needed. See more known issues at https://www.freedesktop.org/wiki/Software/Beignet/.
    - AMD GPU: Install the latest version of the AMD Catalyst drivers from https://support.amd.com and `apt-get install opencl-headers`.
    - Mali GPU (e.g. Odroid XU4): (with root) `mkdir -p /etc/OpenCL/vendors; echo /usr/lib/arm-linux-gnueabihf/mali-egl/libmali.so >/etc/OpenCL/vendors/mali.icd; apt-get install opencl-headers`.
    - Verify: You can install `clinfo` to verify if you have correctly set up the OpenCL stack.
* Install CUDA (optional, Nvidia only):
    - (Ubuntu 14.04 only) Download `cuda-repo-ubuntu1404...*.deb` ("deb (network)") from Nvidia website, follow their installation instructions, including `apt-get install cuda` which installs Nvidia graphics driver.
    - (Jetson TK1) It is preloaded.
    - (Nvidia/Intel dual GPUs) After `apt-get install cuda`, use `sudo prime-select intel` to use Intel GPU for desktop.
    - (Other) Follow Nvidia website's instructions. You must install the samples package.
* Install VAAPI (optional, Intel only)
    1. (Ubuntu 14.04 only) `sudo dpkg -i debs/{libva,i965}*deb; sudo apt-get install -f`
    2. (Other) `sudo apt-get install libva-dev libjpeg-dev`
    3. Linux kernels 4.1 to 4.3 have performance regression. Use 4.0 and earlier or 4.4 and later (Though Ubuntu kernel 4.2.0-28.33~14.04.1 has backported the fix).
* Install OpenNI2 (optional)
    1. (Ubuntu 14.04 only) `sudo apt-add-repository ppa:deb-rob/ros-trusty && sudo apt-get update` (You don't need this if you have ROS repos), then `sudo apt-get install libopenni2-dev`
    2. (Other) `sudo apt-get install libopenni2-dev`
* Build (if you have run `cd depends` previously, `cd ..` back to the libfreenect2 root directory first.)
    ```
    mkdir build && cd build
    cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/freenect2
    make
    make install
    ```
    You need to specify `cmake -Dfreenect2_DIR=$HOME/freenect2/lib/cmake/freenect2` for CMake based third-party application to find libfreenect2.
* Set up udev rules for device access: `sudo cp ../platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/`, then replug the Kinect.
* Run the test program: `./bin/Protonect`
* Run OpenNI2 test (optional): `sudo apt-get install openni2-utils && sudo make install-openni2 && NiViewer2`. Environment variable `LIBFREENECT2_PIPELINE` can be set to `cl`, `cuda`, etc to specify the pipeline.


## Troubleshooting and reporting bugs

First, check https://github.com/OpenKinect/libfreenect2/wiki/Troubleshooting for known issues.
