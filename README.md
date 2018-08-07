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
 libfreenect2 -> https://github.com/OpenKinect/libfreenect2/blob/master/README.md#linux


## Troubleshooting and reporting bugs

First, check https://github.com/OpenKinect/libfreenect2/wiki/Troubleshooting for known issues.
