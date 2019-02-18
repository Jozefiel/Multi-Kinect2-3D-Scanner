#include "camera.h"
#include "kinect.h"

Camera * Camera::create_camera(libfreenect2::Freenect2 * _freenect, int id)
{
    return new Kinect(_freenect,id);
}

int  Camera::getId() { return id; }                                            // return information about camera
std::string  Camera::getSerial() { return serial; }
std::string  Camera::getCamType()  { return camera_type;}
