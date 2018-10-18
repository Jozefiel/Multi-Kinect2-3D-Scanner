#include "camera.h"
#include "kinect.h"

Camera * Camera::create_camera(libfreenect2::Freenect2 * _freenect, int id)
{
    return new Kinect(_freenect,id);
}
