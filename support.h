#ifndef SUPPORT_H
#define SUPPORT_H

#include "camera.h"
#include "kinect.h"

class support
{
public:
    support();
    void camera_init(std::vector<Camera*> connected_cams);
    void kinect_init();
private:
    std::vector<Camera*> connected_cams;
};

#endif // SUPPORT_H
