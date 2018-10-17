#include "support.h"

support::support()
{

}

void support::camera_init(std::vector<Camera*>connected_cams)
{
    //    int connected_realsenses=0;
    //    static int connected_cams;

        libfreenect2::Freenect2 * pFreenect2 = new libfreenect2::Freenect2;             // freenect2 init

        int connected_kinects =pFreenect2->enumerateDevices();                          // number of connected kinects

        for(auto id=0;id<connected_kinects;id++)                                        // add connected kinects to vector cams
        {
            connected_cams.push_back(Camera::create_camera(pFreenect2, id));
        }

        /*
        //  REALSENSE INIT
        //
        */

}

void support::kinect_init()
{

}
