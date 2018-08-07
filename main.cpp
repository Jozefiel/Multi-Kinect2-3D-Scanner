#include <iostream>
#include <string>
#include <thread>

#include "kinect.h"

#include <opencv2/opencv.hpp>

using namespace std;
int main()
{


    libfreenect2::setGlobalLogger(nullptr);

    libfreenect2::PacketPipeline *pipeline = nullptr;
    libfreenect2::Freenect2 *pFreenect2 = new libfreenect2::Freenect2;


    static int connectedDevices = pFreenect2->enumerateDevices();

    string * serial = new string [connectedDevices];

    if(connectedDevices == 0)
    {
        std::cerr << "no device connected!" << std::endl;
        return -1;
    }
    else
    {
        for(int i=0;i<connectedDevices;i++)
        {
           serial[i] = pFreenect2->getDeviceSerialNumber(i);
        }
    }

    Kinect kinect0(0,pFreenect2,pipeline);
    Kinect kinect1(1,pFreenect2,pipeline);
    Kinect kinect2(2,pFreenect2,pipeline);

    bool loop=true;
    while(loop)
    {

        std::thread t1(&Kinect::frames,&kinect0);
        std::thread t2(&Kinect::frames,&kinect1);
        std::thread t3(&Kinect::frames,&kinect2);

        t1.join();
        t2.join();
        t3.join();

        kinect0.getDepth();
      //  kinect1.getDepth();
      //  kinect2.getDepth();

//        kinect0.getRGB();
//        kinect1.getRGB();
//        kinect2.getRGB();

        kinect0.getRangedDepth();
      //  kinect1.getRangedDepth();
      //  kinect2.getRangedDepth();


        kinect0.getRGBD();
      //  kinect1.getRGBD();
      //  kinect2.getRGBD();

        kinect0.getRangedRGBD();
       // kinect1.getRangedRGBD();
       // kinect2.getRangedRGBD();


        char key = cv::waitKey(30);
        switch(key)
        {
            case 'Q':
            case 'q':
            case 27 :
                loop=false;
            break;
        }
    }

    kinect0.~Kinect();
    kinect1.~Kinect();
    kinect2.~Kinect();

    return 0;
}
