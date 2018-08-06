#include <iostream>
#include <string>
#include <thread>

#include "kinect.h"

#include <opencv2/opencv.hpp>

using namespace std;
uint id1=0, id2=1, id3=2;


int main()
{
//    libfreenect2::setGlobalLogger(nullptr);
    libfreenect2::createConsoleLogger(libfreenect2::Logger::Level::Debug);

    libfreenect2::Freenect2 freenect2;
    libfreenect2::PacketPipeline *pipeline = nullptr;
    libfreenect2::Freenect2 *pFreenect2 = nullptr;
    pFreenect2=&freenect2;

    const int connectedDevices = freenect2.enumerateDevices();

    string serial[connectedDevices];

    if(connectedDevices == 0)
    {
        std::cout << "no device connected!" << std::endl;
        return -1;
    }
    else
    {
        for(int i=0;i<connectedDevices;i++)
        {
           serial[0] = freenect2.getDeviceSerialNumber(i);
        }
    }

    libfreenect2::SyncMultiFrameListener listener0(libfreenect2::Frame::Depth );
    libfreenect2::SyncMultiFrameListener *pListener0=&listener0;
    libfreenect2::SyncMultiFrameListener listener1(libfreenect2::Frame::Depth );
    libfreenect2::SyncMultiFrameListener *pListener1=&listener1;
    libfreenect2::SyncMultiFrameListener listener2(libfreenect2::Frame::Depth );
    libfreenect2::SyncMultiFrameListener *pListener2=&listener2;

    Kinect kinect0(0,pFreenect2,pipeline, pListener0);
    Kinect kinect1(1,pFreenect2,pipeline, pListener1);
    Kinect kinect2(2,pFreenect2,pipeline, pListener2);

//    kinect0.start();
//    kinect0.registration();
//    kinect1.start();
//    kinect1.registration();
//    kinect2.start();
//    kinect2.registration();

    cv::Mat depthmat[3];

    while(1)
    {

        std::thread t1(&Kinect::frames,&kinect0,pListener0);
        t1.join();
        std::thread t2(&Kinect::frames,&kinect1,pListener1);
          t2.join();
        std::thread t3(&Kinect::frames,&kinect2,pListener2);
          t3.join();

//        kinect0.frames(pListener0);
//        kinect1.frames(pListener1);
//        kinect2.frames(pListener2);

    //    pListener2->release(kinect2.frame);


        cv::waitKey(10);

        //        kinects[0];
//        kinects[1];
//        kinects[2];
    }
    return 0;
}
