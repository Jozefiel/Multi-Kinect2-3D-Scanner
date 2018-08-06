#include <iostream>
#include <string>
#include <thread>

#include "kinect.h"

#include <opencv2/opencv.hpp>

using namespace std;
unsigned int id1=0, id2=1, id3=2;


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

     //   cv::Mat depth=cv::Mat::zeros(424, 512, CV_32FC1);
        cv::Mat * depthmap0 = new cv::Mat( cv::Mat::zeros(424, 512, CV_32FC1) );
        cv::Mat * depthmap1 = new cv::Mat( cv::Mat::zeros(424, 512, CV_32FC1) );

        cv::Mat * depthmap2 = new cv::Mat( cv::Mat::zeros(424, 512, CV_32FC1) );

       // depthmap0 = &depth;

        std::thread t1(&Kinect::frames,&kinect0, depthmap0);
        std::thread t2(&Kinect::frames,&kinect1, depthmap1);
        std::thread t3(&Kinect::frames,&kinect2, depthmap2);

        t3.join();
        t1.join();
        t2.join();

        std::stringstream serial_string0;
        serial_string0 << kinect0.getSerial();
        cv::imshow(serial_string0.str(), *depthmap0 / 2048);

        std::stringstream serial_string1;
        serial_string1 << kinect1.getSerial();
        cv::imshow(serial_string1.str(),* depthmap1 / 2048);

        std::stringstream serial_string2;
        serial_string2 << kinect2.getSerial();
        cv::imshow(serial_string2.str(),*depthmap2 / 2048);

        cv::waitKey(10);

        //delete depthmap0, depthmap1, depthmap2;

 //       kinect0.frames(depthmap0);
//        kinect1.frames();
//        kinect2.frames();


    //    pListener2->release(kinect2.frame);

        //        kinects[0];
//        kinects[1];
//        kinects[2];
    }
    return 0;
}
