#include "kinect.h"

Kinect::Kinect(int id, libfreenect2::Freenect2 *freenect2, libfreenect2::PacketPipeline *pipeline,  libfreenect2::SyncMultiFrameListener  *pListener)
{
    serial = freenect2->getDeviceSerialNumber(id);
    std::cout<<serial<<std::endl;
    if(pipeline)
    {
        dev = freenect2->openDevice(serial, pipeline);
    }
    else
    {
        dev = freenect2->openDevice(serial);
    }
    start();
    registration();
    dev->setColorFrameListener(pListener);
    dev->setIrAndDepthFrameListener(pListener);

}

void Kinect::start()
{
    std::cout<<"Kinect start: "<<dev->getSerialNumber()<<std::endl;
    dev->start();
}

void Kinect::registration()
{
    std::cout<<"REGISTERED "<<dev->getSerialNumber()<<std::endl;
    registrated = new libfreenect2::Registration(dev->getIrCameraParams(),dev->getColorCameraParams());
}


void Kinect::frames(libfreenect2::SyncMultiFrameListener * pListener)
{
        libfreenect2::FrameMap frame;
        pListener->waitForNewFrame(frame,5000);

        libfreenect2::Frame *depth=frame[libfreenect2::Frame::Depth];
        cv::Mat depthmat;
        cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);
        std::stringstream serial_string;
        serial_string << serial;
        cv::imshow(serial_string.str(),depthmat / 2048);
        cv::waitKey(10);
        pListener->release(frame);

}


void Kinect::getDepth()
{

}

Kinect::~Kinect()
{
    std::cout<<"END "<<dev->getSerialNumber()<<std::endl;
    dev->stop();
    dev->close();
    delete registrated;
}



