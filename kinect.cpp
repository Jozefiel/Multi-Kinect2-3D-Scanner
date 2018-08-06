#include "kinect.h"

Kinect::Kinect(unsigned int id, libfreenect2::Freenect2 *freenect2, libfreenect2::PacketPipeline *pipeline,  libfreenect2::SyncMultiFrameListener  *pListener)
{
    kinect_id = id;
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
    listener=pListener;
    start();
    registration();
    dev->setColorFrameListener(listener);
    dev->setIrAndDepthFrameListener(listener);

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


void Kinect::frames(cv::Mat * depth_map)
{
        libfreenect2::FrameMap frame;
        listener->waitForNewFrame(frame,400);

        libfreenect2::Frame *depth=frame[libfreenect2::Frame::Depth];
        cv::Mat depthmat;
        cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);
        * depth_map = depthmat;
//        std::stringstream serial_string;
//        serial_string << serial;
//        cv::imshow(serial_string.str(), *depth_map / 2048);
//        cv::waitKey(10);
//        auto time = std::time(nullptr);
//        std::cout << kinect_id<<" : " << std::put_time(std::gmtime(&time), "%c") << '\n';
        listener->release(frame);

}

std::string Kinect::getSerial()
{
    std::string _serial = serial;
    return _serial;
}

void Kinect::getDepth()
{
    auto time = std::time(nullptr);
    std::cout << kinect_id<<" : " << std::put_time(std::gmtime(&time), "%c") << '\n';
}

Kinect::~Kinect()
{
    std::cout<<"END "<<dev->getSerialNumber()<<std::endl;
    dev->stop();
    dev->close();
    delete registrated;
}



