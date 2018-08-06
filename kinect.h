#ifndef KINECT_H
#define KINECT_H

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/frame_listener.hpp>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <opencv2/opencv.hpp>

#include <string>
#include <iostream>
#include <stdio.h>


class Kinect
{
public:
    Kinect(unsigned int id, libfreenect2::Freenect2 *freenect2, libfreenect2::PacketPipeline *pipeline, libfreenect2::SyncMultiFrameListener *pListener);
    ~Kinect();
    void start();
    void registration();
    void newFrames();
    void frames(cv::Mat * depth_map);
    void getDepth();
    std::string getSerial();

private:
    std::string serial;
    libfreenect2::Freenect2Device *dev = nullptr;
    libfreenect2::Registration * registrated = nullptr;
    unsigned int kinect_id;
    libfreenect2::SyncMultiFrameListener *listener;

};
#endif // KINECT_H
