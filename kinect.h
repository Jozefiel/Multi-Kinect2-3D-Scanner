#ifndef KINECT_H
#define KINECT_H

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/frame_listener.hpp>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <opencv2/opencv.hpp>


#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>


#include <string>
#include <iostream>
#include <stdio.h>


class Kinect
{
public:
    Kinect(int id, libfreenect2::Freenect2 *freenect2, libfreenect2::PacketPipeline *pipeline);
    ~Kinect();
    void start();
    void registration();

    void newFrames();
    void frames();
    void rangedDepth();
    void rangedRGBD();

    void getDepth();
    void getRangedDepth();
    void getRGB();
    void getIr();
    void getRGBD();
    void getRangedRGBD();

    void cloudInit();
    void cloudData();



    std::string getSerial();
    std::string getId();
    void depthControl();

private:
    std::string serial;
    int kinect_id;

    libfreenect2::Freenect2Device *dev = nullptr;
    libfreenect2::Registration * registrated = nullptr;
    libfreenect2::SyncMultiFrameListener *listener;
    libfreenect2::FrameMap frame;

    cv::Mat * depthMat = new cv::Mat( cv::Mat::zeros(424, 512, CV_32FC1) );
    cv::Mat * rangedDepthMap = new cv::Mat( cv::Mat::zeros(424, 512, CV_32FC1) );
    cv::Mat * irMat = new cv::Mat( cv::Mat::zeros(424, 512, CV_32FC1) );
    cv::Mat * colorMat = new cv::Mat( cv::Mat::zeros(1082, 1920, CV_8UC4) );
    cv::Mat * rgbdMat = new cv::Mat( cv::Mat::zeros(1082, 1920, CV_8UC4) );
    cv::Mat * rangedRGBDMat = new cv::Mat( cv::Mat::zeros(1082, 1920, CV_8UC4) );
    cv::Mat * rangeMask = new cv::Mat( cv::Mat::zeros(424, 512, CV_8U) );
    int low_slider=0;
    int high_slider=65535;

    pcl::PointCloud<pcl::PointXYZRGB> *cloud = new pcl::PointCloud<pcl::PointXYZRGB>;
};
#endif // KINECT_H
