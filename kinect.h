#ifndef KINECT_H
#define KINECT_H

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/frame_listener.hpp>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"



#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>
#include <pcl/conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>

#include <string>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <atomic>
#include <mutex>
#include <chrono>
#include <thread>

#define camAttachTime 2000
#define ir_depth_width 512
#define ir_depth_height 424
#define ir_depth_bpp 4
#define color_width 1920
#define color_height 1080+2
#define color_bpp 4

static std::mutex ui_locker;
static std::mutex cloud_locker;



class Kinect
{
public:
    Kinect(int id, libfreenect2::Freenect2 *freenect2, libfreenect2::PacketPipeline *pipeline);
    ~Kinect();
    void start();
    void registration();

    void newFrames();
//    void frames();
    void frames(std::atomic<bool> &keep_running);
    void rangedDepth(std::string window_name);
    void rangedRGBD(std::string window_name);

    void getDepth();
    void getRangedDepth();
    void getRGB();
    void getIr();
    void getRGBD();
    void getRangedRGBD();

    void headDetect();
    void loadCascades();
    void setCascades(std::string cascade_head, std::string cascade_other, bool load_cascade);

    void cloudInit();
    void cloudDataThread(std::atomic<bool> & keep_running);
    void cloudData();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloudData();


    std::string getSerial();
    std::string getIdString();

    int getId();
    void depthControl(std::string window_name);

    //variables



private:
    std::string serial;
    int kinect_id;

    libfreenect2::Freenect2Device *dev = nullptr;
    libfreenect2::Registration * registrated = nullptr;
    libfreenect2::SyncMultiFrameListener *listener;
    libfreenect2::FrameMap frame;

    libfreenect2::Frame *undistorted = new libfreenect2::Frame(ir_depth_width, ir_depth_height, ir_depth_bpp);
    libfreenect2::Frame *registered = new libfreenect2::Frame(ir_depth_width, ir_depth_height, ir_depth_bpp);
    libfreenect2::Frame *depth2rgb = new libfreenect2::Frame(color_width,color_height, color_bpp);

    cv::Mat * depthMat = new cv::Mat( cv::Mat::zeros(424, 512, CV_32FC1) );
    cv::Mat * rangedDepthMap = new cv::Mat( cv::Mat::zeros(424, 512, CV_32FC1) );
    cv::Mat * irMat = new cv::Mat( cv::Mat::zeros(424, 512, CV_32FC1) );
    cv::Mat * colorMat = new cv::Mat( cv::Mat::zeros(1082, 1920, CV_8UC4) );
    cv::Mat * rgbdMat = new cv::Mat( cv::Mat::zeros(1082, 1920, CV_8UC4) );
    cv::Mat * rangedRGBDMat = new cv::Mat( cv::Mat::zeros(1082, 1920, CV_8UC4) );
    cv::Mat * rangeMask = new cv::Mat( cv::Mat::zeros(424, 512, CV_8U) );
    int low_slider=5000;
    int high_slider=8000;
    int low_depth_slider = 0;
    int high_depth_slider=10;

    cv::CascadeClassifier face_cascade;
    cv::CascadeClassifier eyes_cascade;

    std::string face_cascade_name = "/home/jozef/Documents/QT/3DScan/cascades/haarcascades/haarcascade_fullbody.xml";
    std::string other_cascade_name = "/home/jozef/Documents/QT/3DScan/cascades/haarcascades/haarcascade_eye_tree_eyeglasses.xml";

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

};
#endif // KINECT_H
