#ifndef KINECT_H
#define KINECT_H

#include "camera.h"

#include <string>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <atomic>
#include <mutex>
#include <chrono>
#include <thread>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/frame_listener.hpp>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#define chrono_counter

#define EIGEN_MALLOC_ALREADY_ALIGNED 0

#define camAttachTime 2000
#define ir_depth_width 512
#define ir_depth_height 424
#define ir_depth_bpp 4
#define color_width 1920
#define color_height 1080+2
#define color_bpp 4

class Kinect : public Camera
{
public:

    Kinect(libfreenect2::Freenect2 *_freenect, int id);     // kinect object
    void        start();                                    // kinect start
    void        stop();

    void        registration();                             // kinect register
    void        frames(std::atomic<bool> & keep_running);   // kinect wrapper to opencv

    cv::Mat     getRGB() { return *colorMat; }                                    // return opencv mat
    cv::Mat     getDepth() { return *depthMat; }
    cv::Mat     getIR() { return *irMat; }
    cv::Mat     getRGBD() { return *rgbdMat; }
    cv::Mat     getMask() { return *mask; }
    cv::Mat     getRangedDepth() { return *rangedDepthMat; }
    cv::Mat     getRangedRGBD() { return *rangedRGBDMat; }

    void        depth2cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr  &tmpCloud);
    void        registered2cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &tmpCloud);

    void        cloudData(std::atomic<bool> & keep_running, std::atomic<bool> &compute_cloud_style); // kinect wrapper to pcl
    void        rangeFrames(int lowTreshold, int highTreshold);
    void        computeHist();


    void        cloudInit();                                // prepare cloud for copying
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloudData() { return cloud; }

    int getId();                                            // return information about camera
    std::string getSerial();
    std::string getCamType();
    void loadCamParams();


    bool lockCloud(int lock_time);
    void unlockCloud();
    bool lockFrames(int lock_time);
    void unlockFrames();
    
    void matrix_rotat_koef();
    

    ~Kinect();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

    std::string serial, camera_type;
    int id=0;

    libfreenect2::PacketPipeline * pPipeline            = nullptr;
    libfreenect2::Freenect2 * pFreenect                 = nullptr;
    libfreenect2::Freenect2Device * pDev                = nullptr;
    libfreenect2::Registration * pRegistrated           = nullptr;
    libfreenect2::SyncMultiFrameListener * pListener    = nullptr;
    libfreenect2::FrameMap frame;

    libfreenect2::Frame *undistorted    = new libfreenect2::Frame(ir_depth_width, ir_depth_height, ir_depth_bpp);
    libfreenect2::Frame *undistortedDepth = new libfreenect2::Frame(ir_depth_width, ir_depth_height, ir_depth_bpp);
    libfreenect2::Frame *registered     = new libfreenect2::Frame(ir_depth_width, ir_depth_height, ir_depth_bpp);
    libfreenect2::Frame *depth2rgb      = new libfreenect2::Frame(color_width,color_height, color_bpp);

    cv::Mat * depthMat          = new cv::Mat( cv::Mat::zeros(ir_depth_height, ir_depth_width, CV_32FC1) );
    cv::Mat * irMat             = new cv::Mat( cv::Mat::zeros(ir_depth_height, ir_depth_width, CV_32FC1) );
    cv::Mat * colorMat          = new cv::Mat( cv::Mat::zeros(1082, 1920, CV_8UC4) );
    cv::Mat * rgbdMat           = new cv::Mat( cv::Mat::zeros(ir_depth_height, ir_depth_width, CV_8UC4) );

    cv::Mat * rangedDepthMat    = new cv::Mat( cv::Mat::zeros(ir_depth_height, ir_depth_width, CV_32FC1) );
    cv::Mat * rangedRGBDMat     = new cv::Mat( cv::Mat::zeros(ir_depth_height, ir_depth_width, CV_8UC4) );
    cv::Mat * dDepthMat    = new cv::Mat( cv::Mat::zeros(ir_depth_height, ir_depth_width, CV_32FC1) );
    cv::Mat * dRGBDMat     = new cv::Mat( cv::Mat::zeros(ir_depth_height, ir_depth_width, CV_8UC4) );

    cv::Mat * mask              = new cv::Mat( cv::Mat::zeros(ir_depth_height, ir_depth_width, CV_THRESH_BINARY) );
    cv::Mat * histMat           = new cv::Mat( cv::Mat::zeros(ir_depth_height, ir_depth_width, CV_8UC1) );


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    Eigen::Matrix4d transformation_matrix;

    boost::property_tree::ptree pt;
    std::timed_mutex frame_mutex;
    std::timed_mutex cloud_mutex;
    libfreenect2::Freenect2Device::ColorCameraParams    rgb_calib_params;
    libfreenect2::Freenect2Device::IrCameraParams       ir_calib_params;

    int matrix[16]={0,0,1070,0,0,0,749,0,616};
    double fmatrix[16]={0};
};
#endif // KINECT_H
