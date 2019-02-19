#ifndef CAMERA_H
#define CAMERA_H


#include <opencv2/opencv.hpp>
#include <libfreenect2/libfreenect2.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/common/transforms.h>

#define camAttachTime 2000
#define ir_depth_width 512
#define ir_depth_height 424
#define ir_depth_bpp 4
#define color_width 1920
#define color_height 1080+2
#define color_bpp 4

class Camera
{
public:

    static Camera * create_camera(libfreenect2::Freenect2 * _freenect, int id);

    int                     getId();
    std::string             getSerial();
    std::string             getCamType();

    virtual bool            lockCloud(int)  = 0;
    virtual void            unlockCloud()   = 0;

    virtual bool            lockFrames(int) = 0;
    virtual void            unlockFrames()  = 0;

    virtual void            start() = 0;
    virtual void            stop() = 0;

    virtual void            frames(std::atomic<bool> & keep_running)    = 0;
    virtual void            cloudData(std::atomic<bool> & keep_running, std::atomic<bool> & compute_cloud_style) = 0;


    virtual void            loadCamParams() = 0;

    virtual cv::Mat         getRGB()        = 0;
    virtual cv::Mat         getDepth()      = 0;
    virtual cv::Mat         getIR()         = 0;
    virtual cv::Mat         getRGBD()       = 0;

    virtual cv::Mat         getMask()       = 0;
    virtual cv::Mat         getRangedRGBD() = 0;
    virtual cv::Mat         getRangedDepth()  = 0;
    virtual cv::Mat         getHistogram()    = 0;

    virtual pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloudData()   = 0;

    struct camera_frames
    {
        cv::Mat * depthMat          = new cv::Mat( cv::Mat::zeros(ir_depth_height, ir_depth_width, CV_32FC1) );
        cv::Mat * irMat             = new cv::Mat( cv::Mat::zeros(ir_depth_height, ir_depth_width, CV_32FC1) );
        cv::Mat * colorMat          = new cv::Mat( cv::Mat::zeros(color_height, color_width, CV_8UC4) );
        cv::Mat * rgbdMat           = new cv::Mat( cv::Mat::zeros(ir_depth_height, ir_depth_width, CV_8UC4) );

        cv::Mat * rangedDepthMat    = new cv::Mat( cv::Mat::zeros(ir_depth_height, ir_depth_width, CV_32FC1) );
        cv::Mat * rangedRGBDMat     = new cv::Mat( cv::Mat::zeros(ir_depth_height, ir_depth_width, CV_8UC4) );
        cv::Mat * mask              = new cv::Mat( cv::Mat::zeros(ir_depth_height, ir_depth_width, CV_THRESH_BINARY) );
        cv::Mat * histMat           = new cv::Mat( cv::Mat::zeros(ir_depth_height, ir_depth_width, CV_8UC3) );

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    };

    Eigen::Matrix4d transformation_matrix;
    boost::property_tree::ptree pt;

    std::string serial, camera_type;
    int id=0;

private:

};

#endif // CAMERA_H
