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

class Camera
{
public:

    static Camera * create_camera(libfreenect2::Freenect2 * _freenect, int id);

    virtual int             getId()     = 0;
    virtual std::string     getSerial() = 0;
    virtual bool            lockCloud(int)  = 0;
    virtual void            unlockCloud()   = 0;

    virtual bool            lockFrames(int) = 0;
    virtual void            unlockFrames()  = 0;

    virtual void            start() = 0;
    virtual void            stop() = 0;

    virtual void            frames(std::atomic<bool> & keep_running)    = 0;
    virtual void            cloudData(std::atomic<bool> & keep_running, std::atomic<bool> & compute_cloud_style) = 0;

    virtual std::string     getCamType()    = 0;
    virtual void            loadCamParams() = 0;

    virtual cv::Mat         getRGB()        = 0;
    virtual cv::Mat         getDepth()      = 0;
    virtual cv::Mat         getIR()         = 0;
    virtual cv::Mat         getRGBD()       = 0;

    virtual cv::Mat         getMask()       = 0;
    virtual cv::Mat         getRangedRGBD() = 0;
    virtual cv::Mat         getRangedDepth()    = 0;


    virtual pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloudData()   = 0;


private:

};

#endif // CAMERA_H
