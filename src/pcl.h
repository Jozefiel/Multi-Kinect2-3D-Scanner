#ifndef PCL_H
#define PCL_H

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

#include <chrono>
#include <thread>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

class pclCloud
{
public:
    pclCloud(int cam_id, std::string cam_serial);
    pclCloud(int cam_id);

    ~pclCloud();

    void pclAddCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int);
    void pclCopyCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr kinect_cloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud();


    void setTransformationMatrix(Eigen::Matrix4f transform_matrix);
    void transformPointCloud();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getTransformedCloud();
    void removeOutliers(int meanK, double mulTresh);
    void mergeClouds(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds);
    void creteMesh(int kSearch);

private:
    int id=0;
    std::string serial = "";

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    boost::property_tree::ptree pt;

};

/***********************************************************************************************************************************************************/
class pclViewer
{
public:
    pclViewer(int view_ports, std::string window_name);
    ~pclViewer();

    void pclAddCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud0, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2);
    void pclAddCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int);
    void spinOnce();
    void pclCopyCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr kinect_cloud);


private:
    pcl::visualization::PCLVisualizer *viewer =nullptr;
    std::vector<int> viewPortsId;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    int id=0;

};

#endif // PCL_H
