 #ifndef PCL_H
#define PCL_H

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/2d/morphology.h>


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

    void pclAddCloud(pcl::PointCloud<pcl::PointXYZRGB> cloud, int);
    void pclCopyCloud(pcl::PointCloud<pcl::PointXYZRGB> kinect_cloud);
    pcl::PointCloud<pcl::PointXYZRGB> getCloud() { return cloud; }


    void setTransformationMatrix(Eigen::Matrix4d transform);
    Eigen::Matrix4d getTransformationMatrix();
    void transformPointCloud();
    void transformPointCloud(Eigen::Matrix4d transform);
    pcl::PointCloud<pcl::PointXYZRGB> getTransformedCloud() { return transformed_cloud; }
    void removeOutliers(int meanK, double mulTresh);
    void mergeClouds(std::vector<pcl::PointCloud<pcl::PointXYZRGB>> clouds);


    void computeNormals();
    pcl::PointCloud<pcl::Normal> getCloudNormals() { return normals_cloud; }

    void creteMesh(int kSearch);


private:
    int id=0;
    std::string serial = "";

    Eigen::Matrix4d transform_matrix;
    pcl::PointCloud<pcl::PointXYZRGB> cloud ;//= pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB> transformed_cloud ;//= pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::Normal> normals_cloud ;//= pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);

    boost::property_tree::ptree pt;

};

#endif // PCL_H
