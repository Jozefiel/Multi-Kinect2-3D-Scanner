 #ifndef PCL_H
#define PCL_H
#include "globalsettings.h"

#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/2d/morphology.h>

#include <pcl/search/impl/search.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

#include <chrono>
#include <thread>
#include <mutex>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

class pclCloud
{
public:
    pclCloud(int cam_id, int connectedCam);

    bool pclCopyCloud(pcl::PointCloud<pcl::PointXYZRGB> kinect_cloud, int camId);
    bool mergeClouds(int camId);
    bool mergeAllClouds();
    bool mergeLastClouds();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getMergedCloud();

    bool lockCloud(int lock_time);
    void unlockCloud();


//    std::vector<pcl::PointCloud<pcl::PointXYZRGB>> getCloud() { return camera_clouds; }

    void transformPointCloud();
    void transformPointCloud(Eigen::Matrix4d transform);
    pcl::PointCloud<pcl::PointXYZRGB> getTransformedCloud() { return transformed_cloud; }

    void removeOutliers(int meanK, double mulTresh);


    void computeNormals();
    pcl::PointCloud<pcl::Normal> getCloudNormals() { return normals_cloud; }

    void creteMesh(int kSearch);

    ~pclCloud();

private:

    int id=0;

    std::timed_mutex cloud_mutex;
    std::shared_ptr<GlobalSettings> globalSettings = globalSettings->instance();
    std::vector<std::vector<pcl::PointCloud<pcl::PointXYZRGB>>> camera_clouds;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>> camera_merged_clouds;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_clouds_viewer = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    //= pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB> transformed_cloud ;//= pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::Normal> normals_cloud ;//= pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);

    boost::property_tree::ptree pt;

};

#endif // PCL_H
