#include "pcl.h"

pclCloud::pclCloud(int cam_id)
{
    id=cam_id;
}

void pclCloud::pclCopyCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cam_cloud)
{
    if(!cam_cloud->empty())
    {
        cloud->clear();
        pcl::copyPointCloud(*cam_cloud,*cloud);
    }
    else
    {
        std::cout<<"empty cloud: "<<id<<std::endl;
    }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud::getCloud()
{
    return cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud::getTransformedCloud()
{
    return transformed_cloud;
}

void pclCloud::setTransformationMatrix(Eigen::Matrix4f transform)
{
   transform_matrix=transform;
}

void pclCloud::transformPointCloud()
{
    pcl::transformPointCloud(*cloud,*transformed_cloud,transform_matrix);
}

void pclCloud::removeOutliers(int meanK, double mulTresh)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(meanK);
    sor.setStddevMulThresh(mulTresh);
    sor.filter(*cloud);
}

void pclCloud::mergeClouds(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds)
{
    cloud->clear();
    for(auto i=0;i<clouds.size();i++)
    {
        *cloud+=*clouds[i];
    }
}


pclCloud::~pclCloud()
{
}


//********************************************************************************************************************************************************//

pclViewer::pclViewer(int view_ports, std::string window_name)
{
    if(window_name.empty())
    {
        window_name="PCL_Viewer";
    }
    viewer=new pcl::visualization::PCLVisualizer(window_name);
    double scale_size=(0.99/view_ports);
    for(int i=0;i<view_ports;i++)
    {
        int viewPortId (0);
        viewPortsId.push_back(viewPortId);
        viewer->createViewPort (scale_size*i,  0.0, scale_size+scale_size*i, 1.0, viewPortsId[i]);
    }
}

void pclViewer::pclCopyCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cam_cloud)
{
    if(!cam_cloud->empty())
    {
        pcl::copyPointCloud(*cam_cloud,*cloud);
    }
}

void pclViewer::pclAddCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud0, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2)
{
    if(!cloud0->empty() && !cloud0->empty() && !cloud0->empty())
    {
        viewer->removePointCloud("cloud0");
        viewer->addPointCloud(cloud0,"cloud0");
        viewer->removePointCloud("cloud1");
        viewer->addPointCloud(cloud1,"cloud1");
        viewer->removePointCloud("cloud2");
        viewer->addPointCloud(cloud2,"cloud2");
    }
}


void pclViewer::pclAddCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloud, int cam_id)
{
    std::stringstream id_string;
    id_string << cam_id;
    if(!pCloud->empty())
    {
        viewer->removePointCloud("cloud_"+id_string.str());
        viewer->addPointCloud(pCloud,"cloud_"+id_string.str(),viewPortsId[cam_id]);
    }
}

void pclViewer::spinOnce()
{
    viewer->spinOnce();
}

pclViewer::~pclViewer()
{
}
