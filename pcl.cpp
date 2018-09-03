#include "pcl.h"

pclCloud::pclCloud(int cam_id)
{
    id=cam_id;
}

void pclCloud::pclCopyCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cam_cloud)
{
    pcl::copyPointCloud(*cam_cloud,*cloud);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud::getCloud()
{
    return cloud;
}

pclCloud::~pclCloud()
{
}


//********************************************************************************************************************************************************//

pclViewer::pclViewer(const int connectedDevices)
{
    viewer=new pcl::visualization::PCLVisualizer("3D Viewer");
    double scale_size=(0.99/connectedDevices);
    for(int i=0;i<connectedDevices;i++)
    {
        int viewPortId (0);
        viewPortsId.push_back(viewPortId);
        viewer->createViewPort (scale_size*i,  0.0, scale_size+scale_size*i, 1.0, viewPortsId[i]);
    }
}

void pclViewer::pclCopyCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr kinect_cloud)
{
    if(!kinect_cloud->empty())
    {
        pcl::copyPointCloud(*kinect_cloud,*cloud);
    }
}

void pclViewer::pclAddCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int cam_id)
{
    std::stringstream id_string;
    id_string << cam_id;
    if(!cloud->empty())
    {
        viewer->removePointCloud("cloud_"+id_string.str());
        viewer->addPointCloud(cloud,"cloud_"+id_string.str(),viewPortsId[cam_id]);
    }
}

void pclViewer::spinOnce()
{
    viewer->spinOnce();
}

pclViewer::~pclViewer()
{
}
