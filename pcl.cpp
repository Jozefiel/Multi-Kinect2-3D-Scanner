#include "pcl.h"

pclKinect::pclKinect(int connectedDevices)
{
    viewer=new pcl::visualization::PCLVisualizer("3D Viewer");

    for(int i=0;i<connectedDevices;i++)
    {
        int viewPortId (0);
        viewPortsId.push_back(viewPortId);
    }

    for(int i=0;i<connectedDevices;i++)
    {
        viewer->createViewPort (0.33*i,  0.0, 0.33*i, 1.0, viewPortsId[i]);
    }

}

void pclKinect::pclAddCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int kinect_id)
{
    std::stringstream id_string;
    id_string << kinect_id;
    viewer->removePointCloud("cloud_"+id_string.str());
    viewer->addPointCloud(cloud,"cloud_"+id_string.str(),viewPortsId[kinect_id]);
}

void pclKinect::spinOnce()
{
    viewer->spinOnce();
}
