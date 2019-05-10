#include "support.h"

std::vector<Camera *> support::getConnectedCams()
{
    return connected_cams;
}

void support::cloudInit()
{
    for(auto id=0;id<this->connectedCameras();id++)                                        // add connected kinects to vector cams
    {
        this->clouds.push_back(pclCloud(this->cameras()[id]->getId(),this->cameras()[id]->getSerial()));
    }
    std::cout<<"support::cloudInit started"<<std::endl;
}

std::vector<pclCloud> support::getClouds()
{
    return clouds;
}

void support::transformCloud()
{
    for(auto id=0;id<this->connectedCameras();id++)
    {
        this->clouds[id].transformPointCloud();
    }
    std::cout<<"support::transformCloud"<<std::endl;
}

void support::transformCloud(std::vector<Eigen::Matrix4d> transform_matrix)
{
    for(auto id=0;id<this->connectedCameras();id++)
    {
        this->clouds[id].transformPointCloud(transform_matrix[id]);
    }
    std::cout<<"support::transformCloud"<<std::endl;
}

pcl::PointCloud<pcl::PointXYZRGB> support::getCloudData(int id)
{
    return clouds[id].getCloud();
}

pcl::PointCloud<pcl::PointXYZRGB> support::getTransformedCloudData(int id)
{
    return clouds[id].getTransformedCloud();
}

std::vector<pcl::PointCloud<pcl::PointXYZRGB>> support::mergeClouds(bool transformed)
{
//    merged_clouds.clear();
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>> temp_clouds;

    if(transformed)
    {
        transformCloud();
        for(auto id=0;id<this->connectedCameras();id++)                                        // add connected kinects to vector cams
        {
            temp_clouds.push_back(clouds[id].getTransformedCloud());
        }
    }
    else
    {
        for(auto id=0;id<this->connectedCameras();id++)                                        // add connected kinects to vector cams
        {
            temp_clouds.push_back(clouds[id].getCloud());
        }
    }
//    std::cout<<"support::mergeClouds"<<std::endl;
    return temp_clouds;
}

std::vector<pcl::PointCloud<pcl::PointXYZRGB> > support::mergeClouds(bool transformed, std::vector<Eigen::Matrix4d> transform_matrix)
{
//    merged_clouds.clear();
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>> temp_clouds;

    if(transformed)
    {
        transformCloud(transform_matrix);
        for(auto id=0;id<this->connectedCameras();id++)                                        // add connected kinects to vector cams
        {
            temp_clouds.push_back(clouds[id].getTransformedCloud());
        }
    }
    else
    {
        for(auto id=0;id<this->connectedCameras();id++)                                        // add connected kinects to vector cams
        {
            temp_clouds.push_back(clouds[id].getCloud());
        }
    }
//    std::cout<<"support::mergeClouds"<<std::endl;
    return temp_clouds;
}
