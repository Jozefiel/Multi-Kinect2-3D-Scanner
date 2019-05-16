#include "pcl.h"

pclCloud::pclCloud(int cam_id, int connectedCams)
{
    std::cout << "pclCloud::pclCloud: created cloud without calibration parameters "<<std::endl;
    id=cam_id;
    camera_clouds.resize(static_cast<ulong>(connectedCams));
    camera_merged_clouds.resize(static_cast<ulong>(connectedCams));
}

bool pclCloud::pclCopyCloud(pcl::PointCloud<pcl::PointXYZRGB> cam_cloud, int camId)
{
    auto tmp_cloud=cam_cloud;
    ulong _camId= static_cast<ulong>(camId);
    camera_clouds[static_cast<ulong>(camId)].emplace_back(tmp_cloud);

    if( camera_clouds[static_cast<ulong>(camId)].size() > static_cast<ulong>(globalSettings.operator->()->getBufferSize()))
    {
        camera_clouds[_camId].erase(camera_clouds[_camId].begin());
    }
}

bool pclCloud::mergeClouds(int camId)
{
    try {
        ulong _camId=static_cast<ulong>(camId);
        pcl::PointCloud<pcl::PointXYZRGB> cloud ;
        for(ulong i=0; i<camera_clouds[_camId].size();i++)
        {
            if(!camera_clouds[_camId].at(i).empty())
                cloud+=camera_clouds[_camId].at(i);
        }
        std::cout<<"Merged cloud size: " << cloud.points.size()<<" , id: " << _camId <<std::endl;
    } catch (...)
    {
        std::cout<<this->id<<" Problem with  pclCloud::mergeClouds"<<std::endl;
    }
}

pcl::PointCloud<pcl::PointXYZRGB> pclCloud::getMergedCloud()
{

}



void pclCloud::setTransformationMatrix(Eigen::Matrix4d transform)
{
    transform_matrix (0,0) = transform (0,0);
    transform_matrix (0,1) = transform (0,1);
    transform_matrix (0,2) = transform (0,2);
    transform_matrix (0,3) = transform (0,3);
    transform_matrix (1,0) = transform (1,0);
    transform_matrix (1,1) = transform (1,1);
    transform_matrix (1,2) = transform (1,2);
    transform_matrix (1,3) = transform (1,3);
    transform_matrix (2,0) = transform (2,0);
    transform_matrix (2,1) = transform (2,1);
    transform_matrix (2,2) = transform (2,2);
    transform_matrix (2,3) = transform (2,3);
    transform_matrix (3,0) = transform (3,0);
    transform_matrix (3,1) = transform (3,1);
    transform_matrix (3,2) = transform (3,2);
    transform_matrix (3,3) = transform (3,3);
}

Eigen::Matrix4d pclCloud::getTransformationMatrix()
{
    return transform_matrix;
}

void pclCloud::transformPointCloud()
{
//    if(!cloud.empty())
//    {
//        pcl::transformPointCloud(cloud,transformed_cloud,transform_matrix,true);
//    }
//    else
    {
        std::cout<<"cloud wasn't transformed"<<std::endl;
    }
}

void pclCloud::transformPointCloud(Eigen::Matrix4d transform)
{
//    if(!cloud.empty())
//    {
//        pcl::transformPointCloud(cloud,transformed_cloud,transform,true);
//    }
//    else
    {
        std::cout<<"cloud wasn't transformed"<<std::endl;
    }
}

void pclCloud::removeOutliers(int meanK, double mulTresh)
{
//    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
//    sor.setInputCloud(cloud);
//    sor.setMeanK(meanK);
//    sor.setStddevMulThresh(mulTresh);
//    sor.filter(*cloud);
}

void pclCloud::computeNormals()
{
//    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
//    ne.setInputCloud(cloud);
//    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
//    ne.setSearchMethod (tree);
//    ne.setRadiusSearch (0.03);
//    ne.compute (*normals_cloud);
}



void pclCloud::creteMesh(int kSearch)
{
//      pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
//      copyPointCloud(cloud, *tmp_cloud);

//      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
//      normal_estimation.setInputCloud(tmp_cloud);

//      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
//      normal_estimation.setSearchMethod(tree);

//      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
//      normal_estimation.setRadiusSearch(0.03);

//      normal_estimation.compute(*cloud_normals);
//      pcl::io::savePCDFile("cloud_normals.pcd",*cloud_normals);
//      pcl::io::savePLYFile("cloud_normals.ply",*cloud_normals);
}

pclCloud::~pclCloud()
{
}

