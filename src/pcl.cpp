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

    if(this->lockCloud(15))
    {
        camera_clouds[static_cast<ulong>(camId)].emplace_back(tmp_cloud);

        if( camera_clouds[static_cast<ulong>(camId)].size() > static_cast<ulong>(globalSettings.operator->()->getBufferSize()))
        {
            camera_clouds[_camId].erase(camera_clouds[_camId].begin());
        }
        this->unlockCloud();
    }
}

bool pclCloud::mergeClouds(int camId)
{
    try {
        if(this->lockCloud(15))
        {
            ulong _camId=static_cast<ulong>(camId);
            pcl::PointCloud<pcl::PointXYZRGB> cloud ;
            for(ulong i=0; i<camera_clouds[_camId].size();i++)
            {
                if(!camera_clouds[_camId].at(i).empty())
                    cloud+=camera_clouds[_camId].at(i);
            }
            this->unlockCloud();
            std::cout<<"Merged cloud size: " << cloud.points.size()<<" , id: " << _camId <<std::endl;
        }
    } catch (...)
    {
        std::cout<<this->id<<" Problem with  pclCloud::mergeClouds"<<std::endl;
    }
}

bool pclCloud::mergeAllClouds()
{
    try {
        merged_clouds_viewer->clear();
        for(ulong i=0; i<camera_clouds.size();i++)
        {
            for(ulong j=0; j<camera_clouds[i].size();j++)
            {
                if(!camera_clouds[i].at(j).empty())
                {
                    *merged_clouds_viewer+=camera_clouds[i].at(j);
                }
            }
        }
        std::cout<<"Merged cloud size: " << merged_clouds_viewer->points.size()<<std::endl;
        return true;
    } catch (...)
    {
        std::cout<<" Problem with  pclCloud::mergeClouds"<<std::endl;
        return false;
    }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud::getMergedCloud()
{
    try {

            return merged_clouds_viewer;
            this->unlockCloud();
    } catch (...)
    {
        std::cout<<" Problem with pclCloud::getMergedCloud"<<std::endl;
    }
}


bool pclCloud::mergeLastClouds()
{
    try {
        merged_clouds_viewer->clear();
        for(ulong i=0; i<camera_clouds.size();i++)
        {
            if(camera_clouds.at(i).size()>0)
            {
                if(camera_clouds.at(i).back().size()>0)
                {
                    *merged_clouds_viewer+=camera_clouds.at(i).back();
                }
            }
        }
        std::cout<<"Merged cloud size: " << merged_clouds_viewer->points.size()<<std::endl;
        return true;
    } catch (...)
    {
        std::cout<<" Problem with pclCloud::getMergedCloud"<<std::endl;
    }
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

bool pclCloud::lockCloud(int lock_time)
{
    if(cloud_mutex.try_lock_for(std::chrono::milliseconds(lock_time)))
        return true;
    else
        return false;
}

void pclCloud::unlockCloud()
{
    cloud_mutex.unlock();
}

pclCloud::~pclCloud()
{
}

