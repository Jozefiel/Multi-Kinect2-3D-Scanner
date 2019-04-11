#include "pcl.h"

pclCloud::pclCloud(int cam_id, std::string cam_serial)
{
    id=cam_id;
    serial=cam_serial;
    try {
        cout << "pclCloud::pclCloud: config file exist. calibration setted "<< serial<<endl;

        boost::property_tree::ini_parser::read_ini("config/"+cam_serial+".ini", pt);

        transform_matrix (0,0) = pt.get<float>("Calibration.transform_00");
        transform_matrix (0,1) = pt.get<float>("Calibration.transform_01");
        transform_matrix (0,2) = pt.get<float>("Calibration.transform_02");
        transform_matrix (0,3) = pt.get<float>("Calibration.transform_03");
        transform_matrix (1,0) = pt.get<float>("Calibration.transform_10");
        transform_matrix (1,1) = pt.get<float>("Calibration.transform_11");
        transform_matrix (1,2) = pt.get<float>("Calibration.transform_12");
        transform_matrix (1,3) = pt.get<float>("Calibration.transform_13");
        transform_matrix (2,0) = pt.get<float>("Calibration.transform_20");
        transform_matrix (2,1) = pt.get<float>("Calibration.transform_21");
        transform_matrix (2,2) = pt.get<float>("Calibration.transform_22");
        transform_matrix (2,3) = pt.get<float>("Calibration.transform_23");
        transform_matrix (3,0) = pt.get<float>("Calibration.transform_30");
        transform_matrix (3,1) = pt.get<float>("Calibration.transform_31");
        transform_matrix (3,2) = pt.get<float>("Calibration.transform_32");
        transform_matrix (3,3) = pt.get<float>("Calibration.transform_33");

    } catch (int e) {
        cout << "No init config found for: "<< serial << e <<endl;
        transform_matrix (0,0) = 1;
        transform_matrix (0,1) = 0;
        transform_matrix (0,2) = 0;
        transform_matrix (0,3) = 0;
        transform_matrix (1,0) = 0;
        transform_matrix (1,1) = 1;
        transform_matrix (1,2) = 0;
        transform_matrix (1,3) = 0;
        transform_matrix (2,0) = 0;
        transform_matrix (2,1) = 0;
        transform_matrix (2,2) = 1;
        transform_matrix (2,3) = 0;
        transform_matrix (3,0) = 0;
        transform_matrix (3,1) = 0;
        transform_matrix (3,2) = 0;
        transform_matrix (3,3) = 1;
    }

}

pclCloud::pclCloud(int cam_id)
{
    cout << "pclCloud::pclCloud: created cloud without calibration parameters "<< serial<<endl;
    id=cam_id;
}

void pclCloud::pclCopyCloud(pcl::PointCloud<pcl::PointXYZRGB> cam_cloud)
{
    if(!cam_cloud.empty())
    {
        cloud.clear();
        pcl::copyPointCloud(cam_cloud,cloud);
    }
    else
    {
        std::cout<<"empty cloud: "<<this->id<<std::endl;
    }
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
    if(!cloud.empty())
    {
        pcl::transformPointCloud(cloud,transformed_cloud,transform_matrix,true);
    }
    else
    {
        std::cout<<"cloud wasn't transformed"<<std::endl;
    }
}

void pclCloud::transformPointCloud(Eigen::Matrix4d transform)
{
    if(!cloud.empty())
    {
        pcl::transformPointCloud(cloud,transformed_cloud,transform,true);
    }
    else
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

void pclCloud::mergeClouds(std::vector<pcl::PointCloud<pcl::PointXYZRGB>> clouds)
{
    cloud.clear();
    for(auto i=0;i<clouds.size();i++)
    {
        if(!clouds[i].empty())
            cloud+=clouds[i];
    }
}

void pclCloud::creteMesh(int kSearch)
{
      pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
      copyPointCloud(cloud, *tmp_cloud);

      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
      normal_estimation.setInputCloud(tmp_cloud);

      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
      normal_estimation.setSearchMethod(tree);

      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
      normal_estimation.setRadiusSearch(0.03);

      normal_estimation.compute(*cloud_normals);
      pcl::io::savePCDFile("cloud_normals.pcd",*cloud_normals);
      pcl::io::savePLYFile("cloud_normals.ply",*cloud_normals);
}

pclCloud::~pclCloud()
{
}

