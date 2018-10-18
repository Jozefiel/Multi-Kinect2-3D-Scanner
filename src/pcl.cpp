#include "pcl.h"

pclCloud::pclCloud(int cam_id, std::string cam_serial)
{
    id=cam_id;
    serial=cam_serial;
    try {

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
    if(!cloud->empty())
    {
        pcl::transformPointCloud(*cloud,*transformed_cloud,transform_matrix,true);
    }
    else
    {
        std::cout<<"cloud wasn't transformed"<<std::endl;
    }
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
        if(!clouds[i]->empty())
            *cloud+=*clouds[i];
    }
}

void pclCloud::creteMesh(int kSearch)
{
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        copyPointCloud(*cloud, *tmp_cloud);

      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
      pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
      tree->setInputCloud(tmp_cloud);
      n.setSearchMethod (tree);
      n.setKSearch (kSearch);
      n.compute (*normals);



      pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
      pcl::concatenateFields (*tmp_cloud, *normals, *cloud_with_normals);

      pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
      tree2->setInputCloud (cloud_with_normals);

      pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
      pcl::PolygonMesh triangles;

      gp3.setSearchRadius (0.025);

      gp3.setMu (2.5);
      gp3.setMaximumNearestNeighbors (100);
      gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
      gp3.setMinimumAngle(M_PI/18); // 10 degrees
      gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
      gp3.setNormalConsistency(false);

      gp3.setInputCloud (cloud_with_normals);
      gp3.setSearchMethod (tree2);
      gp3.reconstruct (triangles);
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
    if(view_ports==0)
    {
        std::cout<<"No viewport"<<std::endl;
    }
    else
    {
        double scale_size=(0.99/view_ports);
        for(int i=0;i<view_ports;i++)
        {
            int viewPortId (0);
            viewPortsId.push_back(viewPortId);
            viewer->createViewPort (scale_size*i,  0.0, scale_size+scale_size*i, 1.0, viewPortsId[i]);
        }
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
       // viewer->removePointCloud("cloud0");
        viewer->updatePointCloud(cloud0,"cloud0");
      //  viewer->removePointCloud("cloud1");
        viewer->updatePointCloud(cloud1,"cloud1");
      //  viewer->removePointCloud("cloud2");
        viewer->updatePointCloud(cloud2,"cloud2");
    }
}


void pclViewer::pclAddCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloud, int cam_id)
{
    std::stringstream id_string;
    id_string << cam_id;
    if(!pCloud->empty())
    {
        if(cam_id==0)
        {
            if(!viewer->updatePointCloud(pCloud,"cloud_"+id_string.str()))
            {
                viewer->removePointCloud("cloud_"+id_string.str());
                viewer->addPointCloud(pCloud,"cloud_"+id_string.str());
            }
        }
        else
        {
            viewer->removePointCloud("cloud_"+id_string.str());
            viewer->addPointCloud(pCloud,"cloud_"+id_string.str(),viewPortsId[cam_id]);
        }
    }
}

void pclViewer::spinOnce()
{
    viewer->spinOnce();
}

pclViewer::~pclViewer()
{
}
