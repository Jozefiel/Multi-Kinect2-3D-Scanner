#include "support.h"

support::support(QObject *parent) : QObject(parent)
{

}

//CAMERA_SUPPORT
void support::cameraInit()
{
    this->kinectInit();
    this->realsenseInit();
}

void support::kinectInit()
{
    libfreenect2::setGlobalLogger(nullptr);
    libfreenect2::Freenect2 * pFreenect2 = new libfreenect2::Freenect2;             // freenect2 init
    int connected_kinects = pFreenect2->enumerateDevices();                         // number of connected kinects
    for(auto id=0;id<connected_kinects;id++)                                        // add connected kinects to vector cams
    {
        connected_cams.push_back(Camera::create_camera(pFreenect2, id));
    }
    std::cout<<"support::kinectInit started"<<std::endl;

}

void support::realsenseInit()
{

    std::cout<<"support::realsenseInit started"<<std::endl;
}

std::vector<Camera*> support::cameras()
{
    return connected_cams;
}

int support::connectedCameras()
{
 return static_cast<int>(connected_cams.size());
}

//THREADS

void support::threadsInit()
{
    this->threadCameraSnapping();
    this->threadComputePointCloud();
    this->threadFrameUpdater();
}

void support::threadCameraSnapping()
{

    for (auto cam_threads_counter=0; cam_threads_counter<this->connectedCameras(); cam_threads_counter++)  //run threads with frames function, snapping RGB, Depth, Ir for Kinect
    {
        cam_threads.push_back(std::thread(&Camera::frames,this->cameras()[cam_threads_counter],std::ref(snap_running)));
    }

    for (auto cam_threads_counter=0; cam_threads_counter<this->connectedCameras(); cam_threads_counter++)     //detach threads
    {
        cam_threads[cam_threads_counter].detach();
    }
    std::cout<<"support::threadCameraSnapping started"<<std::endl;
}

void support::threadComputePointCloud()
{
    for (auto cloud_threads_counter=0; cloud_threads_counter<this->connectedCameras(); cloud_threads_counter++)  //run threads with frames function, snapping RGB, Depth, Ir for Kinect
    {
        cloud_threads.push_back(std::thread(&Camera::cloudData,this->cameras()[cloud_threads_counter],std::ref(snap_running),std::ref(compute_cloud_style)));
    }

    for (auto cloud_threads_counter=0; cloud_threads_counter<this->connectedCameras(); cloud_threads_counter++)     //detach threads
    {
        cloud_threads[cloud_threads_counter].detach();
    }
    std::cout<<"support::threadComputePointCloud started"<<std::endl;
}

void support::threadFrameUpdater()
{
    viewer_threads.push_back(std::thread(&support::viewerUpdater,this,std::ref(snap_running)));
    viewer_threads[0].detach();
}

void support::closeThreads()
{
    snap_running=false;
    cloud_threads.clear();
    cam_threads.clear();
    viewer_threads.clear();
}

//CLOUD

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

void support::camera2cloudDataTransfer()
{
    for(auto id=0;id<this->connectedCameras();id++)
    {
        if(connected_cams[id]->lockCloud(mutexTimeDelay))
        {
            clouds[id].pclCopyCloud(connected_cams[id]->getCloudData());
            connected_cams[id]->unlockCloud();
        } else
            std::cout<<"support::camera2cloudDataTransfer error: cloud was not transfered"<<std::endl;
    }
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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr support::getCloudData(int id)
{
    return clouds[id].getCloud();
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr support::getTransformedCloudData(int id)
{
    return clouds[id].getTransformedCloud();
}

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> support::mergeClouds(bool transformed)
{
//    merged_clouds.clear();
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> temp_clouds;

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

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> support::mergeClouds(bool transformed, std::vector<Eigen::Matrix4d> transform_matrix)
{
//    merged_clouds.clear();
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> temp_clouds;

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

//viewer

void support::viewerUpdater(std::atomic<bool> &snap_running)
{
    while(snap_running)
    {
//        for(auto connected_cams_number=0;connected_cams_number<connected_cams.size();connected_cams_number++)
//        {
                cv::Mat tmpIR;
                connected_cams[0]->getIR().convertTo(tmpIR,CV_8UC1);

                cv::Mat tmpDepth;
                connected_cams[0]->getDepth().convertTo(tmpDepth,CV_8UC1);

//                cv::imshow("Depth_Image" + IntToStr(connected_cams[connected_cams_number]->getId()),(connected_cams[connected_cams_number]->getDepth() / imshow_32to8));
//                cv::imshow("RGB_Image" + IntToStr(connected_cams[connected_cams_number]->getId()),(connected_cams[connected_cams_number]->getRGB()));
//                cv::imshow("IR_Image" + IntToStr(connected_cams[connected_cams_number]->getId()),tmpIR);
//                cv::imshow("RGBD_Image" + IntToStr(connected_cams[connected_cams_number]->getId()),(connected_cams[connected_cams_number]->getRGBD() ));
//                cv::imshow("Mask_Image" + IntToStr(connected_cams[connected_cams_number]->getId()),(connected_cams[connected_cams_number]->getMask() ));
//            cv::Mat frame;
//            connected_cams[0]->getRGBD().copyTo(frame);
            emit newRGBD(QPixmap::fromImage(QImage(connected_cams[0]->getRGBD().data,connected_cams[0]->getRGBD().cols,connected_cams[0]->getRGBD().rows,connected_cams[0]->getRGBD().step,QImage::Format_RGBA8888).rgbSwapped()),0);
            emit newDepth(QPixmap::fromImage(QImage(tmpDepth.data,tmpDepth.cols,tmpDepth.rows,tmpDepth.step,QImage::Format_Grayscale8).scaled(0,255)),0);
            emit newIR(QPixmap::fromImage(QImage(tmpIR.data,tmpIR.cols,tmpIR.rows,tmpIR.step,QImage::Format_Indexed8).scaled(0,255)),0);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
       //}
    }
}

//support

std::string support::IntToStr(int n)
{
    std::stringstream result;
    result << n;
    return result.str();
}


