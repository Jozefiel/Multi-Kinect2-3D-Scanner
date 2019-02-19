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
    viewer_threads.push_back(std::thread(&support::pclUpdater,this,std::ref(snap_running)));

    for (auto viewer_threads_counter=0; viewer_threads_counter<this->viewer_threads.size(); viewer_threads_counter++)     //detach threads
    {
        viewer_threads[viewer_threads_counter].detach();
    }
}

void support::closeThreads()
{
    snap_running=false;
    cloud_threads.clear();
    cam_threads.clear();
    viewer_threads.clear();
}

//CLOUD


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
    cv::Mat tmpIR;
    cv::Mat tmpDepth;
    cv::Mat tmpHist;
    cv::Mat tmpRGBD;
    while(snap_running)
    {
        for(auto connected_cams_number=0;connected_cams_number<connected_cams.size();connected_cams_number++)
        {
            tmpDepth=connected_cams[connected_cams_number]->getDepth()/8;
            tmpDepth.convertTo(tmpDepth,CV_8UC1);

            tmpIR=connected_cams[connected_cams_number]->getIR()/64;
            tmpIR.convertTo(tmpIR, CV_8UC1);

            tmpRGBD=connected_cams[connected_cams_number]->getRGBD();
            cv::cvtColor(tmpRGBD,tmpRGBD,CV_RGBA2RGB);

            emit newRGBD(QPixmap::fromImage(QImage(tmpRGBD.data,tmpRGBD.cols,tmpRGBD.rows,tmpRGBD.step,QImage::Format_RGB888).rgbSwapped()),connected_cams_number);
            emit newDepth(QPixmap::fromImage(QImage(tmpDepth.data,tmpDepth.cols,tmpDepth.rows,tmpDepth.step,QImage::Format_Indexed8)),connected_cams_number);
            emit newIR(QPixmap::fromImage(QImage(tmpIR.data,tmpIR.cols,tmpIR.rows,tmpIR.step,QImage::Format_Indexed8)),connected_cams_number);
            if(!connected_cams[connected_cams_number]->getRangedRGBD().empty())
            {
                emit newRangedRGBD(QPixmap::fromImage(QImage(connected_cams[connected_cams_number]->getRangedRGBD().data,connected_cams[connected_cams_number]->getRangedRGBD().cols,connected_cams[connected_cams_number]->getRangedRGBD().rows,connected_cams[connected_cams_number]->getRangedRGBD().step,QImage::Format_RGB888).rgbSwapped()),connected_cams_number);
            }
            if(!connected_cams[connected_cams_number]->getRangedDepth().empty())
            {
                tmpDepth=connected_cams[connected_cams_number]->getRangedDepth()/8;
                tmpDepth.convertTo(tmpDepth,CV_8UC1);
                emit newRangedDepth(QPixmap::fromImage(QImage(tmpDepth.data,tmpDepth.cols,tmpDepth.rows,tmpDepth.step,QImage::Format_Indexed8)),connected_cams_number);
            }
            if(!connected_cams[connected_cams_number]->getHistogram().empty())
            {
                tmpHist=connected_cams[connected_cams_number]->getHistogram();
                tmpHist.convertTo(tmpHist, CV_8UC3);
                emit newHist(QPixmap::fromImage(QImage(tmpHist.data,tmpHist.cols,tmpHist.rows,tmpHist.step,QImage::Format_RGB888)),connected_cams_number);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
       }
    }
}

void support::pclUpdater(std::atomic<bool> &snap_running)
{
    merged_cloud= new pclCloud(4);
    while(snap_running)
    {
        this->camera2cloudDataTransfer();   // store cloud to pcl objects
        merged_cloud->mergeClouds(this->mergeClouds(false));//! error when true, random fallings

        if(!merged_cloud->getCloud()->empty())
            emit newCloud();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

    }
}


//support

std::string support::IntToStr(int n)
{
    std::stringstream result;
    result << n;
    return result.str();
}

void support::changeComputeStyle(int state)
{
    if(state==0)
        compute_cloud_style=false;
    else
        compute_cloud_style=true;

}

void support::saveLUT(cv::Mat depth, cv::Mat rgbd, std::string filename,int counter)
{
#define ir_depth_width 512
#define ir_depth_height 424
    int i,j;

    cv::imwrite("test/"+filename+"_"+IntToStr(counter)+".jpeg",rgbd);

    ofstream myfile;
    myfile.open("test/"+filename+"_"+IntToStr(counter)+".txt");

    for(i = 0; i < depth.cols; i++)
    {
        for(j = 0; j < depth.rows; j++)
        {
            double orig = depth.at<float>(j,i);
            myfile<<orig<<"\n";

        }
    }

    myfile.close();

}
