#include "support.h"

//CAMERA_SUPPORT
void support::cameraInit()
{
    this->kinectInit();
    this->realsenseInit();

    cam_frames = new std::vector<std::queue<Camera::camera_frames>>(connected_cams.size());
    counter_frame = new std::vector<std::queue<int>>(connected_cams.size());

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
    viewer_threads.push_back(std::thread(&support::frameUpdater,this,std::ref(snap_running)));


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
