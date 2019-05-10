#include "support.h"

//CAMERA_SUPPORT
void support::cameraInit()
{
    this->kinectInit();
    this->realsenseInit();

    cam_frames = new std::vector<std::vector<Camera::camera_frames>>(connected_cams.size());
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
//    this->threadComputePointCloud();
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

void support::threadFrameUpdater()
{
 //   viewer_threads.push_back(std::thread(&support::viewerUpdater,this,std::ref(snap_running)));
 //  viewer_threads.push_back(std::thread(&support::pclUpdater,this,std::ref(snap_running)));
    viewer_threads.push_back(std::thread(&support::frameUpdater,this,std::ref(snap_running)));

    for (auto viewer_threads_counter=0; viewer_threads_counter<this->viewer_threads.size(); viewer_threads_counter++)     //detach threads
    {
        viewer_threads[viewer_threads_counter].detach();
    }
}

void support::camera2framesDataTransfer()
{
    for(unsigned long id=0;id<static_cast<unsigned long>(this->connectedCameras());id++)
    {
        if(connected_cams[id]->getFramesReleasedCheck())
        {
            Camera::camera_frames tmp_cam_frames;

            tmp_cam_frames.depthMat= connected_cams[id]->getFrames().depthMat.clone();
            tmp_cam_frames.rgbdMat= connected_cams[id]->getFrames().rgbdMat.clone();
            tmp_cam_frames.colorMat= connected_cams[id]->getFrames().colorMat.clone();
            tmp_cam_frames.irMat= connected_cams[id]->getFrames().irMat.clone();
            tmp_cam_frames.histMat= connected_cams[id]->getFrames().histMat.clone();
            tmp_cam_frames.mask= connected_cams[id]->getFrames().mask.clone();
            tmp_cam_frames.rangedRGBDMat= connected_cams[id]->getFrames().rangedRGBDMat.clone();
            tmp_cam_frames.rangedDepthMat= connected_cams[id]->getFrames().rangedDepthMat.clone();
            tmp_cam_frames.cloud = connected_cams[id]->getFrames().cloud;

            connected_cams[id]->resetFramesReleased();

            cam_frames[0][id].emplace_back(tmp_cam_frames);
            if( cam_frames[0][id].size()>7)
            {
                cam_frames[0][id].erase(cam_frames[0][id].begin());
            }
        }

    }
}


void support::closeThreads()
{
    snap_running=false;
    cloud_threads.clear();
    cam_threads.clear();
    viewer_threads.clear();
}
