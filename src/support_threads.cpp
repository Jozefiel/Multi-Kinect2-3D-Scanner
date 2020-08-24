#include "support.h"

//CAMERA_SUPPORT
void support::cameraInit()
{
    this->kinectInit();
    this->realsenseInit();

    cam_frames = new std::vector<std::vector<Camera::camera_frames>>(connected_cams.size());

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

//THREADS

void support::threadsInit()
{
    this->threadCameraSnapping();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    this->threadFrameUpdater();
}

void support::threadCameraSnapping()
{

    for (ulong cam_threads_counter=0; cam_threads_counter<this->connectedCameras(); cam_threads_counter++)  //run threads with frames function, snapping RGB, Depth, Ir for Kinect
    {
        cam_threads.push_back(std::thread(&Camera::frames,this->cameras()[cam_threads_counter],std::ref(snap_running)));
    }

    for (ulong cam_threads_counter=0; cam_threads_counter<this->connectedCameras(); cam_threads_counter++)     //detach threads
    {
        cam_threads[cam_threads_counter].detach();
    }
    std::cout<<"support::threadCameraSnapping started"<<std::endl;
}

void support::threadFrameUpdater()
{
   // viewer_threads.push_back(std::thread(&support::viewerUpdater,this,std::ref(snap_running)));
   //viewer_threads.push_back(std::thread(&support::pclUpdater,this,std::ref(snap_running)));
    viewer_threads.push_back(std::thread(&support::frameUpdater,this,std::ref(snap_running)));

    for (ulong viewer_threads_counter=0; viewer_threads_counter < this->viewer_threads.size(); viewer_threads_counter++)     //detach threads
    {
        viewer_threads[viewer_threads_counter].detach();
    }
}

bool support::camera2framesDataTransfer()
{
    bool returner=false;
    for(unsigned long id=0;id<this->connectedCameras();id++)
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

            if(this->cameras()[id]->lockFrames(mutex_lock_time))
            {
                cam_frames[0][id].emplace_back(tmp_cam_frames);

                this->cameras()[id]->unlockFrames();
            }

            if( static_cast<int>(cam_frames[0][id].size())>globalSettings.operator->()->getBufferSize())
            {
                cam_frames[0][id].erase(cam_frames[0][id].begin());
                returner=true;
            }
        }
    }
    return returner;
}

bool support::framesClouds2pclDataTransfer()
{
    bool returner=false;
    for(ulong id=0; id< connected_cams.size();id++)
    {
        if(this->cameras()[id]->lockFrames(5))
        {
            auto _cam_frames = cam_frames;
            if(!_cam_frames->at(id).empty())
                clouds[0]->pclCopyCloud(_cam_frames->at(id).back().cloud,static_cast<int>(id));
            this->cameras()[id]->unlockFrames();
            returner=true;
        }
    }
    return returner;
}

void support::closeThreads()
{
    snap_running=false;
    cloud_threads.clear();
    cam_threads.clear();
    viewer_threads.clear();
}
