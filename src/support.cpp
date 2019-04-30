#include "support.h"

support::support(QObject *parent) : QObject(parent)
{

}

void support::camera2framesDataTransfer()
{
    for(unsigned long id=0;id<static_cast<unsigned long>(this->connectedCameras());id++)
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

        cam_frames[0][id].emplace(tmp_cam_frames);
        if( cam_frames[0][id].size()>7)
                cam_frames[0][id].pop();
    }
}

void support::saveSequence()
{
    std::vector<Camera::camera_frames> frames;

    for(auto i=0; i<8;i++)
    {
        frames.push_back(connected_cams[0]->getFrames());

        std::cout<<i;
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
