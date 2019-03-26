#include "support.h"

support::support(QObject *parent) : QObject(parent)
{

}

void support::camera2framesDataTransfer()
{
    for(auto id=0;id<this->connectedCameras();id++)
    {
//        if(connected_cams[id]->lockCloud(mutexTimeDelay))
//        {
            cam_frames[0][id].push(connected_cams[id]->getFrames());
//        } else
 //           std::cout<<"support::camera2framesDataTransfer error: cloud was not transfered"<<std::endl;
//        if( cam_frames[0][id].size()>7)
//                cam_frames[0][id].pop();
    }
    std::cout<<cam_frames->at(0).size()<<std::endl;
    cv::imshow("testik0",*cam_frames->at(0).front().depthMat);
    cv::imshow("testik1",*cam_frames->at(0).back().depthMat);
    cv::imshow("testik2",*cam_frames->at(2).back().depthMat);
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
