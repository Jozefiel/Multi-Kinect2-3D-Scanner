#include "support.h"

support::support(QObject *parent) : QObject(parent)
{

}

void support::camera2framesDataTransfer()
{
    for(auto id=0;id<this->connectedCameras();id++)
    {
        Camera::camera_frames tmp_cam_frames;
        tmp_cam_frames.depthMat= connected_cams[id]->getFrames().depthMat.clone();


//        memcpy(static_cast<void*>(&connected_cams[id]->getFrames()),static_cast<void*>(&tmp_cam_frames),sizeof(tmp_cam_frames));
        cam_frames[0][id].emplace(tmp_cam_frames);
//        } else
 //           std::cout<<"support::camera2framesDataTransfer error: cloud was not transfered"<<std::endl;
        if( cam_frames[0][id].size()>7)
                cam_frames[0][id].pop();

        counter++;
    }
//    std::cout<<"size: " <<counter_frame->at(0).size()<<std::endl;
//    std::cout<< "back: " << counter_frame->at(0).back()<<std::endl;
//    std::cout<< "front: " << counter_frame->at(0).front()<<std::endl;

    cv::imshow("testik1",cam_frames->at(0).back().depthMat);
//    std::this_thread::sleep_for(std::chrono::milliseconds(10));
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
