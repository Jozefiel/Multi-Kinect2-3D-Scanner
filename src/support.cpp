#include "support.h"

support::support(QObject *parent) : QObject(parent)
{

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

void support::changeComputeStyle(int state)
{
    if(state==0)
        compute_cloud_style=false;
    else
        compute_cloud_style=true;

}

void support::saveData()
{
    auto tmp_cam_frame=cam_frames;

    time_t acc_time = time(nullptr);
    std::tm* now = std::localtime(&acc_time);

    std::string path = "output/tmp/" + IntToStr(now->tm_year + 1900)+'_'+IntToStr(now->tm_mon + 1)+'_'+IntToStr(now->tm_mday)+'_'+IntToStr(now->tm_hour)+'_'+IntToStr(now->tm_min)+'_'+IntToStr(now->tm_sec);
    if(this->createDirectory(path))
    {
        for(unsigned long i=0; i<tmp_cam_frame->size();i++)
        {
            for(unsigned long j=0; j<tmp_cam_frame->at(0).size();j++)
            {
                cv::imwrite(path + "/depth_" + IntToStr(i) + "_" + IntToStr(j) + ".hdr",tmp_cam_frame->at(i).at(j).rangedDepthMat);
                cv::imwrite(path + "/rgbd" + IntToStr(i) + "_" + IntToStr(j) + ".jpeg",tmp_cam_frame->at(i).at(j).rangedRGBDMat);
//                pcl::io::savePLYFileBinary(path + "/cloud" + IntToStr(i) + "_" + IntToStr(j) + ".ply" ,tmp_cam_frame->at(i).at(j).cloud);
            }
        }
    }
}


bool support::createDirectory(std::string path)
{
    fs::file_status s = fs::file_status{};

    if(fs::status_known(s) ? fs::exists(s) : fs::exists(path))
    {
        return false;
    }
    else
    {
        fs::create_directories(path);
        return true;
    }
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
