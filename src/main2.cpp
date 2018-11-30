#include <iostream>
#include <string>
#include <thread>
#include <memory>
#include <chrono>
#include <fstream>

#include "camera.h"
#include "kinect.h"
#include "support.h"
#include "pcl.h"


#define imshow_32to8 2048

void frames_show(std::vector<Camera*> connected_cams, pclCloud merged_cloud, std::atomic<bool> & snap_running);
void saveLUT(cv::Mat in, cv::Mat rgbd, std::string filename, int counter);

int transform_bool=true;

int main1()
{

    support support;

    support.cameraInit();                                       //! camera init, create connection with Kinects and Realsenses
    support.threadsInit();                                      //! threads init, detached snapping and cloud computing started
    support.cloudInit();                                        //! create point cloud for each camera


    pclViewer rotationViewer(0,"Rotation 3D Scan");             //! create cloud viewer for all cloud connected together
    pclCloud merged_cloud(4);                                   //! cloud for connected clouds from all cameras

    std::atomic<bool> snap_running {true};

    std::thread im_shower(frames_show,support.cameras(),merged_cloud,std::ref(snap_running));
//    im_shower.detach();

    Eigen::Matrix4d transform_0 = support.getClouds()[0].getTransformationMatrix();
    Eigen::Matrix4d transform_1 = support.getClouds()[1].getTransformationMatrix();
    Eigen::Matrix4d transform_2 = support.getClouds()[2].getTransformationMatrix();

    std::chrono::system_clock::time_point start, stop;
    while(snap_running)
    {

        support.getClouds()[0].setTransformationMatrix(transform_0);
        support.getClouds()[1].setTransformationMatrix(transform_1);
        support.getClouds()[2].setTransformationMatrix(transform_2);

        std::vector<Eigen::Matrix4d> transform_matrix;
        transform_matrix.push_back(transform_0);
        transform_matrix.push_back(transform_1);
        transform_matrix.push_back(transform_2);

        support.camera2cloudDataTransfer();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        merged_cloud.mergeClouds(support.mergeClouds(false));           //! error when true, random fallings

//        start=std::chrono::system_clock::now();

        if(!merged_cloud.getCloud()->empty())
            rotationViewer.pclAddCloud(merged_cloud.getCloud(),0);

        rotationViewer.spinOnce();

//       stop=std::chrono::system_clock::now();
  //     std::cout << "Execution Time last:" << std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count() << " ms" << std::endl;

    }

    return 0;
}

void frames_show(std::vector<Camera*> connected_cams, pclCloud merged_cloud, std::atomic<bool> & snap_running)
{
    std::chrono::system_clock::time_point start, stop;
    int saved_frame_counter=0;

//    while(snap_running)
//    {
//        start=std::chrono::system_clock::now();
//       // min_max_cloud();


//        for(auto connected_cams_number=0;connected_cams_number<connected_cams.size();connected_cams_number++)
//        {
//                 cv::Mat tmpIR;
//                 connected_cams[connected_cams_number]->getIR().convertTo(tmpIR,CV_16UC1);
//                cv::imshow("Depth_Image" + IntToStr(connected_cams[connected_cams_number]->getId()),(connected_cams[connected_cams_number]->getDepth() / imshow_32to8));
//    //            cv::imshow("RGB_Image" + IntToStr(connected_cams[connected_cams_number]->getId()),(connected_cams[connected_cams_number]->getRGB()));
//    //            cv::imshow("IR_Image" + IntToStr(connected_cams[connected_cams_number]->getId()),tmpIR);
//                cv::imshow("RGBD_Image" + IntToStr(connected_cams[connected_cams_number]->getId()),(connected_cams[connected_cams_number]->getRGBD() ));
//    //            cv::imshow("Mask_Image" + IntToStr(connected_cams[connected_cams_number]->getId()),(connected_cams[connected_cams_number]->getMask() ));

//                if(!connected_cams[connected_cams_number]->getRangedRGBD().empty())
//                {
// //                   cv::imshow("croped_RGBD_Image" + IntToStr(connected_cams[connected_cams_number]->getId()),(connected_cams[connected_cams_number]->getRangedRGBD() ));
//                }
//                if(!connected_cams[connected_cams_number]->getRangedDepth().empty())
//                {
// //                   cv::imshow("croped_Depth_Image" + IntToStr(connected_cams[connected_cams_number]->getId()),(connected_cams[connected_cams_number]->getRangedDepth() / imshow_32to8 ));
//                }


//            cv::moveWindow("Depth_Image" + IntToStr(connected_cams[connected_cams_number]->getId()),ir_depth_width*connected_cams_number+1920,0);
//            cv::moveWindow("RGBD_Image" + IntToStr(connected_cams[connected_cams_number]->getId()),ir_depth_width*connected_cams_number+1920,ir_depth_height);
//            cv::moveWindow("IR_Image" + IntToStr(connected_cams[connected_cams_number]->getId()),ir_depth_width*connected_cams_number,ir_depth_height*2);


//        }

//        auto key = cv::waitKey(15);
//        switch( (char) key)
//        {
//            case 'Q':
//            case 'q':
//            case 27 :
//                snap_running=false;
//                break;

//            case 'w':
//            case 'W':
//                for(auto i=0;i<connected_cams.size();i++)
//                {
//                    cv::Mat tmpIR;
//                    connected_cams[i]->getIR().convertTo(tmpIR,CV_8UC1,255,0);
//                    cv::imwrite("output/"+IntToStr(connected_cams[i]->getId())+"/RGBD/RGBD_" +IntToStr(saved_frame_counter)+".jpeg",connected_cams[i]->getRGBD());
//                    cv::imwrite("output/"+IntToStr(connected_cams[i]->getId())+"/DEPTH/DEPTH_" +IntToStr(saved_frame_counter)+".jpeg",connected_cams[i]->getDepth() );
//                    cv::imwrite("output/"+IntToStr(connected_cams[i]->getId())+"/RGB/RGB_" +IntToStr(saved_frame_counter)+".jpeg",connected_cams[i]->getRGB());
//                    cv::imwrite("output/"+IntToStr(connected_cams[i]->getId())+"/IR/IR_" +IntToStr(saved_frame_counter)+".jpeg",connected_cams[i]->getIR() / 64  );
//                    saveLUT(connected_cams[i]->getDepth(),connected_cams[i]->getRGBD(),IntToStr(connected_cams[i]->getId()),saved_frame_counter);
//                }
//                merged_cloud.removeOutliers(10,1.5);
//                pcl::io::savePLYFile("output/CLOUDS/cloud_"+IntToStr(saved_frame_counter)+".ply",*merged_cloud.getCloud());
//////                merged_cloud.creteMesh(10);
//                saved_frame_counter++;
//                break;

//            case 't':
//                break;
//            case '1':
//                connected_cams[0]->start();
//                break;
//            case '2':
//                connected_cams[1]->start();
//                break;
//            case '3':
//                connected_cams[2]->start();
//                break;
//            case '4':
//                connected_cams[0]->stop();
//                break;
//            case '5':
//                connected_cams[1]->stop();
//                break;
//            case '6':
//                connected_cams[2]->stop();
//                break;
//            case '7':
//                connected_cams[1]->stop();
//                connected_cams[2]->stop();

//                connected_cams[0]->start();
//                connected_cams[0]->stop();

//                connected_cams[1]->start();
//                connected_cams[1]->stop();

//                connected_cams[2]->start();
//                connected_cams[2]->stop();

//                break;



//         }
//        stop=std::chrono::system_clock::now();
////        cout << "ImVIEWER: " << std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count() << " ms" << endl;
//    }
}


//void saveLUT(cv::Mat in, cv::Mat rgbd, std::string filename,int counter)
//{
//#define ir_depth_width 512
//#define ir_depth_height 424
//    int i,j;

//    cv::imwrite("test/"+filename+" "+IntToStr(counter)+".jpeg",rgbd);

//    ofstream myfile;
//    myfile.open("test/"+filename+" "+IntToStr(counter)+".txt");

//    for(i = 0; i < in.cols; i++)
//    {
//        for(j = 0; j < in.rows; j++)
//        {
//            double orig = in.at<float>(j,i);
//            myfile<<orig<<"\n";

//        }
//    }

//    myfile.close();


//}

