#include <iostream>
#include <string>
#include <thread>
#include <memory>
#include <chrono>

#include "camera.h"
#include "kinect.h"
#include "pcl.h"


#define imshow_32to8 2048

std::string IntToStr(int n);
void frames_show(std::vector<Camera*> connected_cams, pclCloud merged_cloud, std::atomic<bool> & snap_running);
void matrix_rotat_koef();
void min_max_cloud();

int matrix[16]={0,0,0,0};
//int depth_control[6]={0,0,0,0,0,0};

float fmatrix[16]={0};

int main()
{

    libfreenect2::setGlobalLogger(nullptr);

    std::vector<Camera*> connected_cams;                                                 // vector of Camera objects
    std::vector<std::thread> cam_threads;                                           // vector of threads for image snapping
    std::vector<std::thread> cloud_threads;                                         // vector of threads for image snapping


//! Cameras init
//    int connected_realsenses=0;
//    static int connected_cams;

    libfreenect2::Freenect2 * pFreenect2 = new libfreenect2::Freenect2;             // freenect2 init

    int connected_kinects =pFreenect2->enumerateDevices();                          // number of connected kinects

    for(auto id=0;id<connected_kinects;id++)                                        // add connected kinects to vector cams
    {
        connected_cams.push_back(Camera::create_camera(pFreenect2, id));
    }

    /*
    //  REALSENSE INIT
    //
    */

//! Cameras init
//! Cloud init

    pclViewer cloudViewer(connected_cams.size(),"3D Scan");
    pclViewer rotationViewer(1,"Rotation 3D Scan");


    pclCloud cloud0(connected_cams[0]->getId());
    pclCloud cloud1(connected_cams[0]->getId());
    pclCloud cloud2(connected_cams[0]->getId());
    pclCloud merged_cloud(4);

//! Cloud init



//! Threads
    std::atomic<bool> snap_running {true};

    //threads for snapping images from camera
    for (auto cam_threads_counter=0; cam_threads_counter<connected_cams.size(); cam_threads_counter++)  //run threads with frames function, snapping RGB, Depth, Ir for Kinect
    {
        std::cout<<cam_threads_counter<<std::endl;
        cam_threads.push_back(std::thread(&Camera::frames,connected_cams[cam_threads_counter],std::ref(snap_running)));
    }

    for (auto cam_threads_counter=0; cam_threads_counter<cam_threads.size(); cam_threads_counter++)     //detach threads
    {
        std::cout<<cam_threads_counter<<std::endl;
        cam_threads[cam_threads_counter].detach();
    }

    //threads for computing point clouds
    for (auto cloud_threads_counter=0; cloud_threads_counter<connected_cams.size(); cloud_threads_counter++)  //run threads with frames function, snapping RGB, Depth, Ir for Kinect
    {
        std::cout<<cloud_threads_counter<<std::endl;
        cloud_threads.push_back(std::thread(&Camera::cloudData,connected_cams[cloud_threads_counter],std::ref(snap_running)));
    }

    for (auto cloud_threads_counter=0; cloud_threads_counter<cam_threads.size(); cloud_threads_counter++)     //detach threads
    {
        std::cout<<cloud_threads_counter<<std::endl;
        cloud_threads[cloud_threads_counter].detach();
    }

    std::thread im_shower(frames_show,connected_cams,merged_cloud,std::ref(snap_running));
    im_shower.detach();

//! Threads




    Eigen::Matrix4f transform_0 = Eigen::Matrix4f::Identity();

    transform_0 (0,0) = 1;
    transform_0 (0,1) = 0;
    transform_0 (0,2) = 0;
    transform_0 (0,3) = 0;
    transform_0 (1,0) = 0;
    transform_0 (1,1) = 1;
    transform_0 (1,2) = 0;
    transform_0 (1,3) = 0;
    transform_0 (2,0) = 0;
    transform_0 (2,1) = 0;
    transform_0 (2,2) = 1;
    transform_0 (2,3) = 0;
    transform_0 (3,0) = 0;
    transform_0 (3,1) = 0;
    transform_0 (3,2) = 0;
    transform_0 (3,3) = 1;

    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

    transform_1 (0,0) = 0.052762927148521;
    transform_1 (0,1) = 0.136015093117542;
    transform_1 (0,2) = -0.989300746973814;
    transform_1 (0,3) = 0.713;
    transform_1 (1,0) = -0.035140343085145;
    transform_1 (1,1) = 0.990320024285035;
    transform_1 (1,2) = 0.134281070102777;
    transform_1 (1,3) = -0.029;
    transform_1 (2,0) = 0.997988592022263;
    transform_1 (2,1) = 0.027679305343792;
    transform_1 (2,2) = 0.057031800331981;
    transform_1 (2,3) = 0.559;
    transform_1 (3,0) = 0;
    transform_1 (3,1) = 0;
    transform_1 (3,2) = 0;
    transform_1 (3,3) = 1;


    Eigen::Matrix4f transform_2 = Eigen::Matrix4f::Identity();

    transform_2 (0,0) = -0.033612424484263;
    transform_2 (0,1) = 0.049480919369063;
    transform_2 (0,2) = -0.998209318499222;
    transform_2 (0,3) = 0;
    transform_2 (1,0) = -0.134517708146897;
    transform_2 (1,1) = 0.989461724369153;
    transform_2 (1,2) = 0.053576881239273;
    transform_2 (1,3) = 0;
    transform_2 (2,0) = -0.990340946904245;
    transform_2 (2,1) = 0.136077678650149;
    transform_2 (2,2) = 0.026602147619899;
    transform_2 (2,3) = 0;
    transform_2 (3,0) = 0;
    transform_2 (3,1) = 0;
    transform_2 (3,2) = 0;
    transform_2 (3,3) = 1;


    cloud0.setTransformationMatrix(transform_0);
    cloud1.setTransformationMatrix(transform_2);
    cloud2.setTransformationMatrix(transform_1);

    std::chrono::system_clock::time_point start, stop;
    while(snap_running)
    {

        start=std::chrono::system_clock::now();

        if(connected_cams[0]->lockCloud(10))
        {
            cloud0.pclCopyCloud(connected_cams[0]->getCloudData());
            connected_cams[0]->unlockCloud();
        } else
            std::cout<<"daco je zle"<<std::endl;

        if(connected_cams[1]->lockCloud(10))
        {
            cloud1.pclCopyCloud(connected_cams[1]->getCloudData());
            connected_cams[1]->unlockCloud();
        } else
            std::cout<<"daco je zle"<<std::endl;

        if(connected_cams[2]->lockCloud(10))
        {
            cloud2.pclCopyCloud(connected_cams[2]->getCloudData());
            connected_cams[2]->unlockCloud();
        } else
            std::cout<<"daco je zle"<<std::endl;

//        cloud1.removeOutliers(10,1.0);
//        cloud1.removeOutliers(10,1.0);
//        cloud2.removeOutliers(10,1.0);

        cloudViewer.pclAddCloud(cloud0.getCloud(),0);
        cloudViewer.pclAddCloud(cloud1.getCloud(),1);
        cloudViewer.pclAddCloud(cloud2.getCloud(),2);

        cloud0.transformPointCloud();
        cloud1.transformPointCloud();
        cloud2.transformPointCloud();



        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
        clouds.push_back(cloud0.getTransformedCloud());
       // clouds.push_back(cloud1.getTransformedCloud());
        clouds.push_back(cloud2.getTransformedCloud());

        merged_cloud.mergeClouds(clouds);
        merged_cloud.removeOutliers(10,1.0);
        rotationViewer.pclAddCloud(merged_cloud.getCloud(),0);

        //cloudViewer.spinOnce();
        rotationViewer.spinOnce();

       stop=std::chrono::system_clock::now();
       std::cout << "Execution Time last:" << std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count() << " ms" << std::endl;


    }


    for (int i = 0; i < connected_cams.size(); i++)
    {
        delete connected_cams[i];
    }

    return 0;
}

void frames_show(std::vector<Camera*> connected_cams, pclCloud merged_cloud, std::atomic<bool> & snap_running)
{
    std::chrono::system_clock::time_point start, stop;
    int saved_frame_counter=0;

    while(snap_running)
    {
        start=std::chrono::system_clock::now();
        matrix_rotat_koef();
       // min_max_cloud();


        for(auto connected_cams_number=0;connected_cams_number<connected_cams.size();connected_cams_number++)
        {
//            cv::imshow("Depth_Image" + IntToStr(connected_cams[connected_cams_number]->getId()),(connected_cams[connected_cams_number]->getDepth() / imshow_32to8 ));
//            cv::imshow("RGB_Image" + IntToStr(connected_cams[connected_cams_number]->getId()),(connected_cams[connected_cams_number]->getRGB()));
//            cv::imshow("IR_Image" + IntToStr(connected_cams[connected_cams_number]->getId()),(connected_cams[connected_cams_number]->getIR() / imshow_32to8 ));
            cv::imshow("RGBD_Image" + IntToStr(connected_cams[connected_cams_number]->getId()),(connected_cams[connected_cams_number]->getRGBD() ));
        }

        auto key = cv::waitKey(15);
        switch( (char) key)
        {
            case 'Q':
            case 'q':
            case 27 :
                snap_running=false;
                break;

            case 'w':
            case 'W':
                for(auto i=0;i<connected_cams.size();i++)
                {
                    cv::imwrite("RGBD_"+IntToStr(connected_cams[i]->getId()) +"_" +IntToStr(saved_frame_counter)+".jpeg",connected_cams[i]->getRGBD());
                }
                pcl::io::savePLYFile("clouds.ply",*merged_cloud.getCloud());
                saved_frame_counter++;
                break;
         }
        stop=std::chrono::system_clock::now();
//        cout << "ImVIEWER: " << std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count() << " ms" << endl;
    }
}

void matrix_rotat_koef()
{
#define MAX_TRESHOLD 2
    namedWindow("Rotation",cv::WINDOW_AUTOSIZE);
    cv::createTrackbar( "[0,0]", "Rotation", &matrix[0], MAX_TRESHOLD);
    cv::createTrackbar( "[0,1]", "Rotation", &matrix[1], MAX_TRESHOLD);
    cv::createTrackbar( "[0,2]", "Rotation", &matrix[2], MAX_TRESHOLD);
    cv::createTrackbar( "[0,3]", "Rotation", &matrix[3], MAX_TRESHOLD);
    cv::createTrackbar( "[1,0]", "Rotation", &matrix[4], MAX_TRESHOLD);
    cv::createTrackbar( "[1,1]", "Rotation", &matrix[5], MAX_TRESHOLD);
    cv::createTrackbar( "[1,2]", "Rotation", &matrix[6], MAX_TRESHOLD);
    cv::createTrackbar( "[1,3]", "Rotation", &matrix[7], MAX_TRESHOLD);
    cv::createTrackbar( "[2,0]", "Rotation", &matrix[8], MAX_TRESHOLD);
    cv::createTrackbar( "[2,1]", "Rotation", &matrix[9], MAX_TRESHOLD);
    cv::createTrackbar( "[2,2]", "Rotation", &matrix[10], MAX_TRESHOLD);
    cv::createTrackbar( "[2,3]", "Rotation", &matrix[11], MAX_TRESHOLD);
    cv::createTrackbar( "[3,0]", "Rotation", &matrix[12], MAX_TRESHOLD);
    cv::createTrackbar( "[3,1]", "Rotation", &matrix[13], MAX_TRESHOLD);
    cv::createTrackbar( "[3,2]", "Rotation", &matrix[14], MAX_TRESHOLD);
    cv::createTrackbar( "[3,3]", "Rotation", &matrix[15], MAX_TRESHOLD);
}

//void min_max_cloud()
//{
//#define MAX_TRESHOLD 20
//    namedWindow("Rotation",cv::WINDOW_AUTOSIZE);
//    cv::createTrackbar( "Cloud_0_min", "Depth_control", &depth_control[0], MAX_TRESHOLD);
//    cv::createTrackbar( "Cloud_0_max", "Depth_control", &depth_control[1], MAX_TRESHOLD);
//    cv::createTrackbar( "Cloud_1_min", "Depth_control", &depth_control[2], MAX_TRESHOLD);
//    cv::createTrackbar( "Cloud_1_max", "Depth_control", &depth_control[3], MAX_TRESHOLD);
//    cv::createTrackbar( "Cloud_2_min", "Depth_control", &depth_control[4], MAX_TRESHOLD);
//    cv::createTrackbar( "Cloud_2_max", "Depth_control", &depth_control[5], MAX_TRESHOLD);
//}


std::string IntToStr(int n)
{
    std::stringstream result;
    result << n;
    return result.str();
}



