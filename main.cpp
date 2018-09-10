#include <iostream>
#include <string>
#include <thread>
#include <memory>
#include <chrono>
#include "kinect.h"
#include "pcl.h"

#include <opencv2/opencv.hpp>

void frames_show(std::shared_ptr<Kinect> pKinect0, std::shared_ptr<Kinect> pKinect1, std::shared_ptr<Kinect> pKinect2,  pclCloud merged_cloud, std::atomic<bool> & imshow_running, std::atomic<bool> &main_loop);
std::string IntToStr(int n);
void matrix_rotat_koef();

int matrix[16]={713,29,559,0};
float fmatrix[16]={0};

using namespace std;
int main()
{
    libfreenect2::setGlobalLogger(nullptr);

    libfreenect2::PacketPipeline *pipeline = nullptr;
    libfreenect2::Freenect2 *pFreenect2 = new libfreenect2::Freenect2;

    const int connectedDevices = pFreenect2->enumerateDevices();

    string * serial = new string [connectedDevices];

    if(connectedDevices == 0)
    {
        std::cerr << "no device connected!" << std::endl;
        return -1;
    }
    else
    {
        for(int i=0;i<connectedDevices;i++)
        {
           serial[i] = pFreenect2->getDeviceSerialNumber(i);
        }
    }

    Kinect kinect0(0,pFreenect2,pipeline);
    Kinect kinect1(1,pFreenect2,pipeline);
    Kinect kinect2(2,pFreenect2,pipeline);

    std::shared_ptr<Kinect> pKinect0 (&kinect0);
    std::shared_ptr<Kinect> pKinect1 (&kinect1);
    std::shared_ptr<Kinect> pKinect2 (&kinect2);

  //  pclViewer cloudViewer(connectedDevices,"3D Scan");
    pclViewer rotationViewer(1,"Rotation 3D Scan");

    chrono::system_clock::time_point start, stop;

    std::atomic<bool> snap_running {true};

    std::thread cam0(&Kinect::frames,pKinect0,std::ref(snap_running));
    std::thread cam1(&Kinect::frames,pKinect1,std::ref(snap_running));
    std::thread cam2(&Kinect::frames,pKinect2,std::ref(snap_running));

    cam0.detach();
    cam1.detach();
    cam2.detach();

    std::thread cloud_cam_0(&Kinect::cloudDataThread,pKinect0,std::ref(snap_running));
    std::thread cloud_cam_1(&Kinect::cloudDataThread,pKinect1,std::ref(snap_running));
    std::thread cloud_cam_2(&Kinect::cloudDataThread,pKinect2,std::ref(snap_running));

    cloud_cam_0.detach();
    cloud_cam_1.detach();
    cloud_cam_2.detach();

    pclCloud cloud0(pKinect0->getId());
    pclCloud cloud1(pKinect1->getId());
    pclCloud cloud2(pKinect2->getId());
    pclCloud merged_cloud(4);

    std::atomic<bool> imshow_running {true};
    std::atomic<bool> main_loop {true};

    std::thread im_shower(frames_show,pKinect0,pKinect1,pKinect2,merged_cloud,std::ref(imshow_running),std::ref(main_loop));
    im_shower.detach();



    pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_cloud0, cloud_0,cloud_1;

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


    while(main_loop)
    {

//        fmatrix[0]= matrix[0];
//        fmatrix[1]= matrix[1];
//        fmatrix[2]= matrix[2];

//        transform_1 (0,3) = fmatrix[0]/1000;
//        transform_1 (1,3) = -fmatrix[1]/500 + fmatrix[1]/1000;
//        transform_1 (2,3) = fmatrix[2]/1000;

        transform_2 (0,0) = -0.033612424484263*(1-matrix[0]);
        transform_2 (0,1) = 0.049480919369063*(1-matrix[1]);
        transform_2 (0,2) = -0.998209318499222*(1-matrix[2]);
        transform_2 (0,3) = 0;
        transform_2 (1,0) = -0.134517708146897*(1-matrix[4]);
        transform_2 (1,1) = 0.989461724369153*(1-matrix[5]);
        transform_2 (1,2) = 0.053576881239273*(1-matrix[6]);
        transform_2 (1,3) = 0;
        transform_2 (2,0) = 0.990340946904245*(1-matrix[8]);
        transform_2 (2,1) = 0.136077678650149*(1-matrix[9]);
        transform_2 (2,2) = -0.026602147619899*(1-matrix[10]);
        transform_2 (2,3) = 0;
        transform_2 (3,0) = 0;
        transform_2 (3,1) = 0;
        transform_2 (3,2) = 0;
        transform_2 (3,3) = 1;

        transform_2 (0,3) = fmatrix[4]/1000;
        transform_2 (1,3) = -fmatrix[8]/500 + fmatrix[8]/1000;
        transform_2 (2,3) = fmatrix[11]/1000;

        cloud2.setTransformationMatrix(transform_1);
        cloud1.setTransformationMatrix(transform_2);

        start=chrono::system_clock::now();

        if(pKinect0->cloud_mutex.try_lock_for(std::chrono::milliseconds(10)))
        {
            cloud0.pclCopyCloud(pKinect0->getCloudData());
            cloud0.removeOutliers(10,1.0);
            pKinect0->cloud_mutex.unlock();
        }

        if(pKinect1->cloud_mutex.try_lock_for(std::chrono::milliseconds(10)))
        {
            cloud1.pclCopyCloud(pKinect1->getCloudData());
            cloud1.removeOutliers(10,1.0);
            pKinect1->cloud_mutex.unlock();
        }


        if(pKinect2->cloud_mutex.try_lock_for(std::chrono::milliseconds(10)))
        {
            cloud2.pclCopyCloud(pKinect2->getCloudData());
            cloud2.removeOutliers(10,1.0);
            pKinect2->cloud_mutex.unlock();
        }

       cloud0.transformPointCloud();
       cloud1.transformPointCloud();
       cloud2.transformPointCloud();

       std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
       clouds.push_back(cloud0.getTransformedCloud());
       clouds.push_back(cloud1.getTransformedCloud());
       clouds.push_back(cloud2.getTransformedCloud());

       merged_cloud.mergeClouds(clouds);

//       cloudViewer.pclAddCloud(cloud0.getCloud(),0);
//       cloudViewer.pclAddCloud(merged_cloud,1);
//       cloudViewer.pclAddCloud(cloud2.getCloud(),2);

       rotationViewer.pclAddCloud(merged_cloud.getCloud(),0);

//       cloudViewer.spinOnce();
       rotationViewer.spinOnce();
       stop=chrono::system_clock::now();
//       cout << "Execution Time last:" << chrono::duration_cast<chrono::milliseconds>(stop - start).count() << " ms" << endl;

    }

    return 0;
}

void frames_show(std::shared_ptr<Kinect> pKinect0, std::shared_ptr<Kinect> pKinect1, std::shared_ptr<Kinect> pKinect2, pclCloud merged_cloud, std::atomic<bool> &imshow_running,std::atomic<bool> &main_loop )
{
    chrono::system_clock::time_point start, stop;
    int saved_frame_counter=0;

    while(imshow_running)
    {
        start=chrono::system_clock::now();


        matrix_rotat_koef();

//        pKinect0->getDepth();
//        pKinect1->getDepth();
//        pKinect2->getDepth();

//        pKinect0->getRGB();
//        pKinect1->getRGB();
//        pKinect2->getRGB();

//        pKinect0->getIr();
//        pKinect1->getIr();
//        pKinect2->getIr();

        pKinect0->getRGBD();
        pKinect1->getRGBD();
        pKinect2->getRGBD();

//        pKinect0->getRangedRGBD();
//        pKinect1->getRangedRGBD();
//        pKinect2->getRangedRGBD();

//        pKinect0->getRangedDepth();
//        pKinect1->getRangedDepth();
//        pKinect2->getRangedDepth();

        auto key = cv::waitKey(50);
        switch( (char) key)
        {
            case 'Q':
            case 'q':
            case 27 :
                main_loop=false;
                break;

            case 'w':
            case 'W':
                cv::imwrite("RGBD_1_"+IntToStr(saved_frame_counter)+".jpeg",pKinect0->getRGBDFrame() );
                cv::imwrite("RGBD_2_"+IntToStr(saved_frame_counter)+".jpeg",pKinect1->getRGBDFrame() );
                cv::imwrite("RGBD_3_"+IntToStr(saved_frame_counter)+".jpeg",pKinect2->getRGBDFrame());
                pcl::io::savePLYFile("clouds.ply",*merged_cloud.getCloud());
                saved_frame_counter++;


                break;
         }
        stop=chrono::system_clock::now();
   //     cout << "ImVIEWER: " << chrono::duration_cast<chrono::milliseconds>(stop - start).count() << " ms" << endl;
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


string IntToStr(int n)
{
    stringstream result;
    result << n;
    return result.str();
}
