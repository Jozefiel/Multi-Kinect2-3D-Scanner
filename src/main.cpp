#include <iostream>
#include <string>
#include <thread>
#include <memory>
#include <chrono>

#include "camera.h"
#include "kinect.h"
#include "support.h"
#include "pcl.h"


#define imshow_32to8 2048

std::string IntToStr(int n);
void frames_show(std::vector<Camera*> connected_cams, pclCloud merged_cloud, std::atomic<bool> & snap_running);
void matrix_rotat_koef();
void min_max_cloud();

int matrix[16]={610,73,690,748,101,737};
//int depth_control[6]={0,0,0,0,0,0};

float fmatrix[16]={0};

int main()
{

    libfreenect2::setGlobalLogger(nullptr);
    support support;

    support.cameraInit();                                       //! camera init, create connection with Kinects and Realsenses
    support.threadsInit();                                      //! threads init, detached snapping and cloud computing started
    support.cloudInit();                                        //! create point cloud for each camera

//  pclViewer cloudViewer(support.connectedCameras(),"3D Scan");   create viewport for each connected camera

    pclViewer rotationViewer(0,"Rotation 3D Scan");             //! create cloud viewer for all cloud connected together
    pclCloud merged_cloud(4);                                   //! cloud for connected clouds from all cameras

    std::atomic<bool> snap_running {true};

  std::thread im_shower(frames_show,support.cameras(),merged_cloud,std::ref(snap_running));
  im_shower.detach();

    std::chrono::system_clock::time_point start, stop;
    while(snap_running)
    {

        start=std::chrono::system_clock::now();

        support.camera2cloudDataTransfer();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        merged_cloud.mergeClouds(support.mergeClouds(false));

//        merged_cloud.removeOutliers(20,1.5);

//        start=std::chrono::system_clock::now();

        if(!merged_cloud.getCloud()->empty())
            rotationViewer.pclAddCloud(merged_cloud.getCloud(),0);

        //cloudViewer.spinOnce();
        rotationViewer.spinOnce();

       stop=std::chrono::system_clock::now();
       std::cout << "Execution Time last:" << std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count() << " ms" << std::endl;

    }

//    for (int i = 0; i < connected_cams.size(); i++)
//    {
//        delete connected_cams[i];
//    }

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

            cv::imshow("Depth_Image" + IntToStr(connected_cams[connected_cams_number]->getId()),(connected_cams[connected_cams_number]->getDepth() / imshow_32to8 ));
   //         cv::imshow("RGB_Image" + IntToStr(connected_cams[connected_cams_number]->getId()),(connected_cams[connected_cams_number]->getRGB()));
            cv::imshow("IR_Image" + IntToStr(connected_cams[connected_cams_number]->getId()),(connected_cams[connected_cams_number]->getIR() / imshow_32to8 ));
            cv::imshow("RGBD_Image" + IntToStr(connected_cams[connected_cams_number]->getId()),(connected_cams[connected_cams_number]->getRGBD() ));

            cv::moveWindow("Depth_Image" + IntToStr(connected_cams[connected_cams_number]->getId()),ir_depth_width*connected_cams_number+1920,0);
            cv::moveWindow("RGBD_Image" + IntToStr(connected_cams[connected_cams_number]->getId()),ir_depth_width*connected_cams_number+1920,ir_depth_height);
            cv::moveWindow("IR_Image" + IntToStr(connected_cams[connected_cams_number]->getId()),ir_depth_width*connected_cams_number,ir_depth_height*2);
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
            //    merged_cloud.creteMesh(20);
                saved_frame_counter++;
                break;

            case '1':
                connected_cams[0]->start();
                break;
            case '2':
                connected_cams[1]->start();
                break;
            case '3':
                connected_cams[2]->start();
                break;
            case '4':
                connected_cams[0]->stop();
                break;
            case '5':
                connected_cams[1]->stop();
                break;
            case '6':
                connected_cams[2]->stop();
                break;

         }
        stop=std::chrono::system_clock::now();
//        cout << "ImVIEWER: " << std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count() << " ms" << endl;
    }
}

void matrix_rotat_koef()
{
#define MAX_TRESHOLD 20000
    namedWindow("Rotation",cv::WINDOW_AUTOSIZE);
    cv::createTrackbar( "[0,0]x", "Rotation", &matrix[0], MAX_TRESHOLD);
    cv::createTrackbar( "[0,1]y", "Rotation", &matrix[1], MAX_TRESHOLD);
    cv::createTrackbar( "[0,2]z", "Rotation", &matrix[2], MAX_TRESHOLD);
    cv::createTrackbar( "[0,3]x", "Rotation", &matrix[3], MAX_TRESHOLD);
    cv::createTrackbar( "[1,0]y", "Rotation", &matrix[4], MAX_TRESHOLD);
    cv::createTrackbar( "[1,1]z", "Rotation", &matrix[5], MAX_TRESHOLD);
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

std::string IntToStr(int n)
{
    std::stringstream result;
    result << n;
    return result.str();
}



