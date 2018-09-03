#include <iostream>
#include <string>
#include <thread>
#include <memory>
#include <chrono>
#include "kinect.h"
#include "pcl.h"

#include <opencv2/opencv.hpp>

void frames_show(std::shared_ptr<Kinect> pKinect0, std::shared_ptr<Kinect> pKinect1, std::shared_ptr<Kinect> pKinect2, std::atomic<bool> & imshow_running, std::atomic<bool> &main_loop);

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

    std::string head_cascade = "/home/jozef/Documents/QT/3DScan/cascades/haarcascades/haarcascade_profileface.xml";
    std::string other_cascade = "/home/jozef/Documents/QT/3DScan/cascades/haarcascades/haarcascade_profileface.xml";

    pKinect0->setCascades(head_cascade,other_cascade,true);
    pKinect1->setCascades(head_cascade,other_cascade,true);
    pKinect2->setCascades(head_cascade,other_cascade,true);

    pKinect0->loadCascades();
    pKinect2->loadCascades();
    pKinect1->loadCascades();

    pclViewer pclViewer(connectedDevices);
    pclViewer.spinOnce();

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

    std::atomic<bool> imshow_running {true};
    std::atomic<bool> main_loop {true};

    std::thread im_shower(frames_show,pKinect0,pKinect1,pKinect2,std::ref(imshow_running),std::ref(main_loop));
    im_shower.detach();

    pclCloud cloud0(pKinect0->getId());
    pclCloud cloud1(pKinect1->getId());
    pclCloud cloud2(pKinect2->getId());

    while(main_loop)
    {

       start=chrono::system_clock::now();

       cloud0.pclCopyCloud(pKinect0->getCloudData());
       cloud1.pclCopyCloud(pKinect1->getCloudData());
       cloud2.pclCopyCloud(pKinect2->getCloudData());


       pclViewer.pclAddCloud(cloud0.getCloud(),0);
       pclViewer.pclAddCloud(cloud1.getCloud(),1);
       pclViewer.pclAddCloud(cloud2.getCloud(),2);


       pclViewer.spinOnce();
       stop=chrono::system_clock::now();
       cout << "Execution Time last:" << chrono::duration_cast<chrono::milliseconds>(stop - start).count() << " ms" << endl;

    }

    return 0;
}

void frames_show(std::shared_ptr<Kinect> pKinect0, std::shared_ptr<Kinect> pKinect1, std::shared_ptr<Kinect> pKinect2, std::atomic<bool> &imshow_running,std::atomic<bool> &main_loop )
{
    chrono::system_clock::time_point start, stop;

    while(imshow_running)
    {
        start=chrono::system_clock::now();

        pKinect0->getDepth();
        pKinect1->getDepth();
        pKinect2->getDepth();

//        pKinect0->getRGB();
//        pKinect1->getRGB();
//        pKinect2->getRGB();

//        pKinect0->getIr();
//        pKinect1->getIr();
//        pKinect2->getIr();

//        pKinect0->getRGBD();
//        pKinect1->getRGBD();
//        pKinect2->getRGBD();

        pKinect0->getRangedRGBD();
        pKinect1->getRangedRGBD();
        pKinect2->getRangedRGBD();

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
        }
        stop=chrono::system_clock::now();
   //     cout << "ImVIEWER: " << chrono::duration_cast<chrono::milliseconds>(stop - start).count() << " ms" << endl;
    }
}

