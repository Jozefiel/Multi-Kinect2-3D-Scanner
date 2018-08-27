#include <iostream>
#include <string>
#include <thread>
#include <memory>
#include <chrono>
#include "kinect.h"
#include "pcl.h"

#include <opencv2/opencv.hpp>

void frames_show(std::shared_ptr<Kinect> pKinect0, std::shared_ptr<Kinect> pKinect1, std::shared_ptr<Kinect> pKinect2, std::atomic<bool> & imshow_running);
bool daco = true;


using namespace std;
int main()
{
    libfreenect2::setGlobalLogger(nullptr);

    libfreenect2::PacketPipeline *pipeline = nullptr;
    libfreenect2::Freenect2 *pFreenect2 = new libfreenect2::Freenect2;

    static int connectedDevices = pFreenect2->enumerateDevices();

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


//    pcl::visualization::PCLVisualizer *viewer (new pcl::visualization::PCLVisualizer("3D Viewer"));
    pclKinect pclViewer(connectedDevices);

    chrono::system_clock::time_point start, stop;

    std::atomic<bool> snap_running {true};

    std::thread cam1(&Kinect::frames,pKinect0,std::ref(snap_running));
    std::thread cam2(&Kinect::frames,pKinect1,std::ref(snap_running));
    std::thread cam3(&Kinect::frames,pKinect2,std::ref(snap_running));

    cam1.detach();
    cam2.detach();
    cam3.detach();

    std::atomic<bool> imshow_running {true};

    std::thread im_shower(frames_show,pKinect0,pKinect1,pKinect2,std::ref(imshow_running));
    im_shower.detach();

    bool loop=true;
    while(loop)
    {

       start=chrono::system_clock::now();

//       if(im_shower.joinable())
//       {
//           cout<<"IamHere"<<endl;
//           im_shower.join();
//       }
 //      frames_show(pKinect0,pKinect1,pKinect2);

//       pKinect0->getDepth();
//       pKinect1->getDepth();
//       pKinect2->getDepth();



//
//        kinect0.getRGB();
//        kinect1.getRGB();
//        kinect2.getRGB();

//       kinect0.getRGBD();
//       kinect1.getRGBD();
//       kinect2.getRGBD();

//       kinect0.getRangedRGBD();
//       kinect1.getRangedRGBD();
//       kinect2.getRangedRGBD();


//       std::thread t4(&Kinect::cloudData,&kinect0);
//       std::thread t5(&Kinect::cloudData,&kinect1);
//       std::thread t6(&Kinect::cloudData,&kinect2);

//       t4.join();
//       t5.join();
//       t6.join();




//       kinect0.getDepth();
//       kinect1.getDepth();
//       kinect2.getDepth();



//std::thread t7(&pclKinect::pclAddCloud,&pclViewer,kinect0.getCloudData(),kinect0.getId());
//std::thread t8(&pclKinect::pclAddCloud,&pclViewer,kinect1.getCloudData(),kinect2.getId());
//std::thread t9(&pclKinect::pclAddCloud,&pclViewer,kinect2.getCloudData(),kinect1.getId());

//t7.join();
//t8.join();
//t9.join();

  //     viewer.addPointCloud (kinect0.getCloudData(), "cloud_in_v"+kinect0.getId(),v1);
  //     viewer.addPointCloud (kinect1.getCloudData(), "cloud_in_v"+kinect1.getId(),v2);
  //     viewer.addPointCloud (kinect2.getCloudData(), "cloud_in_v"+kinect2.getId(),v3);

//        auto key = cv::waitKey(5);
//        switch( (char) key)
//        {
//            case 'Q':
//            case 'q':
//            case 27 :
//                loop=false;
//            break;
//        }
    //    pclViewer.spinOnce();

        stop=chrono::system_clock::now();
        cout << "Execution Time last:" << chrono::duration_cast<chrono::milliseconds>(stop - start).count() << " ms" << endl;
    }

//    kinect0.~Kinect();
//    kinect1.~Kinect();
//    kinect2.~Kinect();

    return 0;
}

void frames_show(std::shared_ptr<Kinect> pKinect0, std::shared_ptr<Kinect> pKinect1, std::shared_ptr<Kinect> pKinect2, std::atomic<bool> &imshow_running)
{
    while(imshow_running)
    {
        pKinect0->getDepth();
        pKinect1->getDepth();
        pKinect2->getDepth();
        cv::waitKey(5);
    }

}
