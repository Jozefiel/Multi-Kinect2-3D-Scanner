#include <iostream>
#include <string>
#include <thread>
#include <chrono>

#include "kinect.h"
#include "pcl.h"

#include <opencv2/opencv.hpp>

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

//    pcl::visualization::PCLVisualizer *viewer (new pcl::visualization::PCLVisualizer("3D Viewer"));
    pclKinect pclViewer(connectedDevices);

    chrono::system_clock::time_point then, now, start, stop;

    bool loop=true;
    while(loop)
    {
        start=chrono::system_clock::now();
        then=chrono::system_clock::now();

        //viewer.removePointCloud("cloud_in_v"+kinect0.getId());
        //viewer.removePointCloud("cloud_in_v"+kinect1.getId());
        //viewer.removePointCloud("cloud_in_v"+kinect2.getId());
        now=chrono::system_clock::now();
        cout << "Viewe remover time: " << chrono::duration_cast<chrono::milliseconds>(now - then).count() << " ms" << endl;

        then=chrono::system_clock::now();

        std::thread t1(&Kinect::frames,&kinect0);
        std::thread t2(&Kinect::frames,&kinect1);
        std::thread t3(&Kinect::frames,&kinect2);

        t1.join();
        t2.join();
        t3.join();

        now=chrono::system_clock::now();
        cout << "Kinect frames time: " << chrono::duration_cast<chrono::milliseconds>(now - then).count() << " ms" << endl;

//       kinect0.getDepth();
//       kinect1.getDepth();
//       kinect2.getDepth();

//        kinect0.getRGB();
//        kinect1.getRGB();
//        kinect2.getRGB();

//       kinect0.getRGBD();
//       kinect1.getRGBD();
//       kinect2.getRGBD();

//       kinect0.getRangedRGBD();
//       kinect1.getRangedRGBD();
//       kinect2.getRangedRGBD();

then=chrono::system_clock::now();

       std::thread t4(&Kinect::cloudData,&kinect0);
       std::thread t5(&Kinect::cloudData,&kinect1);
       std::thread t6(&Kinect::cloudData,&kinect2);

       t4.join();
       t5.join();
       t6.join();

now=chrono::system_clock::now();
cout << "PCL cloud time: " << chrono::duration_cast<chrono::milliseconds>(now - then).count() << " ms" << endl;

then=chrono::system_clock::now();


       kinect0.getDepth();
       kinect1.getDepth();
       kinect2.getDepth();

now=chrono::system_clock::now();
cout << "Kinect show frames time: " << chrono::duration_cast<chrono::milliseconds>(now - then).count() << " ms" << endl;

then=chrono::system_clock::now();


std::thread t7(&pclKinect::pclAddCloud,&pclViewer,kinect0.getCloudData(),kinect0.getId());
std::thread t8(&pclKinect::pclAddCloud,&pclViewer,kinect1.getCloudData(),kinect2.getId());
std::thread t9(&pclKinect::pclAddCloud,&pclViewer,kinect2.getCloudData(),kinect1.getId());

t7.join();
t8.join();
t9.join();

       //viewer.addPointCloud (kinect0.getCloudData(), "cloud_in_v"+kinect0.getId(),v1);
      // viewer.addPointCloud (kinect1.getCloudData(), "cloud_in_v"+kinect1.getId(),v2);
      // viewer.addPointCloud (kinect2.getCloudData(), "cloud_in_v"+kinect2.getId(),v3);

now=chrono::system_clock::now();
cout << "PCL show frames time: " << chrono::duration_cast<chrono::milliseconds>(now - then).count() << " ms" << endl;

        int key = cv::waitKey(30);
        switch( (char) key)
        {
            case 'Q':
            case 'q':
            case 27 :
                loop=false;
            break;
        }
        pclViewer.spinOnce();

stop=chrono::system_clock::now();
cout << "Execution Time last:" << chrono::duration_cast<chrono::milliseconds>(stop - start).count() << " ms" << endl;


    }

    kinect0.~Kinect();
    kinect1.~Kinect();
    kinect2.~Kinect();

    return 0;
}
