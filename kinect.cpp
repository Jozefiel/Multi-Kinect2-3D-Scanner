#include "kinect.h"


Kinect::Kinect(int id, libfreenect2::Freenect2 *freenect2, libfreenect2::PacketPipeline *pipeline)
{
    kinect_id = id;
    serial = freenect2->getDeviceSerialNumber(id);

    std::clog<<serial<<std::endl;

    if(pipeline)
    {
        dev = freenect2->openDevice(serial, pipeline);
    }
    else
    {
        dev = freenect2->openDevice(serial);
    }

    libfreenect2::SyncMultiFrameListener * pListener = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Depth | libfreenect2::Frame::Ir );
    listener=pListener;

    dev->setColorFrameListener(listener);
    dev->setIrAndDepthFrameListener(listener);

    start();
    registration();

}

void Kinect::start()
{
    std::cout<<"Kinect start: "<<dev->getSerialNumber()<<std::endl;
    dev->start();
}

void Kinect::registration()
{
    std::cout<<"Frames registered "<<dev->getSerialNumber()<<std::endl;
    registrated = new libfreenect2::Registration(dev->getIrCameraParams(),dev->getColorCameraParams());
}

void Kinect::frames(std::atomic<bool> & keep_running)
{
    std::chrono::system_clock::time_point then, now;
    while(keep_running)
    {
        if(listener->waitForNewFrame(frame,camAttachTime))
        {

            then=std::chrono::system_clock::now();

            libfreenect2::Frame *depth=frame[libfreenect2::Frame::Depth];
            libfreenect2::Frame *ir=frame[libfreenect2::Frame::Ir];
            libfreenect2::Frame *rgb=frame[libfreenect2::Frame::Color];

            cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(*depthMat);
            cv::Mat(ir->height, ir->width, CV_32FC1, ir->data).copyTo(*irMat);
            cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(*colorMat);

//            if(mutex.try_lock_for(std::chrono::milliseconds(10)))
//            {
//                std::cout<<"mutex for frames locked "<<kinect_id<<std::endl;
                registrated->apply(rgb,depth,undistorted,registered,true,depth2rgb);
//                mutex.unlock();
//            }
//            else
//            {
//               std::cout<<"mutex for frames error "<<kinect_id<<std::endl;

//            }

            cv::Mat(registered->height, registered->width, CV_8UC4, registered->data).copyTo(*rgbdMat);

            now=std::chrono::system_clock::now();
//            std::cout << "Snapping: "<<this->getId()<<" "<< std::chrono::duration_cast<std::chrono::milliseconds>(now - then).count() << " ms" << endl;

            listener->release(frame);
        }
        else
        {
            std::cout<<this->getId()<<" hasn't new frame"<<std::endl;
        }
    }
}


std::string Kinect::getSerial()
{
    std::string _serial = serial;
    return _serial;
}

std::string Kinect::getIdString()
{
    std::stringstream id_string;
    id_string << kinect_id;
    return id_string.str();
}

int Kinect::getId()
{
    int _kinect_id=kinect_id;
    return _kinect_id;
}


void Kinect::getDepth()
{
    cv::imshow("depth: " + getIdString(), *depthMat / 2048);
}

void Kinect::rangedDepth(std::string window_name)
{
    depthControl(window_name);
    rangeMask->release();
    rangedDepthMap->release();
    cv::inRange(*depthMat,low_slider/10,high_slider/10,*rangeMask);
    depthMat->copyTo(*rangedDepthMap,*rangeMask);
}

void Kinect::getRangedDepth()
{
    if(!depthMat->empty())
    {
        std::string window_name="rDepth";
        rangedDepth(window_name);
        cv::imshow(window_name + getIdString(), *rangedDepthMap / 2048);
    }
}

void Kinect::getRGB()
{
    if(!colorMat->empty())
    {
        cv::imshow("color: " + getIdString(), *colorMat);
    }
}

void Kinect::getIr()
{
    if(!irMat->empty())
    {
        cv::imshow("ir: " + getIdString(), *irMat  / 2048);
    }
}

void Kinect::getRGBD()
{
    if(!rgbdMat->empty())
    {
        cv::imshow("rgbd: " + getIdString(), *rgbdMat );
    }
}

cv::Mat Kinect::getRGBDFrame()
{
    return *rgbdMat;
}

void Kinect::rangedRGBD(std::string window_name)
{
    rangedRGBDMat->release();
    rangedDepth(window_name);
    if(!rgbdMat->empty() && !rangeMask->empty())
    {
        rgbdMat->copyTo(*rangedRGBDMat,*rangeMask);
    }
    else if (!rgbdMat->empty())
    {
        rgbdMat->copyTo(*rangedRGBDMat);
    }
}

void Kinect::getRangedRGBD()
{
    if(!rgbdMat->empty() && !rangeMask->empty())
    {
        std::string window_name = "rRGBD";
        rangedRGBD(window_name);
        headDetect();
        if(!rangedRGBDMat->empty())
        {
            cv::imshow(window_name + getIdString(), *rangedRGBDMat );
        }
    }
}

void Kinect::depthControl(std::string window_name)
{
    cv::namedWindow(window_name + getIdString());
    cv::createTrackbar( "Low Depth range", window_name + getIdString(), &low_slider, 40950);
    cv::createTrackbar( "High Depth range",window_name + getIdString(), &high_slider, 40950);
    cv::createTrackbar( "Low Depth pcl ", window_name + getIdString(), &low_depth_slider, 10000);
    cv::createTrackbar( "High Depth pcl ", window_name + getIdString(), &high_depth_slider, 10000);
}


void Kinect::setCascades(std::string cascade_head,std::string cascade_other, bool load_cascade)
{
    face_cascade_name=cascade_head;
    other_cascade_name=cascade_other;
    if(load_cascade)
    {
        loadCascades();
    }
}

void Kinect::loadCascades()
{
    if( !face_cascade.load( face_cascade_name ) )
    {
        std::cout<<"face_cascade not loaded"<<std::endl;
    };
    if( !eyes_cascade.load( other_cascade_name ) )
    {
        std::cout<<"eyes_cascade not loaded"<<std::endl;
    };
}

void Kinect::headDetect()
{
    std::vector<cv::Rect> faces;
    cv::Mat tmpMat = rangedRGBDMat->clone();
    cv::Mat tmpMatGray;
    cv::cvtColor(tmpMat,tmpMatGray,CV_BGR2GRAY);
    cv::equalizeHist(tmpMatGray,tmpMatGray);

    face_cascade.detectMultiScale( tmpMatGray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(10, 60) );

    for( size_t i = 0; i < faces.size(); i++ )
    {
        cv::rectangle(*rangedRGBDMat, faces[i], cv::Scalar(255,0,125), 3, 8, 0);
    }
}


void Kinect::cloudInit()
{
    cloud->clear();
    cloud->width = static_cast<uint32_t>(ir_depth_width);
    cloud->height = static_cast<uint32_t>(ir_depth_height);
    cloud->is_dense = false;
    cloud->points.resize( cloud->width * cloud->height);
}

void Kinect::cloudDataThread(std::atomic<bool> &keep_running)
{
    std::chrono::system_clock::time_point then, now;

    while(keep_running)
    {
        then=std::chrono::system_clock::now();

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        tmpCloud->width = static_cast<uint32_t>(ir_depth_width);
        tmpCloud->height = static_cast<uint32_t>(ir_depth_height);
        tmpCloud->is_dense = false;
        tmpCloud->points.resize( tmpCloud->width* tmpCloud->height);

        float x=0, y=0, z=0;
        unsigned long n=0;

        for(int x_side=0;x_side<ir_depth_width;x_side++)
        {
            for(int y_side=0;y_side<ir_depth_height;y_side++)
            {
                float rgb;
                registrated->getPointXYZRGB(undistorted,registered,y_side,x_side,x,y,z,rgb);
                const uint8_t *p = reinterpret_cast<uint8_t*>(&rgb);
                uint8_t b = p[0];
                uint8_t g = p[1];
                uint8_t r = p[2];

                if( z>(0.7) || std::isinf(z))
                {
                    x=NAN;
                    z=NAN;
                    y=NAN;
                }

                tmpCloud->points[n].x=x;
                tmpCloud->points[n].y=y;
                tmpCloud->points[n].z=z;
                tmpCloud->points[n].r=r;
                tmpCloud->points[n].g=g;
                tmpCloud->points[n].b=b;

                n++;
            }
        }

        std::vector<int> removedPoints;
        pcl::removeNaNFromPointCloud(*tmpCloud,*tmpCloud,removedPoints);

        if(!tmpCloud->empty())
        {
            if(cloud_mutex.try_lock_for(std::chrono::milliseconds(5)))
            {
//                std::cout<<"No empty cloud: "<<kinect_id<<std::endl;
                cloudInit();
                pcl::copyPointCloud(*tmpCloud,*cloud);
                cloud_mutex.unlock();
            }
        }
        else
        {
            std::cout<<"Empty cloud: "<<kinect_id<<std::endl;
        }
        tmpCloud->clear();

        now=std::chrono::system_clock::now();
  //      std::cout << "CLOUD: "<<this->getId()<<" "<< std::chrono::duration_cast<std::chrono::milliseconds>(now - then).count() << " ms" << endl;
    }
}


void Kinect::cloudData()
{
    std::chrono::system_clock::time_point then, now;
    then=std::chrono::system_clock::now();

     pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
     tmpCloud->width = static_cast<uint32_t>(ir_depth_width);
     tmpCloud->height = static_cast<uint32_t>(ir_depth_height);
     tmpCloud->is_dense = false;
     tmpCloud->points.resize( tmpCloud->width* tmpCloud->height);

     float x=0, y=0, z=0;
     unsigned long n=0;

     for(int x_side=0;x_side<ir_depth_width;x_side++)
     {
         for(int y_side=0;y_side<ir_depth_height;y_side++)
         {
             float rgb;
             registrated->getPointXYZRGB(undistorted,registered,y_side,x_side,x,y,z,rgb);
             const uint8_t *p = reinterpret_cast<uint8_t*>(&rgb);
             uint8_t b = p[0];
             uint8_t g = p[1];
             uint8_t r = p[2];

//             if(std::isnan(x) || std::isinf(x))
//             {
//                 x=0;
//             }
//             if(std::isnan(y) || std::isinf(y))
//             {
//                 y=0;
//             }
             if(std::isnan(z) || std::isinf(z) || z <low_depth_slider/5000 || z>high_depth_slider/5000)
             {
                 x=0;
                 z=0;
                 y=0;
             }

             tmpCloud->points[n].x=x;
             tmpCloud->points[n].y=y;
             tmpCloud->points[n].z=z;
             tmpCloud->points[n].r=r;
             tmpCloud->points[n].g=g;
             tmpCloud->points[n].b=b;

             n++;
         }
     }

     if(!tmpCloud->empty())
     {
         if(cloud_mutex.try_lock_for(std::chrono::milliseconds(10)))
         {
             std::cout<<"No empty cloud: "<<kinect_id<<std::endl;
             cloudInit();
             pcl::copyPointCloud(*tmpCloud,*cloud);
             cloud_mutex.unlock();
         }
     }
     else
     {
         std::cout<<"Empty cloud: "<<kinect_id<<std::endl;
     }
     tmpCloud->clear();
     now=std::chrono::system_clock::now();
//     std::cout << "CLOUD: "<<this->getId()<<" "<< std::chrono::duration_cast<std::chrono::milliseconds>(now - then).count() << " ms" << endl;
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr Kinect::getCloudData()
{
    return cloud;
}


Kinect::~Kinect()
{
    std::cout<<"END "<<getId()<<" "<<getSerial()<<std::endl;
    dev->stop();
    dev->close();
    delete registrated;
}


