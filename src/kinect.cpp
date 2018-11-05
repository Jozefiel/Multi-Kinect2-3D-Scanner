#include "camera.h"
#include "kinect.h"

#include <exception>

#define minimal_depth 0
#define maximal_depth 0.9
#define mutex_lock_time 15

Kinect::Kinect(libfreenect2::Freenect2 * _freenect,int _id)
{
    try
    {
        this->pFreenect=_freenect;
        this->camera_type="Kinect v2";
        this->id=_id;
        this->serial=pFreenect->getDeviceSerialNumber(id);
        std::cout<<"Serial: "<<this->serial<<std::endl;
        std::cout<<"Id: "<<this->id<<std::endl;
        if(pPipeline)
        {
            this->pDev=pFreenect->openDevice(serial, pPipeline);
        }
        else
        {
            this->pDev=pFreenect->openDevice(serial);
        }

        this->pListener = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Depth | libfreenect2::Frame::Ir );
        this->pDev->setIrAndDepthFrameListener(pListener);
        this->pDev->setColorFrameListener(pListener);

        this->start();
        this->registration();
        this->loadCamParams();
    }
    catch (std::exception& e)
    {
        std::cout << "Problem with libfreenect pointer " << e.what() <<std::endl;
    }
}

void Kinect::start()
{
    std::cout<<"Kinect start: "<<this->pDev->getSerialNumber()<<std::endl;
    this->pDev->start();
}

void Kinect::stop()
{
    std::cout<<"Kinect stop: "<<this->pDev->getSerialNumber()<<std::endl;
    this->pDev->stop();
}

void Kinect::registration()
{
    std::cout<<"Frames registered "<<this->pDev->getSerialNumber()<<std::endl;
    this->pRegistrated = new libfreenect2::Registration(this->pDev->getIrCameraParams(),this->pDev->getColorCameraParams());
}

void Kinect::frames(std::atomic<bool> & keep_running)
{
    std::chrono::system_clock::time_point then, now;

    while(keep_running)
    {
        if(this->pListener->waitForNewFrame(frame,camAttachTime))
        {

            then=std::chrono::system_clock::now();

            libfreenect2::Frame *depth=frame[libfreenect2::Frame::Depth];
            libfreenect2::Frame *ir=frame[libfreenect2::Frame::Ir];
            libfreenect2::Frame *rgb=frame[libfreenect2::Frame::Color];

            cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(*this->depthMat);
            cv::Mat(ir->height, ir->width, CV_32FC1, ir->data).copyTo(*this->irMat);
            cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(*this->colorMat);

            this->pRegistrated->apply(rgb,depth, this->undistorted, this->registered,true, this->depth2rgb);
            cv::Mat( this->registered->height,  this->registered->width, CV_8UC4, registered->data).copyTo(* this->rgbdMat);

            now=std::chrono::system_clock::now();
//            std::cout << "Snapping: "<<this->id<<" "<< std::chrono::duration_cast<std::chrono::milliseconds>(now - then).count() << " ms" << std::endl;

            pListener->release(frame);
        }
        else
        {
            std::cout<<this->id<<" hasn't new frame"<<std::endl;
        }
    }
}

std::string Kinect::getCamType()
{
    return camera_type;
}

void Kinect::loadCamParams()
{
    try {
        std::cout << "Kinect::loadCamParams "<< serial<<std::endl;

        boost::property_tree::ini_parser::read_ini("config/"+serial+".ini", pt);

        calib_params.cx = pt.get<float>("Calibration.transform_rgb_cx");
        calib_params.cy = pt.get<float>("Calibration.transform_rgb_cy");
        calib_params.fx = pt.get<float>("Calibration.transform_rgb_fx");
        calib_params.fy = pt.get<float>("Calibration.transform_rgb_fy");

    } catch (int e) {

    }
}

cv::Mat Kinect::getRGB()
{
    return *colorMat;
}

cv::Mat Kinect::getDepth()
{
    return *depthMat;
}

cv::Mat Kinect::getIR()
{
    return *irMat;
}

cv::Mat Kinect::getRGBD()
{
    return *rgbdMat;
}

void Kinect::cloudInit()
{
    cloud->clear();
    cloud->width = static_cast<uint32_t>(ir_depth_width);
    cloud->height = static_cast<uint32_t>(ir_depth_height);
    cloud->is_dense = false;
    cloud->points.resize( cloud->width * cloud->height);
}

void Kinect::cloudData(std::atomic<bool> & keep_running)
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
                pRegistrated->getPointXYZRGB(undistorted,registered,y_side,x_side,x,y,z,rgb);
                const uint8_t *p = reinterpret_cast<uint8_t*>(&rgb);
                uint8_t b = p[0];
                uint8_t g = p[1];
                uint8_t r = p[2];

                if((r<150 && b <150 && g<150) || (r >10 && b >10 && g >10) )
                {
                    if(std::isinf(x) || std::isinf(y) ||  std::isinf(z) )
                    {
                        x=NAN;
                        z=NAN;
                        y=NAN;
                    }
                    else if ( z > static_cast<float>(maximal_depth) )
                    {
                        x=NAN;
                        z=NAN;
                        y=NAN;
                    }
                }
                else
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
            if(cloud_mutex.try_lock_for(std::chrono::milliseconds(mutex_lock_time)))
            {
                cloudInit();
                pcl::copyPointCloud(*tmpCloud,*cloud);
                cloud_mutex.unlock();
            }
        }
        tmpCloud->clear();
        now=std::chrono::system_clock::now();
//        std::cout << "CLOUD: "<<this->getId()<<" "<< std::chrono::duration_cast<std::chrono::milliseconds>(now - then).count() << " ms" << std::endl;
    }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Kinect::getCloudData()
{
    return cloud;
}

int Kinect::getId()
{
    return id;
}

std::string Kinect::getSerial()
{
    return serial;
}

bool Kinect::lockCloud(int lock_time)
{
    if(cloud_mutex.try_lock_for(std::chrono::milliseconds(lock_time)))
        return true;
    else
        return false;
}

void Kinect::unlockCloud()
{
    cloud_mutex.unlock();
}

Kinect::~Kinect()
{
    std::cout<<"END "<<this->id<<" "<<this->serial<<std::endl;
    this->pDev->stop();
    this->pDev->close();
    delete this->pRegistrated;
}


