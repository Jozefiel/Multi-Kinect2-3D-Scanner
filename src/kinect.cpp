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

        this->loadCamParams();
//        this->pDev->setIrCameraParams(ir_calib_params);
        this->registration();

    }
    catch (std::exception& e)
    {
        std::cout << "Problem with libfreenect pointer " << e.what() <<std::endl;
    }
}

int Kinect::getId()
{
    return id;
}

std::string Kinect::getSerial()
{
    return serial;
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
          //  this->pRegistrated->undistortDepth(depth,undistortedDepth);
            this->pRegistrated->apply(rgb,depth, this->undistorted, this->registered,true, this->depth2rgb);
          //  cv::Mat( this->undistortedDepth->height,  this->undistortedDepth->width, CV_8UC4, undistortedDepth->data).copyTo(* this->depthMat);
            cv::Mat( this->registered->height,  this->registered->width, CV_8UC4, registered->data).copyTo(* this->rgbdMat);

            //now=std::chrono::system_clock::now();
  //          std::cout << "Snapping: "<<this->id<<" "<< std::chrono::duration_cast<std::chrono::milliseconds>(now - then).count() << " ms" << std::endl;
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

        rgb_calib_params.cx = pt.get<float>("Calibration.transform_rgb_cx");
        rgb_calib_params.cy = pt.get<float>("Calibration.transform_rgb_cy");
        rgb_calib_params.fx = pt.get<float>("Calibration.transform_rgb_fx");
        rgb_calib_params.fy = pt.get<float>("Calibration.transform_rgb_fy");

        ir_calib_params.cx = pt.get<float>("Calibration.transform_ir_cx");
        ir_calib_params.cy = pt.get<float>("Calibration.transform_ir_cy");
        ir_calib_params.fx = pt.get<float>("Calibration.transform_ir_fx");
        ir_calib_params.fy = pt.get<float>("Calibration.transform_ir_fy");
        ir_calib_params.k1 = pt.get<float>("Calibration.transform_ir_k1");
        ir_calib_params.k2 = pt.get<float>("Calibration.transform_ir_k2");
        ir_calib_params.k3 = pt.get<float>("Calibration.transform_ir_k3");
        ir_calib_params.p1 = pt.get<float>("Calibration.transform_ir_p1");
        ir_calib_params.p2 = pt.get<float>("Calibration.transform_ir_p2");

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

cv::Mat Kinect::getMask()
{
    return *mask;
}

cv::Mat Kinect::getRangedRGBD()
{
    return *rangedRGBDMat;
}

cv::Mat Kinect::getRangedDepth()
{
    return *rangedDepthMat;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Kinect::getCloudData()
{
    return cloud;
}

void Kinect::cloudInit()
{
    cloud->clear();
    cloud->width = static_cast<uint32_t>(ir_depth_width);
    cloud->height = static_cast<uint32_t>(ir_depth_height);
    cloud->is_dense = false;
    cloud->points.resize( cloud->width * cloud->height);
}

void Kinect::rangeFrames(int lowTreshold,int highTreshold)
{
    rangedDepthMat->release();
    rangedRGBDMat->release();
    cv::Mat *tmpDepthMat=new cv::Mat(*depthMat);
    cv::Mat *tmpRGBDMat=new cv::Mat(*rgbdMat);

    tmpRGBDMat->convertTo(*tmpRGBDMat,CV_8UC3);
    cv::cvtColor(*tmpRGBDMat,*tmpRGBDMat,CV_RGBA2RGB);

    cv::inRange(*tmpDepthMat,lowTreshold,highTreshold,*mask);

    cv::Mat element = getStructuringElement( cv::MORPH_RECT,cv::Size( 2*1 + 1, 2*1+1 ),cv::Point( 1, 1 ) );
    /// Apply the dilation operation
    erode( *mask, *mask, element );

    tmpDepthMat->copyTo(*rangedDepthMat,*mask);
    tmpRGBDMat->copyTo(*rangedRGBDMat,*mask);

    tmpDepthMat->release();
    tmpRGBDMat->release();
    delete tmpDepthMat;
    delete tmpRGBDMat;




}

void Kinect::cloudData(std::atomic<bool> & keep_running, std::atomic<bool> & compute_cloud_style )
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

        if(compute_cloud_style==false)
            this->registered2cloud(tmpCloud);
        else
            this->depth2cloud(tmpCloud);


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



void Kinect::registered2cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &tmpCloud)
{
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
}


void Kinect::depth2cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &tmpCloud)
{

    double fy = 3.551063664968910e+02;
    double fx = 3.714166846959092e+02;
    double cy = 1.675200071820351e+02;
    double cx = 2.473181663244050e+02;

    rangeFrames(50,800);

    uint32_t n=0;
    double x=0, y=0, z=0;

    for (int y_side = 0; y_side < ir_depth_width; y_side++)
    {
        for (int x_side = 0; x_side < ir_depth_height; x_side++)
        {
            double Z = this->rangedDepthMat->at<float>(x_side, y_side) / 1;
            if(Z>0)
            {
                Z=1.0 / (Z * -0.0030711016 + 3.3309495161);
                x=(x_side-cx)*Z/fx;
                y=(y_side-cy)*Z/fy;
                z=Z;
            }
            else
            {
                x=NAN;
                y=NAN;
                z=NAN;
            }

            tmpCloud->points[n].x=x;
            tmpCloud->points[n].y=y;
            tmpCloud->points[n].z=z;
            tmpCloud->points[n].r=this->rangedRGBDMat->at<cv::Vec3b>(x_side,y_side)[2];
            tmpCloud->points[n].g=this->rangedRGBDMat->at<cv::Vec3b>(x_side,y_side)[1];
            tmpCloud->points[n].b=this->rangedRGBDMat->at<cv::Vec3b>(x_side,y_side)[0];
            n++;
        }
    }

    std::vector<int> removedPoints;
    pcl::removeNaNFromPointCloud(*tmpCloud,*tmpCloud,removedPoints);
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

bool Kinect::lockFrames(int lock_time)
{
    if(frame_mutex.try_lock_for(std::chrono::milliseconds(lock_time)))
        return true;
    else
        return false;
}

void Kinect::unlockFrames()
{
    frame_mutex.unlock();
}

Kinect::~Kinect()
{
    std::cout<<"END "<<this->id<<" "<<this->serial<<std::endl;
    this->pDev->stop();
    this->pDev->close();
    delete this->pRegistrated;
}


