#include "camera.h"
#include "kinect.h"

#include <exception>


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
//        this->pDev->setColorCameraParams(rgb_calib_params);
        this->registration();

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
//        then=std::chrono::system_clock::now();
        if(this->pListener->waitForNewFrame(frame,camAttachTime))
        {
            if(frame_mutex.try_lock_for(std::chrono::milliseconds(frame_mutex_lock_time)))
            {
                libfreenect2::Frame *depth=frame[libfreenect2::Frame::Depth];
                libfreenect2::Frame *ir=frame[libfreenect2::Frame::Ir];
                libfreenect2::Frame *rgb=frame[libfreenect2::Frame::Color];

                this->pRegistrated->undistortDepth(depth,new_libfreenect_frames.undistortedDepth);
                this->pRegistrated->apply(rgb,depth, new_libfreenect_frames.undistortedDepth, new_libfreenect_frames.registered,true, new_libfreenect_frames.depth2rgb);

                cv::Mat(ir->height, ir->width, CV_32FC1, ir->data).copyTo(*this->new_cam_frames.irMat);
                cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(*this->new_cam_frames.colorMat);
                cv::Mat(new_libfreenect_frames.undistortedDepth->height,  new_libfreenect_frames.undistortedDepth->width, CV_32FC1, new_libfreenect_frames.undistortedDepth->data).copyTo(* this->new_cam_frames.depthMat);
                cv::Mat(new_libfreenect_frames.registered->height,  new_libfreenect_frames.registered->width, CV_8UC4, new_libfreenect_frames.registered->data).copyTo(* this->new_cam_frames.rgbdMat);

//              now=std::chrono::system_clock::now();
//              std::cout << "FRAME: "<<this->getId()<<" "<< std::chrono::duration_cast<std::chrono::milliseconds>(now - then).count() << " ms" << std::endl;

                pListener->release(frame);
                frame_mutex.unlock();
            }
        }
        else
        {
            std::cout<<this->id<<" hasn't new frame"<<std::endl;
        }
    }
}

void Kinect::cloneFrames()
{
    std::unique_lock<std::timed_mutex> frame_lock(frame_mutex,std::try_to_lock);
    if(!frame_lock)
    {

        memcpy(&new_cam_frames,&cam_frames,sizeof (cam_frames));
//        memcpy(&new_libfreenect_frames,&libfreenect_frames,sizeof (libfreenect_frames));

    }
}

void Kinect::loadCamParams()
{
    Eigen::Matrix4d transform_matrix;
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

        transform_matrix (0,0) = pt.get<float>("Calibration.transform_00");
        transform_matrix (0,1) = pt.get<float>("Calibration.transform_01");
        transform_matrix (0,2) = pt.get<float>("Calibration.transform_02");
        transform_matrix (0,3) = pt.get<float>("Calibration.transform_03");
        transform_matrix (1,0) = pt.get<float>("Calibration.transform_10");
        transform_matrix (1,1) = pt.get<float>("Calibration.transform_11");
        transform_matrix (1,2) = pt.get<float>("Calibration.transform_12");
        transform_matrix (1,3) = pt.get<float>("Calibration.transform_13");
        transform_matrix (2,0) = pt.get<float>("Calibration.transform_20");
        transform_matrix (2,1) = pt.get<float>("Calibration.transform_21");
        transform_matrix (2,2) = pt.get<float>("Calibration.transform_22");
        transform_matrix (2,3) = pt.get<float>("Calibration.transform_23");
        transform_matrix (3,0) = pt.get<float>("Calibration.transform_30");
        transform_matrix (3,1) = pt.get<float>("Calibration.transform_31");
        transform_matrix (3,2) = pt.get<float>("Calibration.transform_32");
        transform_matrix (3,3) = pt.get<float>("Calibration.transform_33");


    } catch (int e) {

        std::cout << "No init config found for: "<< serial << e <<std::endl;
        transform_matrix (0,0) = 1;
        transform_matrix (0,1) = 0;
        transform_matrix (0,2) = 0;
        transform_matrix (0,3) = 0;
        transform_matrix (1,0) = 0;
        transform_matrix (1,1) = 1;
        transform_matrix (1,2) = 0;
        transform_matrix (1,3) = 0;
        transform_matrix (2,0) = 0;
        transform_matrix (2,1) = 0;
        transform_matrix (2,2) = 1;
        transform_matrix (2,3) = 0;
        transform_matrix (3,0) = 0;
        transform_matrix (3,1) = 0;
        transform_matrix (3,2) = 0;
        transform_matrix (3,3) = 1;
    }
   transformation_matrix = transform_matrix;
}

void Kinect::cloudInit(size_t size)
{
    new_cam_frames.cloud->clear();
    new_cam_frames.cloud->is_dense = false;
    new_cam_frames.cloud->points.resize(size);
}

void Kinect::computeHist()
{
    int histSize = maximal_depth-minimal_depth;
    float range[] = { minimal_depth, maximal_depth }; //the upper boundary is exclusive
    const float* histRange = { range };
    bool uniform = true, accumulate = false;
    cv::Mat b_hist;
    cv::calcHist( cam_frames.rangedDepthMat, 1, nullptr, cv::Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate );
    int hist_w = 512; int hist_h = 424;
    int bin_w = cvRound( (double) hist_w/histSize );
    cv::Mat histImage( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );
    cv::normalize(b_hist, b_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );
    for( int i = 1; i < histSize; i++ )
    {
        line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(b_hist.at<float>(i-1)) ) ,cv::Point( bin_w*(i), hist_h - cvRound(b_hist.at<float>(i)) ),cv::Scalar( 255, 0, 0), 2, 8, 0  );
    }
    histImage.copyTo(*cam_frames.histMat);
}

void Kinect::cloudData(std::atomic<bool> & keep_running, std::atomic<bool> & compute_cloud_style )
{
    std::chrono::system_clock::time_point then, now;
    Eigen::Matrix4d transform = transformation_matrix;

    while(keep_running)
    {
        then=std::chrono::system_clock::now();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        tmpCloud->width = static_cast<uint32_t>(ir_depth_width);
        tmpCloud->height = static_cast<uint32_t>(ir_depth_height);
        tmpCloud->is_dense = false;
        tmpCloud->points.resize( tmpCloud->width* tmpCloud->height);

        cloneFrames();
        computeHist();
        filterFrames();
        if(compute_cloud_style==false)
            this->registered2cloud(tmpCloud);
        else
            this->depth2cloud(tmpCloud);

        pcl::transformPointCloud(*tmpCloud,*tmpCloud,transformation_matrix,true);

        if(!tmpCloud->empty())
        {
            if(cloud_mutex.try_lock_for(std::chrono::milliseconds(mutex_lock_time)))
            {
                cloudInit(tmpCloud->points.size());
                pcl::copyPointCloud(*tmpCloud,*new_cam_frames.cloud);
                cloud_mutex.unlock();
            }
        }
        tmpCloud->clear();
//        now=std::chrono::system_clock::now();
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
            pRegistrated->getPointXYZRGB(libfreenect_frames.undistortedDepth,libfreenect_frames.registered,y_side,x_side,x,y,z,rgb);

//            std::cout<<x<<" "<<y<<""<<z<<std::endl;
            const uint8_t *p = reinterpret_cast<uint8_t*>(&rgb);
            uint8_t b = p[0];
            uint8_t g = p[1];
            uint8_t r = p[2];

//          if(this->cam_frames.mask->at<bool>(y_side,x_side)==true)

//            if((r<150 && b <150 && g<150) && (r >10 && b >10 && g >10))
//            {
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
//            }
//            else
//            {
//                x=NAN;
//                z=NAN;
//                y=NAN;
//            }

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

//    double fy = 3.551063664968910e+02;
//    double fx = 3.714166846959092e+02;
//    double cy = 1.675200071820351e+02;
//    double cx = 2.473181663244050e+02;

    double fx = pt.get<float>("Calibration.depth_fx");
    double fy = pt.get<float>("Calibration.depth_fy");
    double cx = pt.get<float>("Calibration.depth_cx");
    double cy = pt.get<float>("Calibration.depth_cy");
    double scale = pt.get<float>("Calibration.depth_s");

    uint32_t n=0;
    double x=0, y=0, z=0;


    for (int y_side = 0; y_side < ir_depth_width; y_side++)
    {
        for (int x_side = 0; x_side < ir_depth_height; x_side++)
        {
            double Z = this->new_cam_frames.rangedDepthMat->at<float>(x_side, y_side) / 1;
            if(Z>0)
            {
                Z= scale /(Z * -0.0030711016 + 3.3309495161);
                //Z=1*tan(Z/-0.0871 + 0.0549);
                //for common calibration -> registration2cloud x is swaped with y
                x=(y_side-cy)*Z/fy;
                y=(x_side-cx)*Z/fx;
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
            tmpCloud->points[n].r=this->new_cam_frames.rangedRGBDMat->at<cv::Vec3b>(x_side,y_side)[2];
            tmpCloud->points[n].g=this->new_cam_frames.rangedRGBDMat->at<cv::Vec3b>(x_side,y_side)[1];
            tmpCloud->points[n].b=this->new_cam_frames.rangedRGBDMat->at<cv::Vec3b>(x_side,y_side)[0];
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

