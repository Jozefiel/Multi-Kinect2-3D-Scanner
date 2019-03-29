#include "kinect.h"

void Kinect::frames(std::atomic<bool> & keep_running)
{
    std::chrono::system_clock::time_point then, now;
    std::ofstream myfile;
    auto frame_counter=0;
    while(keep_running)
    {
        then=std::chrono::system_clock::now();
        if(this->pListener->waitForNewFrame(frame,camAttachTime))
        {
            if(frame_mutex.try_lock_for(std::chrono::milliseconds(frame_mutex_lock_time)))
            {
                libfreenect2::Frame *depth=frame[libfreenect2::Frame::Depth];
                libfreenect2::Frame *ir=frame[libfreenect2::Frame::Ir];
                libfreenect2::Frame *rgb=frame[libfreenect2::Frame::Color];

                cv::Mat interoplateDepth;
                cv::Mat(static_cast<int>(depth->height),  static_cast<int>(depth->width), CV_32FC1, depth->data).copyTo(interoplateDepth);


                this->pRegistrated->undistortDepth(depth,new_libfreenect_frames.undistortedDepth);
                cv::resize(interoplateDepth,interoplateDepth,cv::Size(static_cast<int>(depth->width),static_cast<int>(depth->height)),0,0,cv::INTER_LINEAR);
                depth->data = interoplateDepth.data;

                this->pRegistrated->apply(rgb,depth, new_libfreenect_frames.undistortedDepth, new_libfreenect_frames.registered,true, new_libfreenect_frames.depth2rgb);

                cv::Mat(static_cast<int>(ir->height), static_cast<int>(ir->width), CV_32FC1, ir->data).copyTo(this->new_cam_frames.irMat);
                cv::Mat(static_cast<int>(rgb->height), static_cast<int>(rgb->width), CV_8UC4, rgb->data).copyTo(this->new_cam_frames.colorMat);
                cv::Mat(static_cast<int>(new_libfreenect_frames.undistortedDepth->height),  static_cast<int>(new_libfreenect_frames.undistortedDepth->width), CV_32FC1, new_libfreenect_frames.undistortedDepth->data).copyTo(this->new_cam_frames.depthMat);
                cv::Mat(static_cast<int>(new_libfreenect_frames.registered->height),  static_cast<int>(new_libfreenect_frames.registered->width), CV_8UC4, new_libfreenect_frames.registered->data).copyTo(this->new_cam_frames.rgbdMat);

                now=std::chrono::system_clock::now();
                std::cout << "FRAME: "<<this->getId()<<" "<< std::chrono::duration_cast<std::chrono::milliseconds>(now - then).count() << " ms" << std::endl;

//                frame_counter++;
//                cv::imwrite ("test/" + serial + "/" + std::to_string(frame_counter) + "_rgbd.png",*this->new_cam_frames.rgbdMat);
//                cv::imwrite ("test/" + serial + "/" + std::to_string(frame_counter) + "_ir.exr",*this->new_cam_frames.irMat);
//                cv::imwrite ("test/" + serial + "/" + std::to_string(frame_counter) + "_depth.hdr",*this->new_cam_frames.depthMat);

//                myfile.open ("test/" + serial + "/" + std::to_string(frame_counter) + "_depth.txt");
//                for(auto i=0;i<new_libfreenect_frames.undistortedDepth->height*new_libfreenect_frames.undistortedDepth->width;i++)
//                {
//                    myfile << std::to_string(new_libfreenect_frames.undistortedDepth->data[i]) + "\n";
//                }
//                myfile.close();

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
        if(frame_mutex.try_lock_for(std::chrono::milliseconds(frame_mutex_lock_time)))
        {
        cam_frames = new_cam_frames;
        frame_mutex.unlock();
        }
    }
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
                pcl::copyPointCloud(*tmpCloud,*cam_frames.cloud);
                cloud_mutex.unlock();
            }
        }
        tmpCloud->clear();
        now=std::chrono::system_clock::now();
        std::cout << "CLOUD: "<<this->getId()<<" "<< std::chrono::duration_cast<std::chrono::milliseconds>(now - then).count() << " ms" << std::endl;
    }
}

void Kinect::cloudInit(size_t size)
{
    cam_frames.cloud->clear();
    cam_frames.cloud->is_dense = false;
    cam_frames.cloud->points.resize(size);
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

    float fx = pt.get<float>("Calibration.depth_fx");
    float fy = pt.get<float>("Calibration.depth_fy");
    float cx = pt.get<float>("Calibration.depth_cx");
    float cy = pt.get<float>("Calibration.depth_cy");
    float scale = pt.get<float>("Calibration.depth_s");

    uint32_t n=0;
    double x=0, y=0, z=0;


    for (int y_side = 0; y_side < ir_depth_width; y_side++)
    {
        for (int x_side = 0; x_side < ir_depth_height; x_side++)
        {
            double Z = this->cam_frames.rangedDepthMat.at<float>(x_side, y_side) / 1;
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
            tmpCloud->points[n].r=this->cam_frames.rangedRGBDMat.at<cv::Vec3b>(x_side,y_side)[2];
            tmpCloud->points[n].g=this->cam_frames.rangedRGBDMat.at<cv::Vec3b>(x_side,y_side)[1];
            tmpCloud->points[n].b=this->cam_frames.rangedRGBDMat.at<cv::Vec3b>(x_side,y_side)[0];
            n++;
        }
    }

    std::vector<int> removedPoints;
    pcl::removeNaNFromPointCloud(*tmpCloud,*tmpCloud,removedPoints);
}