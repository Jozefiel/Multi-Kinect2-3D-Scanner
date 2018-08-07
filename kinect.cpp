#include "kinect.h"

#define camAttachTime 1000
#define ir_depth_width 512
#define ir_depth_height 424
#define ir_depth_bpp 4
#define color_width 1920
#define color_height 1080+2
#define color_bpp 4

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


void Kinect::frames()
{
        listener->waitForNewFrame(frame,camAttachTime);
        libfreenect2::Frame *depth=frame[libfreenect2::Frame::Depth];
        libfreenect2::Frame *ir=frame[libfreenect2::Frame::Ir];
        libfreenect2::Frame *rgb=frame[libfreenect2::Frame::Color];

        cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(*depthMat);
        cv::Mat(ir->height, ir->width, CV_32FC1, ir->data).copyTo(*irMat);
        cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(*colorMat);

        libfreenect2::Frame *undistorted = new libfreenect2::Frame(ir_depth_width, ir_depth_height, ir_depth_bpp);
        libfreenect2::Frame *registered = new libfreenect2::Frame(ir_depth_width, ir_depth_height, ir_depth_bpp);
        libfreenect2::Frame *depth2rgb = new libfreenect2::Frame(color_width,color_height, color_bpp);

        registrated->apply(rgb,depth,undistorted,registered,true,depth2rgb);
        cv::Mat(registered->height, registered->width, CV_8UC4, registered->data).copyTo(*rgbdMat);

        listener->release(frame);
}

std::string Kinect::getSerial()
{
    std::string _serial = serial;
    return _serial;
}

std::string Kinect::getId()
{
    std::stringstream id_string;
    id_string << kinect_id;

    return id_string.str();
}


void Kinect::getDepth()
{
    cv::imshow("depth: " + getId(), *depthMat / 2048);
}

void Kinect::rangedDepth()
{
    depthControl();
    rangeMask->release();
    cv::inRange(*depthMat,low_slider/10,high_slider/10,*rangeMask);
    depthMat->copyTo(*rangedDepthMap,*rangeMask);
}

void Kinect::getRangedDepth()
{
    rangedDepthMap->release();
    rangedDepth();
    cv::imshow("rDepth: " + getId() , *rangedDepthMap / 2048);
}

void Kinect::getRGB()
{
    cv::imshow("color: " + getId() , *colorMat);
}

void Kinect::getIr()
{
    cv::imshow("ir: " + getId(), *irMat);
}

void Kinect::getRGBD()
{
    cv::imshow("rgbd: " + getId(), *rgbdMat );
}

void Kinect::rangedRGBD()
{
    cv::imshow("ranged: " + getId(), *rangeMask );
    rgbdMat->copyTo(*rangedRGBDMat,*rangeMask);
}

void Kinect::getRangedRGBD()
{
    rangedRGBDMat->release();
    rangedRGBD();
    cv::imshow("rRGBD: " + getId(), *rangedRGBDMat );
}

void Kinect::depthControl()
{
    cv::createTrackbar( "Low Depth range", "rDepth: " + getId(), &low_slider, 40950);
    cv::createTrackbar( "High Depth range","rDepth: " + getId(), &high_slider, 40950);
}

void Kinect::cloudInit()
{
    cloud->width = static_cast<uint32_t>(ir_depth_width);
    cloud->height = static_cast<uint32_t>(ir_depth_height);
    cloud->is_dense = false;
    cloud->points.resize( cloud->width* cloud->height);
}

void Kinect::cloudData()
{
    for(int x_side=0;x_side<ir_depth_width;x_side++)
    {
        for(int y_side=0;y_side<ir_depth_height;y_side++)
        {
            float rgb;
            registrated->getPointXYZRGB(r,c,x,y,z,rgb);
        }

    }
}


Kinect::~Kinect()
{
    std::cout<<"END "<<dev->getSerialNumber()<<std::endl;
    dev->stop();
    dev->close();
    delete registrated;

}


