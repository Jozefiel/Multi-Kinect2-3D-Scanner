#include "camera.h"
#include "kinect.h"

Camera * Camera::create_camera(libfreenect2::Freenect2 * _freenect, int id)
{
    return new Kinect(_freenect,id);
}

int  Camera::getId() { return id; }                                            // return information about camera
std::string  Camera::getSerial() { return serial; }
std::string  Camera::getCamType()  { return camera_type;}


void Camera::rangeFrames(cv::Mat  tmpDepthMat, cv::Mat  tmpRGBDMat, cv::Mat  tmpMask)
{
    tmpRGBDMat.convertTo(tmpRGBDMat,CV_8UC3);
    cv::cvtColor(tmpRGBDMat,tmpRGBDMat,cv::COLOR_RGBA2RGB);
    cv::inRange(tmpDepthMat,globalSettings.operator->()->getMinDepth(),globalSettings.operator->()->getMaxDepth(),tmpMask);
}

void Camera::morphFrames(cv::Mat & tmpDepthMat, cv::Mat &tmpRGBDMat, cv::Mat &tmpMask)
{
    cv::Mat element = getStructuringElement( cv::MORPH_RECT,cv::Size( 4*1 + 1, 4*1+1 ),cv::Point( 4, 4 ) );
    cv::erode( tmpMask, tmpMask, element );

    cv::Mat tmpBilateralDepthMat;
    cv::bilateralFilter( tmpDepthMat, tmpBilateralDepthMat, 7, 15, 15 );
}

void Camera::faceDetection(cv::Mat  rangedRGBD, cv::Mat rgbd)
{

}

Camera::~Camera()
{

}
