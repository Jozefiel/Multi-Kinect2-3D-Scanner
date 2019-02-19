#include "camera.h"
#include "kinect.h"

void Kinect::filterFrames()
{
    cam_frames.rangedDepthMat->release();
    cam_frames.rangedRGBDMat->release();

    cv::Mat *tmpDepthMat = new cv::Mat(new_libfreenect_frames.undistortedDepth->height,  new_libfreenect_frames.undistortedDepth->width, CV_32FC1, new_libfreenect_frames.undistortedDepth->data);
    cv::Mat *tmpRGBDMat  = new cv::Mat(new_libfreenect_frames.registered->height,  new_libfreenect_frames.registered->width, CV_8UC4, new_libfreenect_frames.registered->data);

    rangeFrames(tmpDepthMat, tmpRGBDMat);

    cv::Mat element = getStructuringElement( cv::MORPH_RECT,cv::Size( 4*1 + 1, 4*1+1 ),cv::Point( 4, 4 ) );
    /// Apply the dilation operation
    cv::erode( *cam_frames.mask, *cam_frames.mask, element );

    cv::Mat tmpBilateralDepthMat;
    cv::bilateralFilter( *tmpDepthMat, tmpBilateralDepthMat, 7, 15, 15 );

    tmpDepthMat->copyTo(*cam_frames.rangedDepthMat,*cam_frames.mask);
    tmpRGBDMat->copyTo(*cam_frames.rangedRGBDMat,*cam_frames.mask);

    tmpRGBDMat->convertTo(*tmpRGBDMat,CV_8UC4);
    cv::cvtColor(*tmpRGBDMat,*tmpRGBDMat,CV_RGB2RGBA);

    libfreenect_frames.undistortedDepth->data = cam_frames.rangedDepthMat->data;
    libfreenect_frames.registered->data = tmpRGBDMat->data;

//    libfreenect_frames.registered->data         = new_cam_frames.rangedRGBDMat->data;

    tmpDepthMat->release();
    tmpRGBDMat->release();
    delete tmpDepthMat;
    delete tmpRGBDMat;
}

void Kinect::rangeFrames(cv::Mat * tmpDepthMat, cv::Mat * tmpRGBDMat)
{
    tmpRGBDMat->convertTo(*tmpRGBDMat,CV_8UC3);
    cv::cvtColor(*tmpRGBDMat,*tmpRGBDMat,CV_RGBA2RGB);
    cv::inRange(*tmpDepthMat,80,800,*cam_frames.mask);
}

