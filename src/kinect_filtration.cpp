#include "camera.h"
#include "kinect.h"

void Kinect::filterFrames()
{
    cv::Mat *tmpDepthMat = new cv::Mat(new_libfreenect_frames.undistortedDepth->height,  new_libfreenect_frames.undistortedDepth->width, CV_32FC1, new_libfreenect_frames.undistortedDepth->data);
    cv::Mat *tmpRGBDMat  = new cv::Mat(new_libfreenect_frames.registered->height,  new_libfreenect_frames.registered->width, CV_8UC4, new_libfreenect_frames.registered->data);
    cv::Mat *tmpMask     = new cv::Mat( cv::Mat::zeros(ir_depth_height, ir_depth_width, CV_THRESH_BINARY) );

    rangeFrames(tmpDepthMat, tmpRGBDMat,tmpMask);
    morphFrames(tmpDepthMat, tmpRGBDMat,tmpMask);

    cam_frames.rangedDepthMat->release();
    cam_frames.rangedRGBDMat->release();
    cam_frames.mask->release();

    tmpMask->copyTo(*cam_frames.mask);
    tmpDepthMat->copyTo(*cam_frames.rangedDepthMat,*cam_frames.mask);
    tmpRGBDMat->copyTo(*cam_frames.rangedRGBDMat,*cam_frames.mask);

    tmpRGBDMat->convertTo(*tmpRGBDMat,CV_8UC4);
    cv::cvtColor(*tmpRGBDMat,*tmpRGBDMat,CV_RGB2RGBA);

    libfreenect_frames.undistortedDepth->data = cam_frames.rangedDepthMat->data;
    libfreenect_frames.registered->data = tmpRGBDMat->data;

//    libfreenect_frames.registered->data         = new_cam_frames.rangedRGBDMat->data;

    tmpDepthMat->release();
    tmpRGBDMat->release();
    tmpMask->release();
    delete tmpDepthMat;
    delete tmpRGBDMat;
    delete tmpMask;
}


