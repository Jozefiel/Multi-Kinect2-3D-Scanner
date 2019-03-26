#include "camera.h"
#include "kinect.h"

void Kinect::filterFrames()
{
    cv::Mat *tmpDepthMat = new cv::Mat(new_libfreenect_frames.undistortedDepth->height, new_libfreenect_frames.undistortedDepth->width, CV_32FC1, new_libfreenect_frames.undistortedDepth->data);
    cv::Mat *tmpRGBDMat  = new cv::Mat(new_libfreenect_frames.registered->height,       new_libfreenect_frames.registered->width, CV_8UC4, new_libfreenect_frames.registered->data);
    cv::Mat *tmpMask     = new cv::Mat( cv::Mat::zeros(ir_depth_height, ir_depth_width, CV_THRESH_BINARY) );

    rangeFrames(tmpDepthMat, tmpRGBDMat,tmpMask);
    morphFrames(tmpDepthMat, tmpRGBDMat,tmpMask);

    //TODO mutex with viewer_updater
//    cam_frames.rangedDepthMat->release();
//    cam_frames.rangedRGBDMat->release();
//    cam_frames.mask->release();

    tmpMask->copyTo(*cam_frames.mask);
    tmpDepthMat->copyTo(*cam_frames.rangedDepthMat,*cam_frames.mask);
    tmpRGBDMat->copyTo(*cam_frames.rangedRGBDMat,*cam_frames.mask);

    tmpRGBDMat->convertTo(*tmpRGBDMat,CV_8UC4);
    cv::cvtColor(*tmpRGBDMat,*tmpRGBDMat,CV_RGB2RGBA);

    libfreenect_frames.undistortedDepth->data = cam_frames.rangedDepthMat->data;
    libfreenect_frames.registered->data = tmpRGBDMat->data;

//    libfreenect_frames.registered->data = new_cam_frames.rangedRGBDMat->data;

    tmpDepthMat->release();
    tmpRGBDMat->release();
    tmpMask->release();
    delete tmpDepthMat;
    delete tmpRGBDMat;
    delete tmpMask;
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
    histRange=nullptr;
}
