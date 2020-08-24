#include "camera.h"
#include "kinect.h"
void Kinect::filterFrames()
{

    auto tmpDepthMat = std::make_shared<cv::Mat>(new_cam_frames.depthMat);
    auto tmpRGBDMat  = std::make_shared<cv::Mat>(new_cam_frames.rgbdMat);
    auto tmpMask     = std::make_shared<cv::Mat>(cv::Mat::zeros(ir_depth_height, ir_depth_width, cv::THRESH_BINARY));

    rangeFrames(*tmpDepthMat, *tmpRGBDMat,*tmpMask);
    morphFrames(*tmpDepthMat, *tmpRGBDMat,*tmpMask);

    new_cam_frames.rangedDepthMat.release();
    new_cam_frames.rangedRGBDMat.release();
    new_cam_frames.mask.release();

    tmpMask->copyTo(new_cam_frames.mask);
    tmpDepthMat->copyTo(new_cam_frames.rangedDepthMat,new_cam_frames.mask);
    tmpRGBDMat->copyTo(new_cam_frames.rangedRGBDMat,new_cam_frames.mask);

    tmpRGBDMat->convertTo(*tmpRGBDMat,CV_8UC4);
    cv::cvtColor(*tmpRGBDMat,*tmpRGBDMat,cv::COLOR_RGB2RGBA);

    cv::flip(new_cam_frames.rangedDepthMat,new_cam_frames.rangedDepthMat,+1);
    cv::flip(new_cam_frames.rangedRGBDMat,new_cam_frames.rangedRGBDMat,+1);

    libfreenect_frames.undistortedDepth->data = new_cam_frames.rangedDepthMat.data;
    libfreenect_frames.registered->data = new_cam_frames.rangedRGBDMat.data;

    libfreenect_frames.registered->data         = new_cam_frames.rangedRGBDMat.data;

//    tmpDepthMat->release();
//    tmpRGBDMat->release();
//    tmpMask->release();

}

void Kinect::computeHist()
{
    int histSize = maximal_depth-minimal_depth;
    float range[] = { minimal_depth, maximal_depth }; //the upper boundary is exclusive
    const float* histRange = { range };
    bool uniform = true, accumulate = false;
    cv::Mat b_hist;
    cv::calcHist( &cam_frames.rangedDepthMat, 1, nullptr, cv::Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate );
    int hist_w = 512; int hist_h = 424;
    int bin_w = cvRound( static_cast<double>(hist_w/histSize ));
    cv::Mat histImage( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );
    cv::normalize(b_hist, b_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );
    for( int i = 1; i < histSize; i++ )
    {
        line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(b_hist.at<float>(i-1)) ) ,cv::Point( bin_w*(i), hist_h - cvRound(b_hist.at<float>(i)) ),cv::Scalar( 255, 0, 0), 2, 8, 0  );
    }
    histImage.copyTo(cam_frames.histMat);
    histRange=nullptr;
}
