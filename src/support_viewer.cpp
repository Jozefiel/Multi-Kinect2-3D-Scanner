#include "support.h"

void support::viewerUpdater(std::atomic<bool> &snap_running)
{
    cv::Mat tmpIR;
    cv::Mat tmpDepth;
    cv::Mat tmpHist;
    cv::Mat tmpRGBD;

    std::vector<Camera::camera_frames> frames0,frames1,frames2;
    auto i=0;



    while(snap_running)
    {
        for(auto connected_cams_number=0;connected_cams_number<connected_cams.size();connected_cams_number++)
        {
            tmpDepth=connected_cams[connected_cams_number]->getDepth()/8;
            tmpDepth.convertTo(tmpDepth,CV_8UC1);

            tmpIR=connected_cams[connected_cams_number]->getIR()/64;
            tmpIR.convertTo(tmpIR, CV_8UC1);

            tmpRGBD=connected_cams[connected_cams_number]->getRGBD();
            cv::cvtColor(tmpRGBD,tmpRGBD,CV_RGBA2RGB);

            emit newRGBD(QPixmap::fromImage(QImage(tmpRGBD.data,tmpRGBD.cols,tmpRGBD.rows,tmpRGBD.step,QImage::Format_RGB888).rgbSwapped()),connected_cams_number);
            emit newDepth(QPixmap::fromImage(QImage(tmpDepth.data,tmpDepth.cols,tmpDepth.rows,tmpDepth.step,QImage::Format_Indexed8)),connected_cams_number);
            emit newIR(QPixmap::fromImage(QImage(tmpIR.data,tmpIR.cols,tmpIR.rows,tmpIR.step,QImage::Format_Indexed8)),connected_cams_number);
            if(!connected_cams[connected_cams_number]->getRangedRGBD().empty())
            {
                emit newRangedRGBD(QPixmap::fromImage(QImage(connected_cams[connected_cams_number]->getRangedRGBD().data,connected_cams[connected_cams_number]->getRangedRGBD().cols,connected_cams[connected_cams_number]->getRangedRGBD().rows,connected_cams[connected_cams_number]->getRangedRGBD().step,QImage::Format_RGB888).rgbSwapped()),connected_cams_number);
            }
            if(!connected_cams[connected_cams_number]->getRangedDepth().empty())
            {
                tmpDepth=connected_cams[connected_cams_number]->getRangedDepth()/8;
                tmpDepth.convertTo(tmpDepth,CV_8UC1);
                emit newRangedDepth(QPixmap::fromImage(QImage(tmpDepth.data,tmpDepth.cols,tmpDepth.rows,tmpDepth.step,QImage::Format_Indexed8)),connected_cams_number);
            }
            if(!connected_cams[connected_cams_number]->getHistogram().empty())
            {
                tmpHist=connected_cams[connected_cams_number]->getHistogram();
                tmpHist.convertTo(tmpHist, CV_8UC3);
                emit newHist(QPixmap::fromImage(QImage(tmpHist.data,tmpHist.cols,tmpHist.rows,tmpHist.step,QImage::Format_RGB888)),connected_cams_number);
            }


         //   std::this_thread::sleep_for(std::chrono::milliseconds(10));
       }
    }
}


void support::pclUpdater(std::atomic<bool> &snap_running)
{
    merged_cloud= new pclCloud(4);
    while(snap_running)
    {
        this->camera2cloudDataTransfer();   // store cloud to pcl objects
        merged_cloud->mergeClouds(this->mergeClouds(false));//! error when true, random fallings

        if(!merged_cloud->getCloud().empty())
            emit newCloud();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

    }
}


void support::frameUpdater(std::atomic<bool> &snap_running)
{
    while(snap_running)
    {
        this->camera2framesDataTransfer();   // store cloud to pcl objects
    }
}
