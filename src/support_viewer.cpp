#include "support.h"

void support::viewerUpdater()
{
    cv::Mat tmpIR;
    cv::Mat tmpDepth;
    cv::Mat tmpHist;
    cv::Mat tmpRGBD;

    std::vector<Camera::camera_frames> frames0,frames1,frames2;
    try {
        for(ulong connected_cams_number=0;connected_cams_number<connected_cams.size();connected_cams_number++)
        {
            if(cam_frames->at(connected_cams_number).size()>globalSettings.operator->()->getBufferSize()-1)
            {
                tmpDepth=cam_frames->at(connected_cams_number).back().depthMat.clone()/8;
                tmpDepth.convertTo(tmpDepth,CV_8UC1);

                tmpIR=cam_frames->at(connected_cams_number).back().irMat.clone()/64;
                tmpIR.convertTo(tmpIR, CV_8UC1);

                tmpRGBD=cam_frames->at(connected_cams_number).back().rgbdMat.clone();
                cv::cvtColor(tmpRGBD,tmpRGBD,CV_RGBA2RGB);

                emit newRGBD(QPixmap::fromImage(QImage(tmpRGBD.data,tmpRGBD.cols,tmpRGBD.rows,tmpRGBD.step,QImage::Format_RGB888).rgbSwapped()),connected_cams_number);
                emit newDepth(QPixmap::fromImage(QImage(tmpDepth.data,tmpDepth.cols,tmpDepth.rows,tmpDepth.step,QImage::Format_Indexed8)),connected_cams_number);
                emit newIR(QPixmap::fromImage(QImage(tmpIR.data,tmpIR.cols,tmpIR.rows,tmpIR.step,QImage::Format_Indexed8)),connected_cams_number);

                if(!cam_frames->at(connected_cams_number).back().rangedRGBDMat.empty())
                {
                    emit newRangedRGBD(QPixmap::fromImage(QImage(cam_frames->at(connected_cams_number).back().rangedRGBDMat.data,cam_frames->at(connected_cams_number).back().rangedRGBDMat.cols,cam_frames->at(connected_cams_number).back().rangedRGBDMat.rows,cam_frames->at(connected_cams_number).back().rangedRGBDMat.step,QImage::Format_RGBA8888).rgbSwapped()),connected_cams_number);
                }
                if(!cam_frames->at(connected_cams_number).back().rangedDepthMat.empty())
                {
                    tmpDepth=cam_frames->at(connected_cams_number).back().rangedDepthMat.clone()/8;
                    tmpDepth.convertTo(tmpDepth,CV_8UC1);
                    emit newRangedDepth(QPixmap::fromImage(QImage(tmpDepth.data,tmpDepth.cols,tmpDepth.rows,tmpDepth.step,QImage::Format_Indexed8)),connected_cams_number);
                }
                if(!cam_frames->at(connected_cams_number).back().histMat.empty())
                {
                    tmpHist=cam_frames->at(connected_cams_number).back().histMat;
                    tmpHist.convertTo(tmpHist, CV_8UC3);
                    emit newHist(QPixmap::fromImage(QImage(tmpHist.data,tmpHist.cols,tmpHist.rows,tmpHist.step,QImage::Format_RGB888)),connected_cams_number);
                }
            }
       }
    }  catch (...) { }
}

void support::pclUpdater()
{
        if(this->clouds[0]->mergeLastClouds())
            std::cout<<"emited"<<std::endl;
            emit newCloud();
}

void support::frameUpdater(std::atomic<bool> &snap_running)
{
    while(snap_running)
    {
        if(this->camera2framesDataTransfer() && cam_frames->at(connectedCameras()-1).size()>globalSettings->getBufferSize()-1)
        {
            std::thread(&support::viewerUpdater,this).detach();
//            std::thread(&support::framesClouds2pclDataTransfer,this).detach();
//            std::thread(&support::pclUpdater,this).detach();
//            viewerUpdater();
            framesClouds2pclDataTransfer();
            pclUpdater();
        }  // store cloud to pcl objects
    }
}
