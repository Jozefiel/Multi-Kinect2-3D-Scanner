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

        transform_matrix (0,0) = pt.get<double>("Calibration.transform_00");
        transform_matrix (0,1) = pt.get<double>("Calibration.transform_01");
        transform_matrix (0,2) = pt.get<double>("Calibration.transform_02");
        transform_matrix (0,3) = pt.get<double>("Calibration.transform_03");
        transform_matrix (1,0) = pt.get<double>("Calibration.transform_10");
        transform_matrix (1,1) = pt.get<double>("Calibration.transform_11");
        transform_matrix (1,2) = pt.get<double>("Calibration.transform_12");
        transform_matrix (1,3) = pt.get<double>("Calibration.transform_13");
        transform_matrix (2,0) = pt.get<double>("Calibration.transform_20");
        transform_matrix (2,1) = pt.get<double>("Calibration.transform_21");
        transform_matrix (2,2) = pt.get<double>("Calibration.transform_22");
        transform_matrix (2,3) = pt.get<double>("Calibration.transform_23");
        transform_matrix (3,0) = pt.get<double>("Calibration.transform_30");
        transform_matrix (3,1) = pt.get<double>("Calibration.transform_31");
        transform_matrix (3,2) = pt.get<double>("Calibration.transform_32");
        transform_matrix (3,3) = pt.get<double>("Calibration.transform_33");


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

