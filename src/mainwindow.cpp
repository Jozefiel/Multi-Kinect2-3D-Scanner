#include "mainwindow.h"
#include "ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent) :QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setWindowTitle ("3D OSAS");

    Support.push_back(new support());
    Support[0]->snap_running=true;
    Support[0]->cameraInit();                                       //! camera init, create connection with Kinects and Realsenses
    Support[0]->cloudInit();
    Support[0]->threadsInit();


    ui->graphicsView_rgbd_0->setScene(new QGraphicsScene(this));
    ui->graphicsView_depth_0->setScene(new QGraphicsScene(this));
    ui->graphicsView_ir_0->setScene(new QGraphicsScene(this));
    ui->graphicsView_ranged_rgbd_0->setScene(new QGraphicsScene(this));
    ui->graphicsView_ranged_depth_0->setScene(new QGraphicsScene(this));
    ui->graphicsView_histogram_0->setScene(new QGraphicsScene(this));


    ui->graphicsView_rgbd_1->setScene(new QGraphicsScene(this));
    ui->graphicsView_depth_1->setScene(new QGraphicsScene(this));
    ui->graphicsView_ir_1->setScene(new QGraphicsScene(this));
    ui->graphicsView_ranged_rgbd_1->setScene(new QGraphicsScene(this));
    ui->graphicsView_ranged_depth_1->setScene(new QGraphicsScene(this));
    ui->graphicsView_histogram_1->setScene(new QGraphicsScene(this));


    ui->graphicsView_rgbd_2->setScene(new QGraphicsScene(this));
    ui->graphicsView_depth_2->setScene(new QGraphicsScene(this));
    ui->graphicsView_ir_2->setScene(new QGraphicsScene(this));
    ui->graphicsView_ranged_rgbd_2->setScene(new QGraphicsScene(this));
    ui->graphicsView_ranged_depth_2->setScene(new QGraphicsScene(this));
    ui->graphicsView_histogram_2->setScene(new QGraphicsScene(this));


    connect(Support[0], SIGNAL(newRGBD(QPixmap,int)), this, SLOT(onNewRGBD(QPixmap,int)));
    connect(Support[0], SIGNAL(newDepth(QPixmap,int)), this, SLOT(onNewDepth(QPixmap,int)));
    connect(Support[0], SIGNAL(newIR(QPixmap,int)), this, SLOT(onNewIR(QPixmap,int)));
    connect(Support[0], SIGNAL(newRangedRGBD(QPixmap,int)), this,  SLOT(onNewRangedRGBD(QPixmap,int)));
    connect(Support[0], SIGNAL(newRangedDepth(QPixmap,int)), this, SLOT(onNewRangedDepth(QPixmap,int)));
    connect(Support[0], SIGNAL(newHist(QPixmap,int)), this, SLOT(onNewHist(QPixmap,int)));


    connect(Support[0], SIGNAL(newCloud()), this, SLOT(onNewCloud()));

    ui->graphicsView_rgbd_0->scene()->addItem(&rgbd[0]);
    ui->graphicsView_depth_0->scene()->addItem(&depth[0]);
    ui->graphicsView_ir_0->scene()->addItem(&ir[0]);
    ui->graphicsView_ranged_depth_0->scene()->addItem(&ranged_depth[0]);
    ui->graphicsView_ranged_rgbd_0->scene()->addItem(&ranged_rgbd[0]);
    ui->graphicsView_histogram_0->scene()->addItem(&histogram[0]);


    ui->graphicsView_rgbd_1->scene()->addItem(&rgbd[1]);
    ui->graphicsView_depth_1->scene()->addItem(&depth[1]);
    ui->graphicsView_ir_1->scene()->addItem(&ir[1]);
    ui->graphicsView_ranged_depth_1->scene()->addItem(&ranged_depth[1]);
    ui->graphicsView_ranged_rgbd_1->scene()->addItem(&ranged_rgbd[1]);
    ui->graphicsView_histogram_1->scene()->addItem(&histogram[1]);


    ui->graphicsView_rgbd_2->scene()->addItem(&rgbd[2]);
    ui->graphicsView_depth_2->scene()->addItem(&depth[2]);
    ui->graphicsView_ir_2->scene()->addItem(&ir[2]);
    ui->graphicsView_ranged_depth_2->scene()->addItem(&ranged_depth[2]);
    ui->graphicsView_ranged_rgbd_2->scene()->addItem(&ranged_rgbd[2]);
    ui->graphicsView_histogram_2->scene()->addItem(&histogram[2]);

    viewer=new pcl::visualization::PCLVisualizer("viewer",false);
    ui->qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());

//    pcl::io::loadPLYFile("hlava002.ply",*ethalon);
//    viewer->addPointCloud(ethalon, "ethalon");
//    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "ethalon");
//    ui->qvtkWidget->update ();

    viewer->addPointCloud(Support[0]->getClouds()[0]->getMergedCloud(),"cloud");

}

MainWindow::~MainWindow()
{
    delete ui;
    Support[0]->closeThreads();
}

void MainWindow::onNewCloud()
{

//    viewer->removePointCloud("cloud");
//    viewer->removePointCloud("normals");
//    viewer->addPointCloud(Support[0]->getClouds()[0]->getMergedCloud(),"cloud");
    if(Support[0]->getClouds()[0]->getMergedCloud()!=nullptr && Support[0]->getClouds()[0]->getMergedCloud()->size()>0)
    {
        viewer->updatePointCloud(Support[0]->getClouds()[0]->getMergedCloud(),"cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2 , "cloud");
    }
//    Support[0]->merged_cloud->computeNormals();

//    viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(Support[0]->merged_cloud->getCloud(),Support[0]->merged_cloud->getCloudNormals(),10,0.03, "normals");



    ui->qvtkWidget->update();
}

void MainWindow::onNewRGBD(QPixmap newFrm,int i)
{
    if(!newFrm.isNull() && !newFrm.size().isEmpty())
        rgbd[i].setPixmap(newFrm);
}

void MainWindow::onNewDepth(QPixmap newFrm,int i)
{
    if(!newFrm.isNull() && !newFrm.size().isEmpty())
        depth[i].setPixmap(newFrm);
}

void MainWindow::onNewRangedRGBD(QPixmap newFrm,int i)
{
    if(!newFrm.isNull() && !newFrm.size().isEmpty())
        ranged_rgbd[i].setPixmap(newFrm);
}

void MainWindow::onNewRangedDepth(QPixmap newFrm,int i)
{
    if(!newFrm.isNull() && !newFrm.size().isEmpty())
        ranged_depth[i].setPixmap(newFrm);
}

void MainWindow::onNewIR(QPixmap newFrm,int i)
{
    if(!newFrm.isNull() && !newFrm.size().isEmpty())
        ir[i].setPixmap(newFrm);
}

void MainWindow::onNewHist(QPixmap newFrm,int i)
{
    if(!newFrm.isNull() && !newFrm.size().isEmpty())
        histogram[i].setPixmap(newFrm);
}


void MainWindow::on_pushButton_clicked()
{

    for(auto i=0;i<Support[0]->getConnectedCams().size();i++)
    {
        Support[0]->getConnectedCams()[i]->start();
    }
}

void MainWindow::on_pushButton_2_clicked()
{
    for(auto i=0;i<Support[0]->getConnectedCams().size();i++)
    {
        Support[0]->getConnectedCams()[i]->stop();
    }
}

void MainWindow::on_checkBox_stateChanged(int arg1)
{
    Support[0]->changeComputeStyle(arg1);
}

void MainWindow::on_save_all_button_clicked()
{

    Support[0]->saveData();

}

void MainWindow::on_sequence_stop_clicked()
{
    Support[0]->getConnectedCams()[1]->stop();
    Support[0]->getConnectedCams()[2]->stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    Support[0]->getConnectedCams()[0]->start();
    Support[0]->getConnectedCams()[0]->stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    Support[0]->getConnectedCams()[1]->start();
    Support[0]->getConnectedCams()[1]->stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    Support[0]->getConnectedCams()[2]->start();
    Support[0]->getConnectedCams()[2]->stop();

}


void MainWindow::on_cam_1_toggled(bool checked)
{
    if(checked)
    {
        Support[0]->getConnectedCams()[0]->stop();
    }
    else
    {
        Support[0]->getConnectedCams()[0]->start();
    }
}

void MainWindow::on_cam_2_toggled(bool checked)
{
    if(checked)
    {
        Support[0]->getConnectedCams()[1]->stop();
    }
    else
    {
        Support[0]->getConnectedCams()[1]->start();
    }
}

void MainWindow::on_cam_3_toggled(bool checked)
{
    if(checked)
    {
        Support[0]->getConnectedCams()[2]->stop();
    }
    else
    {
        Support[0]->getConnectedCams()[2]->start();
    }
}

void MainWindow::on_cam_4_toggled(bool checked)
{

}

void MainWindow::on_recalibration_clicked(bool checked)
{
    std::cout<<"recalibrating camera params"<<std::endl;
    for(int i=0; i < Support[0]->getConnectedCams().size();i++)
    {
        Support[0]->getConnectedCams()[i]->loadCamParams();
    }
}



