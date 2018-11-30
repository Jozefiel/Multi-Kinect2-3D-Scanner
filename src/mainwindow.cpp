#include "mainwindow.h"
#include "ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent) :QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setWindowTitle ("3D OSAS");

    Support=new support();
    Support->cameraInit();                                       //! camera init, create connection with Kinects and Realsenses

    ui->graphicsView_rgbd_0->setScene(new QGraphicsScene(this));
    ui->graphicsView_depth_0->setScene(new QGraphicsScene(this));
    ui->graphicsView_ir_0->setScene(new QGraphicsScene(this));

    ui->graphicsView_rgbd_1->setScene(new QGraphicsScene(this));
    ui->graphicsView_depth_1->setScene(new QGraphicsScene(this));
    ui->graphicsView_ir_1->setScene(new QGraphicsScene(this));

    ui->graphicsView_rgbd_2->setScene(new QGraphicsScene(this));
    ui->graphicsView_depth_2->setScene(new QGraphicsScene(this));
    ui->graphicsView_ir_2->setScene(new QGraphicsScene(this));

    connect(Support, SIGNAL(newRGBD(QPixmap,int)), this, SLOT(onNewRGBD(QPixmap,int)));
    connect(Support, SIGNAL(newDepth(QPixmap,int)), this, SLOT(onNewDepth(QPixmap,int)));
    connect(Support, SIGNAL(newIR(QPixmap,int)), this, SLOT(onNewIR(QPixmap,int)));
    connect(Support, SIGNAL(newCloud()), this, SLOT(onNewCloud()));

    ui->graphicsView_rgbd_0->scene()->addItem(&rgbd[0]);
    ui->graphicsView_depth_0->scene()->addItem(&depth[0]);
    ui->graphicsView_ir_0->scene()->addItem(&ir[0]);
    ui->graphicsView_rgbd_1->scene()->addItem(&rgbd[1]);
    ui->graphicsView_depth_1->scene()->addItem(&depth[1]);
    ui->graphicsView_ir_1->scene()->addItem(&ir[1]);
    ui->graphicsView_rgbd_2->scene()->addItem(&rgbd[2]);
    ui->graphicsView_depth_2->scene()->addItem(&depth[2]);
    ui->graphicsView_ir_2->scene()->addItem(&ir[2]);

    viewer=new pcl::visualization::PCLVisualizer("test",false);
    ui->qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
    ui->qvtkWidget->update ();
}

MainWindow::~MainWindow()
{
    delete ui;
    Support->closeThreads();
}

void MainWindow::onNewCloud()
{
    std::cout<<"MainWindow::onNewCloud new cloud for viewer"<<std::endl;
    pcl::copyPointCloud(Support->cloudik,*cloud);
    viewer->updatePointCloud(cloud, "cloud");
    ui->qvtkWidget->update();
}

void MainWindow::onNewRGBD(QPixmap newFrm,int i)
{
    rgbd[i].setPixmap(newFrm);
}

void MainWindow::onNewDepth(QPixmap newFrm,int i)
{
    depth[i].setPixmap(newFrm);
}

void MainWindow::onNewIR(QPixmap newFrm,int i)
{
    ir[i].setPixmap(newFrm);
}

void MainWindow::on_pushButton_clicked()
{
    Support->snap_running=true;
    Support->cloudInit();
    Support->threadsInit();                                      //! threads init, detached snapping and cloud computing started
}

void MainWindow::on_pushButton_2_clicked()
{
    Support->closeThreads();
}
