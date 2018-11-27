#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <iostream>
#include <string>
#include <thread>
#include <memory>
#include <chrono>
#include <fstream>

#include "camera.h"
#include "kinect.h"
#include "support.h"
#include "pcl.h"

//support support;
//std::auto_ptr<support> support(new class support);

MainWindow::MainWindow(QWidget *parent) :QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    Support=new support();
    Support->cameraInit();                                       //! camera init, create connection with Kinects and Realsenses

    ui->graphicsView_0->setScene(new QGraphicsScene(this));
    ui->graphicsView_1->setScene(new QGraphicsScene(this));
    ui->graphicsView_2->setScene(new QGraphicsScene(this));

    connect(Support, SIGNAL(newRGBD(QPixmap,int)), this, SLOT(onNewRGBD(QPixmap,int)));
    connect(Support, SIGNAL(newDepth(QPixmap,int)), this, SLOT(onNewDepth(QPixmap,int)));
    connect(Support, SIGNAL(newIR(QPixmap,int)), this, SLOT(onNewIR(QPixmap,int)));

    ui->graphicsView_0->scene()->addItem(&rgbd);
    ui->graphicsView_1->scene()->addItem(&depth);
    ui->graphicsView_2->scene()->addItem(&ir);
}

MainWindow::~MainWindow()
{
    delete ui;
    Support->closeThreads();
}

void MainWindow::onNewRGBD(QPixmap newFrm,int i)
{
    rgbd.setPixmap(newFrm);
}

void MainWindow::onNewDepth(QPixmap newFrm,int i)
{
    depth.setPixmap(newFrm);
}

void MainWindow::onNewIR(QPixmap newFrm,int i)
{
    ir.setPixmap(newFrm);
}

void MainWindow::on_pushButton_clicked()
{
    Support->snap_running=true;
    Support->threadsInit();                                      //! threads init, detached snapping and cloud computing started
    Support->cloudInit();
}

void MainWindow::on_pushButton_2_clicked()
{
    Support->closeThreads();
}
