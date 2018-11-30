#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsItem>
#include <QGraphicsView>
#include <QDebug>
#include <QPixmap>
#include <QVTKWidget.h>

#include <iostream>
#include <string>
#include <thread>
#include <memory>
#include <chrono>
#include <fstream>

#include <vtkRenderWindow.h>

#include "camera.h"
#include "kinect.h"
#include "support.h"
#include "pcl.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onNewRGBD(QPixmap newFrm,int i);
    void onNewDepth(QPixmap newFrm,int i);
    void onNewIR(QPixmap newFrm, int i);
    void onNewCloud();

    void on_pushButton_clicked();
    void on_pushButton_2_clicked();

protected:
     support * Support=nullptr;
     QGraphicsPixmapItem rgbd[4],depth[4],ir[4];
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
     pcl::visualization::PCLVisualizer  * viewer=nullptr;

private:
    Ui::MainWindow *ui;

};

#endif // MAINWINDOW_H
