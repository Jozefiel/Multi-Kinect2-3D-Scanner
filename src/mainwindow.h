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

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/features/normal_3d.h>
#include <vtkRenderWindow.h>
#include <pcl/point_types.h>


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
    void onNewRangedRGBD(QPixmap newFrm,int i);
    void onNewRangedDepth(QPixmap newFrm,int i);
    void onNewIR(QPixmap newFrm, int i);
    void onNewHist(QPixmap newFrm, int i);
    void onNewCloud();

    void on_pushButton_clicked();
    void on_pushButton_2_clicked();
    void on_checkBox_stateChanged(int arg1);
    void on_save_all_button_clicked();
    void on_sequence_stop_clicked();

    void on_cam_1_toggled(bool checked);

    void on_cam_2_toggled(bool checked);

    void on_cam_3_toggled(bool checked);

    void on_cam_4_toggled(bool checked);

    void on_recalibration_clicked(bool checked);

protected:
    std::vector<support*> Support;
    QGraphicsPixmapItem rgbd[4],depth[4],ir[4],ranged_rgbd[4],ranged_depth[4],histogram[4];
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ethalon = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::visualization::PCLVisualizer  * viewer=nullptr;
private:
    Ui::MainWindow *ui;

};

#endif // MAINWINDOW_H
