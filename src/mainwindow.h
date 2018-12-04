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

    void on_pushButton_3_clicked();

    void on_horizontalSlider_actionTriggered(int action);

    void on_checkBox_toggled(bool checked);

    void on_checkBox_stateChanged(int arg1);

    void on_pushButton_7_clicked();

    void on_save_all_button_clicked();

    void on_sequence_stop_clicked();

protected:
    std::vector<support*> Support;
    QGraphicsPixmapItem rgbd[4],depth[4],ir[4];
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::visualization::PCLVisualizer  * viewer=nullptr;
    int saved_frame_counter=0;
private:
    Ui::MainWindow *ui;

};

#endif // MAINWINDOW_H
