#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsItem>
#include <QGraphicsView>
#include <QDebug>
#include <QPixmap>

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

    void on_pushButton_clicked();
    void on_pushButton_2_clicked();

private:
    Ui::MainWindow *ui;
    support *Support=nullptr;
    QGraphicsPixmapItem rgbd,depth,ir;

};

#endif // MAINWINDOW_H
