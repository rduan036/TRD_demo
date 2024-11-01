#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFile>
#include <QDebug>
#include <QFileDialog>

#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/highgui.h>
#include <sstream>
#include <QPixmap>
#include <QImage>
#include <QLabel>
#include <QListWidgetItem>
#include <iterator>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
//    void on_CameraButton_clicked();

    void on_Load_dataset_botton_clicked();

//    void on_Load_init_file_botton_clicked();

    void on_Tracking_start_botton_clicked();

    void on_Tracking_Pause_botton_clicked();

    void on_Tracking_stop_botton_clicked();

    void on_Debug_SingleStep_clicked();

    void on_Change_result_botton_clicked();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
