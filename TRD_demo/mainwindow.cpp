#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "TRD_Tracker.h"
#include "slic.h"
#include "general_function.h"

#define KEYESC 27
#define Nf 5   // min number of features
#define Sd 50  // size of dictionary
#define StartFrame 0

//variables for input
//
QString InputImageAddress;
cv::Mat InputImageData, PreviousImageData;
QString img_dir;
QStringList images;
cv::Rect initial_bounding_box, tracking_bounding_box;
//TRD_Tracker mytracker(Nf,Sd);
int frame_index = StartFrame;

bool cameraFlag = 0;
bool TrackingEnableFlag = 0;

const int numSuperpixel = 10;
TRD_Tracker* mytrackers[numSuperpixel];
std::vector<int> myTrackingCounter;
std::vector<cv::Mat> patch_templates;
std::vector<cv::Point2i> PointList, OriginalPoints;
std::vector<cv::Point2i> PreviousPoints;
std::vector<float> DistVector;
float dist_threshold, fix_para;
cv::Point2i tracking_center, gaussian_center;
float tracking_scale;

//bool clicked=false;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_Load_dataset_botton_clicked()
{
    frame_index = StartFrame;
    for(int it = 0; it < numSuperpixel; it ++)
    {
        delete mytrackers[it];
        mytrackers[it] = NULL;
        myTrackingCounter.push_back(0);
    }

    patch_templates.clear();
    PointList.clear();
    DistVector.clear();

    QString dataset_dir = QFileDialog::getExistingDirectory(this, tr("Open Directory"),
                                                    "/home",
                                                    QFileDialog::ShowDirsOnly
                                                    | QFileDialog::DontResolveSymlinks);
    img_dir = dataset_dir + "/img/";
    QString init_dir = dataset_dir + "/groundtruth_rect.txt";
    QFile file(init_dir);
     if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
         return;
     int nn = 0;
     QString txt_line;
     QTextStream in(&file);
        while (!in.atEnd() && nn < frame_index+1)
        {
           txt_line = in.readLine();
           nn ++;
        }
        file.close();
//     QString txt_line = file.readLine();
     txt_line.replace(","," ");
     QTextStream stream(&txt_line);
     int number_table[5] = {0,0,0,0,0};
     int ii = 0;
     int number;
     while (!stream.atEnd())
     {
             stream >> number;
             number_table[ii] = number;
             ii ++;
     }
     initial_bounding_box = cvRect(number_table[0],number_table[1],number_table[2],number_table[3]);
     tracking_bounding_box = initial_bounding_box;
     tracking_center.x = tracking_bounding_box.x + tracking_bounding_box.width/2;
     tracking_center.y = tracking_bounding_box.y + tracking_bounding_box.height/2;
     tracking_scale = 1;
//     checkBoundary(InputImageData, initial_bounding_box);

     QDir directory(img_dir);
     images = directory.entryList(QStringList() << "*.png" << "*.jpg",QDir::Files);
     InputImageAddress = img_dir + images[frame_index];

     InputImageData = cv::imread(InputImageAddress.toStdString());
     myFixRect(initial_bounding_box,InputImageData);

     if(InputImageData.empty())
     {
         std::cout << "load error..." << std::endl;
     }
     else
     {
         SLIC slic;
         cv::Mat img,result;
         img = InputImageData(initial_bounding_box);
         result = InputImageData.clone();
         slic.GenerateSuperpixels(img, numSuperpixel);

         int* label = slic.GetLabel();
         PointList = findSuperpixelCenters(initial_bounding_box, label);
         OriginalPoints = PointList;
         int sub_rect_size = std::max(std::min(initial_bounding_box.height,initial_bounding_box.width)/2,32);
         cv::Rect sub_sample_box = cvRect(0,0,sub_rect_size,sub_rect_size);

         for(int it = 0; it < PointList.size(); it ++)
         {
             sub_sample_box.x = PointList[it].x-sub_rect_size/2;
             sub_sample_box.y = PointList[it].y-sub_rect_size/2;
             mytrackers[it] = new TRD_Tracker(Nf,Sd);
             mytrackers[it]->TrackerInitialization(InputImageData, sub_sample_box);
             cv::Mat patch_rgb, patch_hsv;
             patch_rgb = InputImageData(sub_sample_box);
             cv::cvtColor(patch_rgb, patch_hsv, CV_RGB2HSV);
             patch_templates.push_back(patch_hsv);
             myTrackingCounter[it] = 10;
             cv::rectangle(result, sub_sample_box, cv::Scalar(0,255,0), 1, 8, 0 );
         }

        DistVector = findDistancesVector(PointList);

//        qDebug() << "number = " << PointList.size();
//        std::cout << "points = " << PointList[0] << ", " << PointList[1] << ", " << PointList[2] << ", " << PointList[3] << ", " << PointList[4];
//        std::cout << ", " << PointList[5] << ", " << PointList[6] << ", " << PointList[7] << ", " << PointList[8] << std::endl;

//        qDebug() << "vector = " << DistVector[0] << ", " << DistVector[1] << ", " << DistVector[2] << ", " << DistVector[3] << ", " << DistVector[4];
//        qDebug() << DistVector[5] << ", " << DistVector[6] << ", " << DistVecordertor[7] << ", " << DistVector[8];

        int max_idx = findInxOfMax(DistVector);
        float max_dist = DistVector[max_idx];
        dist_threshold = getDistThreshold(DistVector);
        fix_para = max_dist/dist_threshold;
//        fix_para = fix_para*1.1;
//        fix_para = (fix_para - 1)*1.5 + 1;
//        qDebug() << max_dist << " / " << dist_threshold << " = " << fix_para;
//        mytracker.TrackerInitialization(InputImageData, initial_bounding_box);
//        cv::rectangle(InputImageData, initial_bounding_box, cv::Scalar(0,255,0), 1, 8, 0 );
        cv::imshow("result",result);
//        qDebug() << " sequence size = " << images.size();
     }
}

void MainWindow::on_Tracking_start_botton_clicked()
{
    TrackingEnableFlag = 1;
    while(frame_index + 1 < images.size() && TrackingEnableFlag)
    {
        if(cv::waitKey(10) == KEYESC)
            break;
        frame_index = frame_index + 1;
        qDebug() << "----------------image number " << frame_index + 1 << "-----------------------";
        InputImageData.release();
        InputImageAddress = img_dir + images[frame_index];
        InputImageData = cv::imread(InputImageAddress.toStdString());
//        mytracker.RunTracker(InputImageData);
//        cv::rectangle(InputImageData, mytracker.results[frame_index-StartFrame], cv::Scalar(0,255,0), 1, 8, 0 );
        cv::Mat result;
        result = InputImageData.clone();

        for(int it = 0; it < numSuperpixel; it ++)
        {
            if(mytrackers[it] != NULL)
            {
                mytrackers[it]->RunTracker(InputImageData);
//                PointList[it].x += mytrackers[it]->ShiftVector[0];
//                PointList[it].y += mytrackers[it]->ShiftVector[1];
                cv::Rect sub_rect_it = mytrackers[it]->results.back();
                PointList[it].x = sub_rect_it.x + sub_rect_it.width/2;
                PointList[it].y = sub_rect_it.y + sub_rect_it.height/2;

                if(mytrackers[it]->tracking_confidence < 0.5)
                {
                    cv::rectangle(result, mytrackers[it]->results.back(), cv::Scalar(0,0,255), 1, 8, 0 );
                    myTrackingCounter[it] += -1;
                }
                else if(mytrackers[it]->tracking_confidence < 0.8)
                {
                    cv::rectangle(result, mytrackers[it]->results.back(), cv::Scalar(0,255,255), 1, 8, 0 );
                    myTrackingCounter[it] += 2;
                }
                else
                {
                    cv::rectangle(result, mytrackers[it]->results.back(), cv::Scalar(0,255,0), 1, 8, 0 );
                    myTrackingCounter[it] += 5;
                }
            }
        }

        DistVector.clear();
        DistVector = findDistancesVector(PointList);

        for(int it = 0; it < PointList.size(); it ++)
        {
            float sample_distance = DistVector[it];
            if(sample_distance > fix_para*dist_threshold || myTrackingCounter[it] < 1)
            {
                myTrackingCounter[it] = 0;
                std::vector<int> main_point_index = findIndexOfMax(myTrackingCounter,3);
                cv::Point2i resample_point = findResamplePoint(OriginalPoints, PointList, main_point_index, it);
                cv::Rect estimateRegion = mytrackers[it]->results.back();
                estimateRegion.x = resample_point.x - estimateRegion.width*3/2;
                estimateRegion.y = resample_point.y - estimateRegion.height*3/2;
                estimateRegion.width += 2*estimateRegion.width;
                estimateRegion.height += 2*estimateRegion.height;
                myFixRect(estimateRegion,InputImageData);
                cv::Mat search_patch = InputImageData(estimateRegion);
                cv::cvtColor(search_patch, search_patch, CV_RGB2HSV);
                cv::Mat matching_results = TplMatch( search_patch, patch_templates[it] );
                cv::Point2i estimation_point = minmax(matching_results);
                cv::Rect sub_sample_it = mytrackers[it]->results.back();
                sub_sample_it.x = estimation_point.x + estimateRegion.x;
                sub_sample_it.y = estimation_point.y + estimateRegion.y;
                mytrackers[it]->Tracking_Recovery(sub_sample_it);
                if(mytrackers[it]->Matching_Test(sub_sample_it) > 0.6)
                {
                    myTrackingCounter[it] = 5;
                    std::cout << "found..." << std::endl;
                }
                else
                {
                    myTrackingCounter[it] = 2;
                    std::cout << "not sure..." << std::endl;
//                    std::cout << "matching whole region = " << mytrackers[it]->Matching_Test(estimateRegion) << std::endl;
                }

                std::cout << std::endl;

                PointList[it].x = sub_sample_it.x + sub_sample_it.width/2;
                PointList[it].y = sub_sample_it.y + sub_sample_it.height/2;
            }
//            std::cout << "tracker ID = " << it << "... distance = " << sample_distance << std::endl;
        }

        DistVector.clear();
        DistVector = findDistancesVector(PointList);

        dist_threshold = getDistThreshold(DistVector);

//        qDebug() << " distance threshold = " << fix_para*dist_threshold;

        cv::imshow("result",result);
        cv::waitKey(1);
    }
    qDebug() << "--------------------------stop----------------------------------";
}

void MainWindow::on_Tracking_Pause_botton_clicked()
{
    TrackingEnableFlag = 0;
}

void MainWindow::on_Tracking_stop_botton_clicked()
{
    frame_index = StartFrame;
    TrackingEnableFlag = 0;
}

void MainWindow::on_Debug_SingleStep_clicked()
{
        if(frame_index + 1 < images.size())
        {
                frame_index = frame_index + 1;
                qDebug() << "----------------image number " << frame_index + 1 << "-----------------------";
                InputImageData.release();
                InputImageAddress = img_dir + images[frame_index];
                InputImageData.release();
                InputImageData = cv::imread(InputImageAddress.toStdString());
//                mytracker.FeatureMatchingTest(InputImageData);
//                cv::rectangle(InputImageData, mytracker.results[frame_index-StartFrame], cv::Scalar(0,255,0), 1, 8, 0 );
                cv::Mat result;
                result = InputImageData.clone();
                for(int it = 0; it < numSuperpixel; it ++)
                {
                    if(mytrackers[it] != NULL)
                    {
                        mytrackers[it]->RunTracker(InputImageData);
                        if(mytrackers[it]->tracking_confidence < 0.5)
                        {
                            cv::rectangle(result, mytrackers[it]->results.back(), cv::Scalar(0,0,255), 1, 8, 0 );
                            myTrackingCounter[it] += -1;
                        }
                        else if(mytrackers[it]->tracking_confidence < 0.8)
                        {
                            cv::rectangle(result, mytrackers[it]->results.back(), cv::Scalar(0,255,255), 1, 8, 0 );
                            myTrackingCounter[it] += 2;
                        }
                        else
                        {
                            cv::rectangle(result, mytrackers[it]->results.back(), cv::Scalar(0,255,0), 1, 8, 0 );
                            myTrackingCounter[it] += 5;
                        }
                        if(myTrackingCounter[it] < 0)
                        {
                            std::vector<int> main_point_index = findIndexOfMax(myTrackingCounter,3);
                            cv::Rect currentBestLocation0 = mytrackers[main_point_index[0]]->results.back();
                            cv::Rect currentBestLocation1 = mytrackers[main_point_index[1]]->results.back();
                            cv::Rect currentBestLocation2 = mytrackers[main_point_index[2]]->results.back();
                            cv::Rect estimateRegion = currentBestLocation0;
                            estimateRegion.x = (currentBestLocation0.x + currentBestLocation1.x + currentBestLocation2.x)/3;
                            estimateRegion.y = (currentBestLocation0.y + currentBestLocation1.y + currentBestLocation2.y)/3;
        //                    mytrackers[it]->~TRD_Tracker();
                            delete mytrackers[it];
                            mytrackers[it] = NULL;
                            mytrackers[it] = new TRD_Tracker(Nf,Sd);
                            mytrackers[it]->TrackerInitialization(InputImageData, estimateRegion);
                            myTrackingCounter[it] = 10;
                            std::cout << "regenerate tracker " << it << std::endl;
                        }
                    }
                }
                cv::imshow("result",result);
//                SLIC slic;
//                cv::Mat img,result;
//                img = InputImageData(mytracker.results[frame_index-StartFrame]);
//                slic.GenerateSuperpixels(img, numSuperpixel);
//                if (img.channels() == 3)
//                    result = slic.GetImgWithContours(cv::Scalar(0, 0, 255));
//                else
//                    result = slic.GetImgWithContours(cv::Scalar(128));
//                cv::imshow("superpixel seg",result);
        }
        else
        {
            qDebug() << "index number = " << frame_index << " sequence has been finished...";
        }
}

void MainWindow::on_Change_result_botton_clicked()
{
//    QString s;
//    QByteArray pchar;
//    QString fileName = QFileDialog::getSaveFileName(this);
//    if (fileName.isEmpty())
//    {
//        InputImageData = cv::imread("Troll_Problem.jpg");
//        cv::imshow("result",InputImageData);
//        qDebug() << "It is not a good idea...";
//    }
//    qDebug() << fileName;
//    QFile myfile(fileName);
//    myfile.open(QIODevice::WriteOnly|QIODevice::Append);
//    for(int ii = 0; ii < images.size(); ii ++)
//    {
//        s = QString::number(mytracker.results[ii].x) + " " + QString::number(mytracker.results[ii].y) + " " + QString::number(mytracker.results[ii].width) + " " + QString::number(mytracker.results[ii].height) + "\n";
//        pchar = s.toLatin1();
//        myfile.write(pchar);
//    }
//    myfile.close();
//    qDebug() << "results saved...";
    qDebug() << "not available now...";
}
