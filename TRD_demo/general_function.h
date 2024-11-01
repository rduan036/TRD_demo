#ifndef GENERAL_FUNCTION_H
#define GENERAL_FUNCTION_H

#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <numeric>

#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

void myFixRect(cv::Rect &, cv::Mat);
std::vector<cv::Point2i> findSuperpixelCenters(cv::Rect, int*);
float distance2points(cv::Point2i,cv::Point2i);
float findDistances(std::vector<cv::Point2i>, cv::Point2i);
std::vector<float> findDistancesVector(std::vector<cv::Point2i>);
std::vector<int> findIndexOfMax(std::vector<int>,int);
cv::Point2i findCenterOfRect(cv::Rect);
cv::Mat TplMatch(cv::Mat&, cv::Mat&);
cv::Point minmax(cv::Mat&);
int findInxOfMax(std::vector<float>);
int findInxOfMin(std::vector<float>);
float getDistThreshold(std::vector<float>);
cv::Point2i findResamplePoint(std::vector<cv::Point2i>,std::vector<cv::Point2i>,std::vector<int>,int);

//void denyResult(cv::Mat&, cv::Rect);
//cv::Rect reSample(std::vector<cv::Point2i>,std::vector<int>,cv::Mat);
//cv::Point2i generateRandomPoint(cv::Point2i,int);

#endif // GENERAL_FUNCTION_H
