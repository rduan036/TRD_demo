#ifndef TRD_TRACKER_H
#define TRD_TRACKER_H

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
//#include <opencv2/nonfree/nonfree.hpp>

class TRD_Tracker
{
private:
    struct MyComparator_float
    {
        const std::vector<float> & value_vector;

        MyComparator_float(const std::vector<float> & val_vec):
            value_vector(val_vec) {}

        bool operator()(int i1, int i2)
        {
            return value_vector[i1] > value_vector[i2];
        }
    };
    struct Dictionary
    {
        cv::Rect tracking_rect;
        std::vector<cv::KeyPoint> keypoints;
        std::vector<float> voting;
        cv::Mat descriptors;
        cv::Mat voting_matrix; //feature link table
    };
    unsigned int nfeatures;     //minimum number of detected features
    int Dictionary_size; //maximum size of dictionary
    float scale[2];
    float confidence_weight;    //for voting process
    float matching_confidence;  //for confidence dictionary updating
    int recover_flag;
    cv::Mat previous_frame;
    cv::Mat current_frame;
    cv::Rect Searching_rect;
    cv::Mat candidate_descriptors;      //descriptor of candidate
    std::vector<cv::KeyPoint> candidate;//keypoints of candidateoutlining
    cv::Mat image_patch;                //tracking region
    std::vector< cv::DMatch > good_matches;//matched correspondences
    std::vector< cv::DMatch > outliers;//matched correspondences
    Dictionary Static_Dictionary;
    Dictionary Confidence_Dictionary;
    Dictionary Dynamitic_Dictionary;
public:
    TRD_Tracker(int n, int s);
    ~TRD_Tracker();
    // results
    std::vector<cv::Rect> results;
    int Center[2];
    // basic operation
    bool ** create2DboolMatrix(int m, int n);
    void set2DboolMatrix(bool **matrix, int m, int n, bool value);
    int CvFloatMatToInt(cv::Mat, int m, int n);
    void FixRect(cv::Rect &r, cv::Mat I);
    void DeleteOneColOfMat(cv::Mat& object,int num);
    void DictionaryCover(Dictionary& D, Dictionary Dsource);
    void TrackerClear();
    void RectAdaption();
    // algorithm blocks
    void TrackerInitialization(cv::Mat img, cv::Rect init_rect);
    std::vector<cv::KeyPoint> FAST_Detection(cv::Mat image, int mi);
//    std::vector<cv::KeyPoint> SIFT_Detection(cv::Mat image, int mi);
    void FAST_Extraction(cv::Mat image, std::vector<cv::KeyPoint>& keypoint, cv::Mat& descriptors);
//    void SIFT_Extraction(cv::Mat image, std::vector<cv::KeyPoint>& keypoint, cv::Mat& descriptors);
    void DictionaryInitialization(cv::Mat image, Dictionary& D, cv::Rect init_rect);
    void FeatureDetecting();
    void FeatureMatching(Dictionary D);
    void UpdatingFrame(cv::Mat frame);
    void DynamiticDictionaryUpdating();
    void FeaturePositionShift();
    void FeatureScaleUpdate();
    void ShowFeatureMatching();
    void UpdateAppearanceModel();
    void VotingProcess(cv::Mat G);
    void ThreeLayersMatching();
    void SearchingRegionChange(int option);
    // function for testing
    void TrackerDebug();
    void FeatureMatchingTest(cv::Mat img);
    void RunTracker(cv::Mat img);
    float tracking_confidence;
    bool initialization_success;
    int ShiftVector[2];
    // new version
    float Matching_Test(cv::Rect Searching);
    void Tracking_Recovery(cv::Rect New_location);
//    cv::Mat HomographyTransform();
};

#endif // TRD_TRACKER_H


