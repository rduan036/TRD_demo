#include "TRD_Tracker.h"
#include "homography_decomp.h"

using namespace cv;
using namespace std;

#define unsigned int uint;

//**********************************************************************************************************************
//general function
//**********************************************************************************************************************

template <typename T1>
float Average(std::vector<T1> v)
{
       int sum=0;
       for(uint i=0;i<v.size();i++)
               sum+=v[i];
       return sum/v.size();
}

template <typename T2>
float Deviation(std::vector<T2> v, float ave)
{
       float E=0.0;
       for(uint i=0;i<v.size();i++)
       {
           E+=((float)v[i] - ave)*((float)v[i] - ave);
       }

       E = (float)sqrt((double)(E / v.size()));
       return E;
}

//**********************************************************************************************************************
//member class function
//**********************************************************************************************************************

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief TRD_Tracker::TRD_Tracker
/// \param n
/// \param s
/// \param mi
/// constructor function / deconstructor

TRD_Tracker::TRD_Tracker(int n, int s)
{
    nfeatures = n;
    Dictionary_size = s;
    confidence_weight = 1;
    matching_confidence = 1;
    ShiftVector[0] = 0;
    ShiftVector[1] = 0;
    scale[0] = 1;
    scale[1] = 1;
    recover_flag = 0;
    results.clear();
    tracking_confidence = 1;
    initialization_success = 0;
//    out_view_flag = 0;

    std::cout << "tracker generated..." << std::endl;
}

TRD_Tracker::~TRD_Tracker()
{
    std::cout << "the tracker has been cleared..." << std::endl;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief TRD_Tracker::TrackerInitialization
/// \param img(first frame)
/// \param init_rect(initial bounding box)
/// initialize tracker

void TRD_Tracker::TrackerInitialization(Mat img, Rect init_rect)
{
    img.copyTo(previous_frame);
    img.copyTo(current_frame);
    DictionaryInitialization(img, Static_Dictionary, init_rect);
    DictionaryInitialization(img, Confidence_Dictionary, init_rect);
    DictionaryInitialization(img, Dynamitic_Dictionary, init_rect);
    results.push_back(init_rect);
    std::cout << "tracker initized..." << std::endl;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief TRD_Tracker::create2DboolMatrix
/// \param m: number of rows
/// \param n: number of cols
/// \return 2D bool matrix pointer

bool** TRD_Tracker::create2DboolMatrix(int m, int n)
{
    bool **matrix = new bool*[m];

    for (int i = 0; i < m; i++)
        matrix[i] = new bool[n];

    return matrix;
}

void TRD_Tracker::set2DboolMatrix(bool **matrix, int m, int n, bool value)
{
   for(int i = 0; i < m; i ++)
   {
       for(int j = 0; j < n; j ++)
       {
           matrix[i][j] = value;
       }
   }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief TRD_Tracker::CvFloatMatToInt
/// \param A<float>
/// \param m
/// \param n
/// \return int A(m,n)
/// convert float element in matrix A to int

int TRD_Tracker::CvFloatMatToInt(cv::Mat A, int m, int n)
{
    float e;
    int v;
    e = A.at<float>(m,n);
    if(e >= 0)
        e = e + 0.5;
    else
        e = e - 0.5;
    v = (int)e;
    return v;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief TRD_Tracker::FixRect
/// \param r (rect)
/// \param I (image)
/// handle the rect out of image problem

void TRD_Tracker::FixRect(cv::Rect &r, cv::Mat I)
{
    if(r.x < 0)
    {
        r.x = 0;
    }
    if(r.x + r.width > I.cols)
    {
        r.width = I.cols - r.x;
    }
    if(r.y < 0)
    {
        r.y = 0;
    }
    if(r.y + r.height > I.rows)
    {
        r.height = I.rows - r.y;
    }

    if(r.width < 16 || r.height < 16)
    {
        r = Confidence_Dictionary.tracking_rect;
    }
}

void TRD_Tracker::DeleteOneColOfMat(cv::Mat& object,int num)
{
    if (num<0 || num>=object.cols)
    {
        std::cout << " it is not a good idea... " << std::endl;
    }
    else
    {
        if (num == object.cols-1)
        {
            object = object.t();
            object.pop_back();
            object = object.t();
        }
        else
        {
            for (int i=num+1;i<object.cols;i++)
            {
                object.col(i-1) = object.col(i) + cv::Scalar(0,0,0,0);
            }
            object = object.t();
            object.pop_back();
            object = object.t();
        }
    }
}

void TRD_Tracker::TrackerClear()
{
    confidence_weight = 1;
    matching_confidence = 1;
    tracking_confidence = 1;
    ShiftVector[0] = 0;
    ShiftVector[1] = 0;
    scale[0] = 1;
    scale[1] = 1;
    recover_flag = 0;
    initialization_success = 0;
    previous_frame.release();
    current_frame.release();
    candidate_descriptors.release();      //descriptor of candidate
    candidate.clear();//keypoints of candidate
    image_patch.release();                //tracking region
    good_matches.clear();//matched correspondences
    outliers.clear();//matched correspondences
    results.clear();

    Static_Dictionary.descriptors.release();
    Static_Dictionary.keypoints.clear();
    Static_Dictionary.voting.clear();
    Static_Dictionary.voting_matrix.release();
    Static_Dictionary.tracking_rect = cv::Rect();

    Confidence_Dictionary.descriptors.release();
    Confidence_Dictionary.keypoints.clear();
    Confidence_Dictionary.voting.clear();
    Confidence_Dictionary.voting_matrix.release();
    Confidence_Dictionary.tracking_rect = cv::Rect();

    Dynamitic_Dictionary.descriptors.release();
    Dynamitic_Dictionary.keypoints.clear();
    Dynamitic_Dictionary.voting.clear();
    Dynamitic_Dictionary.voting_matrix.release();
    Dynamitic_Dictionary.tracking_rect = cv::Rect();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief TRD_Tracker::SIFT/FAST_Detection
/// \param image
/// \return keypoint

std::vector<cv::KeyPoint> TRD_Tracker::FAST_Detection(cv::Mat image, int mi)
{
    cv::Ptr<cv::FeatureDetector> detector;
    if(mi)
    {
        detector = cv::FeatureDetector::create("GridFAST");
    }
    else
    {
        detector = cv::FeatureDetector::create("PyramidFAST");
    }
    std::vector<cv::KeyPoint> keypoints_1;
//    keypoints_1.clear();
    detector->detect( image, keypoints_1 );
    if(keypoints_1.size() > Dictionary_size)
    {
        cv::KeyPointsFilter::retainBest(keypoints_1, Dictionary_size);
    }

    return keypoints_1;
}

//std::vector<cv::KeyPoint> TRD_Tracker::SIFT_Detection(cv::Mat image, int mi)
//{
//    cv::Ptr<cv::FeatureDetector> detector;
//    if(mi)
//    {
//        detector = cv::FeatureDetector::create("PyramidSIFT");
//    }
//    else
//    {
//        detector = cv::FeatureDetector::create("GridSIFT");
//    }
//    std::vector<cv::KeyPoint> keypoints_1;
//    keypoints_1.clear();
//    detector->detect( image, keypoints_1 );

//    return keypoints_1;
//}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief TRD_Tracker::FAST_Extraction
/// \param image
/// \param keypoint
/// \return descriptor matrix

void TRD_Tracker::FAST_Extraction(cv::Mat image, std::vector<cv::KeyPoint>& keypoint, cv::Mat& descriptors)
{
    cv::BriefDescriptorExtractor extractor;
    extractor.compute( image, keypoint, descriptors );
    descriptors.convertTo(descriptors, CV_32F);
}

//void TRD_Tracker::SIFT_Extraction(cv::Mat image, std::vector<cv::KeyPoint>& keypoint, cv::Mat& descriptors)
//{
//    cv::SiftDescriptorExtractor extractor;
//    extractor.compute( image, keypoint, descriptors );
//    descriptors.convertTo(descriptors, CV_32F);
//}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief TRD_Tracker::DictionaryInitialization
/// \param image (frist frame)
/// \param D (dictionary)
/// \param init_rect
/// initialize dictionary

void TRD_Tracker::DictionaryInitialization(cv::Mat image, Dictionary& D, cv::Rect init_rect)
{
    D.tracking_rect = init_rect;
    Searching_rect = D.tracking_rect;

    if(Searching_rect.width > 50 && Searching_rect.height > 50)
    {
        SearchingRegionChange(0);
        SearchingRegionChange(0);
    }
    FixRect(Searching_rect, image);
    D.keypoints.clear();
    D.voting.clear();
    D.descriptors.release();
    D.voting_matrix.release();

    D.keypoints = FAST_Detection(image(Searching_rect), 0);
//    D.keypoints = SIFT_Detection(image(Searching_rect), 0);
    int size_n = D.keypoints.size();

    for(int i = 0; i < size_n; i ++)
    {
        D.keypoints[i].pt.x = D.keypoints[i].pt.x + Searching_rect.x;
        D.keypoints[i].pt.y = D.keypoints[i].pt.y + Searching_rect.y;
        D.voting.push_back(1);
    }

    FAST_Extraction(image, D.keypoints, D.descriptors);
//    SIFT_Extraction(image, D.keypoints, D.descriptors);
    size_n = D.keypoints.size();
    D.voting_matrix = cv::Mat::eye(size_n,size_n,CV_32F);

    if(size_n > nfeatures)
    {
        initialization_success = 1;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief TRD_Tracker::UpdatingFrame
/// \param frame
/// update current frame

void TRD_Tracker::UpdatingFrame(cv::Mat frame)
{
    previous_frame.release();
    current_frame.copyTo(previous_frame);
    current_frame.release();
    frame.copyTo(current_frame);
}

void TRD_Tracker::SearchingRegionChange(int option)
{
    // searching region shrink
    if(option == 0)
    {
        Searching_rect.x = Searching_rect.x + Searching_rect.width/10;
        Searching_rect.y = Searching_rect.y + Searching_rect.height/10;
        Searching_rect.width = Searching_rect.width - Searching_rect.width/5;
        Searching_rect.height = Searching_rect.height - Searching_rect.height/5;
    }
    else
    {
        Searching_rect.x = Searching_rect.x - Searching_rect.width/10;
        Searching_rect.y = Searching_rect.y - Searching_rect.height/10;
        Searching_rect.width = Searching_rect.width + Searching_rect.width/5;
        Searching_rect.height = Searching_rect.height + Searching_rect.height/5;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief TRD_Tracker::FeatureDetecting
/// \param D (dictionary)
/// detect candidate features in current frame

void TRD_Tracker::FeatureDetecting()
{
    candidate.clear();

    image_patch.release();
    candidate_descriptors.release();
    image_patch = current_frame(Searching_rect).clone();
    candidate = FAST_Detection(image_patch, 0);
//    candidate = SIFT_Detection(image_patch, 0);
//    candidate = FAST_Detection(current_frame, 1);

    for(uint i = 0; i < candidate.size(); i ++)
    {
        candidate[i].pt.x = candidate[i].pt.x + Searching_rect.x;
        candidate[i].pt.y = candidate[i].pt.y + Searching_rect.y;
    }
    FAST_Extraction(current_frame, candidate, candidate_descriptors);
//    SIFT_Extraction(current_frame, candidate, candidate_descriptors);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief TRD_Tracker::FeatureMatching
/// \param D (dictionary)
/// after detected candidate features, matching the candidates with dictionary

void TRD_Tracker::FeatureMatching(Dictionary D)
{
    cv::FlannBasedMatcher matcher;
    std::vector< cv::DMatch > matches;

    matcher.match( D.descriptors, candidate_descriptors, matches );

    double min_dist = 100;

    //-- Quick calculation of max and min distances between keypoints
    for( int i = 0; i < D.descriptors.rows; i++ )
    {
        double dist = matches[i].distance;
        if( dist < min_dist )
            min_dist = dist;
    }

    good_matches.clear();
    outliers.clear();
    for( int i = 0; i < D.descriptors.rows; i++ )
    {
        if( matches[i].distance <= cv::max(2*min_dist, 0.02) )
        {
            good_matches.push_back( matches[i]);
        }
        else
        {
            outliers.push_back(matches[i]);
        }
    }

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief TRD_Tracker::ShowFeatureMatching
/// display feature matching results

void TRD_Tracker::ShowFeatureMatching()
{
    cv::Mat img_matches;
    cv::drawMatches( previous_frame, Static_Dictionary.keypoints, current_frame, candidate,
                 good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                 std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    cv::imshow( "result", img_matches );
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief TRD_Tracker::FeaturePositionShift//    cv::SiftDescriptorExtractor extractor;
//    cv::SurfDescriptorExtractor extractor;
/// ranking weighted shift and update inliers

void TRD_Tracker::FeaturePositionShift()
{
    Center[0] = Dynamitic_Dictionary.tracking_rect.x + Dynamitic_Dictionary.tracking_rect.width/2;
    Center[1] = Dynamitic_Dictionary.tracking_rect.y + Dynamitic_Dictionary.tracking_rect.height/2;
    //find the ranking weighted shift vector

    if(good_matches.size() > 0)
    {
        float Shift_Vector[good_matches.size()][2];
        float Shift_Weight[good_matches.size()];

        for(uint i = 0; i < good_matches.size(); i ++)
        {
            Shift_Vector[i][0] = float(candidate[good_matches[i].trainIdx].pt.x - Dynamitic_Dictionary.keypoints[good_matches[i].queryIdx].pt.x);
            Shift_Vector[i][1] = float(candidate[good_matches[i].trainIdx].pt.y - Dynamitic_Dictionary.keypoints[good_matches[i].queryIdx].pt.y);
            Shift_Weight[i] = Dynamitic_Dictionary.voting[good_matches[i].queryIdx];
        }
        cv::Mat s = cv::Mat(good_matches.size(), 2, CV_32F, Shift_Vector);
        cv::Mat r = cv::Mat(1, good_matches.size(), CV_32F, Shift_Weight);
        cv::Mat sr,v;
        cv::reduce(r,sr,1,CV_REDUCE_SUM);
        r = 1.0 / sr * r;
        v = r * s;
        ShiftVector[0] = CvFloatMatToInt(v,0,0);
        ShiftVector[1] = CvFloatMatToInt(v,0,1);

        if(ShiftVector[0]*ShiftVector[0] + ShiftVector[1]*ShiftVector[1] > Searching_rect.width*Searching_rect.height && recover_flag == 0)
        {
            ShiftVector[0] = 0;
            ShiftVector[1] = 0;
//            std::cout << "recover flag = " << recover_flag << " big shift detected..." << std::endl;
        }

        //dictionary feature shift

        for(int it = 0; it < Dynamitic_Dictionary.keypoints.size(); it ++ )
        {
            Dynamitic_Dictionary.keypoints[it].pt.x += ShiftVector[0];
            Dynamitic_Dictionary.keypoints[it].pt.y += ShiftVector[1];
        }
        Center[0] += ShiftVector[0];
        Center[1] += ShiftVector[1];
//        for(uint i = 0; i < Dynamitic_Dictionary.keypoints.size(); i ++)
//        {
//            Dynamitic_Dictionary.keypoints[i].pt.x = Dynamitic_Dictionary.keypoints[i].pt.x + ShiftVector[0];
//            Dynamitic_Dictionary.keypoints[i].pt.y = Dynamitic_Dictionary.keypoints[i].pt.y + ShiftVector[1];
//        }

        //remove outliers
        std::vector<float> Error_Vector;
        float error, mean_error, std_error;
        for(uint i = 0; i < good_matches.size(); i ++)
        {
            error = (Shift_Vector[i][0] - ShiftVector[0])*(Shift_Vector[i][0] - ShiftVector[0]) + (Shift_Vector[i][1] - ShiftVector[1])*(Shift_Vector[i][1] - ShiftVector[1]);
            Error_Vector.push_back(error);
        }
        mean_error = Average(Error_Vector);
        std_error = Deviation(Error_Vector,mean_error);

        std::vector<cv::DMatch> temp_good_matches;
        float voting_score_sum = 0.0;
        float voting_score_good = 0.0;
        for(uint i = 0; i < good_matches.size(); i ++)
        {
            if(Error_Vector[i] > 2.0*std_error)
            {
                outliers.push_back(good_matches[i]);
            }
            else
            {
                temp_good_matches.push_back(good_matches[i]);
                voting_score_good = voting_score_good + Dynamitic_Dictionary.voting[good_matches[i].queryIdx];
            }
            voting_score_sum = voting_score_sum + Dynamitic_Dictionary.voting[good_matches[i].queryIdx];
        }
        good_matches.clear();
        good_matches = temp_good_matches;
        if(recover_flag == 2)
            matching_confidence = 1;
        else if(good_matches.size() > 6)
            matching_confidence = voting_score_good/voting_score_sum;
        else if(good_matches.size() > 3)
            matching_confidence = 0.5;
        else
            matching_confidence = 0.0;

//        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//        // refine
//        if(good_matches.size() > 6)
//        {
//            std::vector<Point2f> obj;
//            std::vector<Point2f> scene;

    //        cv::Mat P(3,1,CV_64F);
    //        cv::Mat P_new(3,1,CV_64F);

//            for( uint i = 0; i < good_matches.size(); i++ )
//            {
//                //-- Get the keypoints from the good matches
//                obj.push_back( Dynamitic_Dictionary.keypoints[ good_matches[i].queryIdx ].pt );
//                scene.push_back( candidate[ good_matches[i].trainIdx ].pt );
//            }
//            cv::Mat H = cv::findHomography(obj, scene, CV_RANSAC);
//            float mK[3][3] = {743.69488, 0, 637.97675, 0, 748.08985, 515.50930, 0, 0, 1};
//            cv::Mat K = cv::Mat(3,3,CV_32FC1,mK);
//            cv::OutputArrayOfArrays R,T,N;
//            decomposeHomographyMat(H,K,R,T,N);
//            std::cout << "R = " << R << std::endl << "T = " << T << std::endl << " N = " << N << std::endl;
//        }
    //        perspectiveTransform( Dynamitic_Dictionary.keypoints.pt, Dynamitic_Dictionary.keypoints.pt, H);
    //        std::cout << "H = " << H << std::endl;
//            for(it = Dynamitic_Dictionary.keypoints.begin(); it != Dynamitic_Dictionary.keypoints.end(); it ++)
//            {
//                obj.clear();
//                scene.clear();
//                obj.push_back(it->pt);
//                perspectiveTransform(obj, scene, H);
//                it->pt.x = scene[0].x;
//                it->pt.y = scene[0].y;
//            }

//            obj.clear();
//            scene.clear();
//            obj.push_back(cv::Point2f(Center[0],Center[1]));
//            perspectiveTransform(obj, scene, H);
//            Center[0] = scene[0].x;
//            Center[1] = scene[1].y;

//    //        cv::Mat img_keypoints_H;
//    //        drawKeypoints( current_frame, Dynamitic_Dictionary.keypoints, img_keypoints_H, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
//    //        imshow("homography points transform", img_keypoints_H );
//    //        ShiftVector[0] += int(H.at<double>(0,2));
//    //        ShiftVector[1] += int(H.at<double>(1,2));
//        }
//        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    }
    else
    {
        ShiftVector[0] = 0;
        ShiftVector[1] = 0;
    }
        //points transform/shift
//            cv::Mat img_keypoints;
//            drawKeypoints( current_frame, Dynamitic_Dictionary.keypoints, img_keypoints, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
//            imshow("points transform/shift", img_keypoints);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///// \brief TRD_Tracker::HomographyTransform
///// find homography of features transformation

//cv::Mat TRD_Tracker::HomographyTransform()
//{
//    std::vector<Point2f> obj;
//    std::vector<Point2f> scene;

//    for( uint i = 0; i < good_matches.size(); i++ )
//    {
//        //-- Get the keypoints from the good matches
//        obj.push_back( Dynamitic_Dictionary.keypoints[ good_matches[i].queryIdx ].pt );
//        scene.push_back( candidate[ good_matches[i].trainIdx ].pt );
//    }
//    cv::Mat Homo = cv::findHomography(obj, scene, CV_RANSAC);
//    return Homo;
//}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief TRD_Tracker::FeatureScaleUpdate
/// update target scale by compute standard deviation ratio

void TRD_Tracker::FeatureScaleUpdate()
{
    if(good_matches.size() > 6)
    {
        std::vector<int> d_x;
        std::vector<int> d_y;
        std::vector<int> c_x;
        std::vector<int> c_y;
        for(uint i = 0; i < good_matches.size(); i ++)
        {
            d_x.push_back((int)Dynamitic_Dictionary.keypoints[good_matches[i].queryIdx].pt.x);
            d_y.push_back((int)Dynamitic_Dictionary.keypoints[good_matches[i].queryIdx].pt.y);
            c_x.push_back((int)candidate[good_matches[i].trainIdx].pt.x);
            c_y.push_back((int)candidate[good_matches[i].trainIdx].pt.y);
        }

        float dx_ave, dy_ave, cx_ave, cy_ave;
        float dx_div, dy_div, cx_div, cy_div;
        dx_ave = Average(d_x);
        dy_ave = Average(d_y);
        cx_ave = Average(c_x);
        cy_ave = Average(c_y);
        dx_div = Deviation(d_x,dx_ave);
        dy_div = Deviation(d_y,dy_ave);
        cx_div = Deviation(c_x,cx_ave);
        cy_div = Deviation(c_y,cy_ave);
        scale[0] = (cx_div / dx_div - 1) * 0.5 + 1;
        scale[1] = (cy_div / dy_div - 1) * 0.5 + 1;
    }
    else
    {
        scale[0] = 1;
        scale[1] = 1;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief TRD_Tracker::DynamiticDictionaryUpdating
/// update tracking rect

void TRD_Tracker::DynamiticDictionaryUpdating()
{
    UpdateAppearanceModel();
    Dynamitic_Dictionary.tracking_rect.width = (int)(scale[0] * Dynamitic_Dictionary.tracking_rect.width);
    Dynamitic_Dictionary.tracking_rect.height = (int)(scale[1] * Dynamitic_Dictionary.tracking_rect.height);
    Dynamitic_Dictionary.tracking_rect.x = Center[0] - Dynamitic_Dictionary.tracking_rect.width/2;
    Dynamitic_Dictionary.tracking_rect.y = Center[1] - Dynamitic_Dictionary.tracking_rect.height/2;
//    Dynamitic_Dictionary.tracking_rect.x = Dynamitic_Dictionary.tracking_rect.x + ShiftVector[0];
//    Dynamitic_Dictionary.tracking_rect.y = Dynamitic_Dictionary.tracking_rect.y + ShiftVector[1];
//    if(scale[0] != 1)
//    {
//        Dynamitic_Dictionary.tracking_rect.x -= (int)((scale[0] - 1) * Dynamitic_Dictionary.tracking_rect.width / 2);
//        Dynamitic_Dictionary.tracking_rect.width = (int)(scale[0] * Dynamitic_Dictionary.tracking_rect.width);
//    }
//    if(scale[1] != 1)
//    {
//        Dynamitic_Dictionary.tracking_rect.y -= (int)((scale[1] - 1) * Dynamitic_Dictionary.tracking_rect.height / 2);
//        Dynamitic_Dictionary.tracking_rect.height = (int)(scale[1] * Dynamitic_Dictionary.tracking_rect.height);
//    }

    tracking_confidence += matching_confidence*0.5;

    if(matching_confidence > 0.95)
    {
        DictionaryInitialization(current_frame, Confidence_Dictionary, Dynamitic_Dictionary.tracking_rect);
    }
    if(recover_flag == 2)
    {
        DictionaryInitialization(current_frame, Confidence_Dictionary, Dynamitic_Dictionary.tracking_rect);
    }
}

void TRD_Tracker::UpdateAppearanceModel()
{
    uint d_t = Dynamitic_Dictionary.voting_matrix.rows;

    uint d_c = candidate.size();
    uint new_size = d_t + d_c;

    cv::Mat new_link_table = cv::Mat::eye(new_size, new_size, CV_32F);
    Dynamitic_Dictionary.voting_matrix.copyTo(new_link_table(cv::Rect(0,0,d_t,d_t)));

    for(uint i = 0; i < d_c; i ++)
    {
        Dynamitic_Dictionary.keypoints.push_back(candidate[i]);
        Dynamitic_Dictionary.descriptors.push_back(candidate_descriptors.row(i));
    }

    // initial x, index
    std::vector<float> index;
    Dynamitic_Dictionary.voting.clear();
    for(uint i = 0; i < new_size; i ++)
    {
        Dynamitic_Dictionary.voting.push_back(1);
        index.push_back(i);
    }

    // add link
    for(uint i = 0; i < good_matches.size(); i ++)
    {
        new_link_table.at<float>(good_matches[i].trainIdx + d_t, good_matches[i].queryIdx) = 1;
        for(uint j = 0; j < outliers.size(); j ++)
        {
            new_link_table.at<float>(outliers[j].trainIdx + d_t ,good_matches[i].trainIdx + d_t) = 1;
        }
    }

    // voting process
    VotingProcess(new_link_table);

    // sorting and optimization
    std::sort(index.begin(), index.end(), MyComparator_float(Dynamitic_Dictionary.voting));


    if(index.size() > (uint)Dictionary_size)
    {
        std::vector<cv::KeyPoint> top_n_keypoint;
        std::vector<float> top_n_voting;
        cv::Mat top_n_descriptors;
        cv::Mat row_voting_matrix,t_row_voting_matrix, col_voting_matrix, optimized_voting_matrix;

        for(int i = 0; i < Dictionary_size; i ++)
        {
            top_n_keypoint.push_back(Dynamitic_Dictionary.keypoints[index[i]]);
            top_n_voting.push_back(Dynamitic_Dictionary.voting[index[i]]);
            top_n_descriptors.push_back(Dynamitic_Dictionary.descriptors.row(index[i]));
            row_voting_matrix.push_back(new_link_table.row(index[i]));
        }

        cv::transpose(row_voting_matrix, t_row_voting_matrix);
        for(int i = 0; i < Dictionary_size; i ++)
        {
            col_voting_matrix.push_back(t_row_voting_matrix.row(index[i]));
        }

        cv::transpose(col_voting_matrix, optimized_voting_matrix);

        Dynamitic_Dictionary.keypoints.clear();
        Dynamitic_Dictionary.voting.clear();
        Dynamitic_Dictionary.descriptors.release();
        Dynamitic_Dictionary.voting_matrix.release();

        Dynamitic_Dictionary.keypoints = top_n_keypoint;
        Dynamitic_Dictionary.voting = top_n_voting;
        top_n_descriptors.copyTo(Dynamitic_Dictionary.descriptors);
        optimized_voting_matrix.copyTo(Dynamitic_Dictionary.voting_matrix);
    }
    else
    {
        Dynamitic_Dictionary.voting_matrix.release();
        new_link_table.copyTo(Dynamitic_Dictionary.voting_matrix);
    }

}

void TRD_Tracker::VotingProcess(cv::Mat G)
{
    cv::Mat Cj, Ci, D, A, B, x;
    int n_size = G.rows;
    B = cv::Mat::zeros(n_size, n_size, CV_32F);
    D = cv::Mat::zeros(n_size, n_size, CV_32F);
    x = cv::Mat::ones(n_size,1, CV_32F);
    cv::reduce(G,Cj,0,CV_REDUCE_SUM, -1);
    cv::reduce(G,Ci,1,CV_REDUCE_SUM, -1);
    for(int i = 0; i < n_size; i ++)
    {
        D.at<float>(i,i) = 1.0 / Cj.at<float>(0,i);
        B.at<float>(i,i) = 1.0 / Ci.at<float>(i,0);
    }

    A = confidence_weight * G * D + (1 - confidence_weight) * B * G;

    for(int i = 0; i < n_size; i ++)
    {
        x = A * x;
        for(int j = 0; j < n_size; j ++)
        {
            if(x.at<float>(j,0) < 1)
            {
                x.at<float>(j,0) = 1.0;
            }
        }
    }

    Dynamitic_Dictionary.voting.clear();

    for(int i = 0; i < n_size; i ++)
    {
        Dynamitic_Dictionary.voting.push_back(x.at<float>(i,0));
    }

}

void TRD_Tracker::DictionaryCover(Dictionary& D, Dictionary Dsource)
{
    D.tracking_rect = Dsource.tracking_rect;
    D.descriptors.release();
    D.keypoints.clear();
    D.voting.clear();
    D.voting_matrix.release();
    D.keypoints.insert(D.keypoints.end(), Dsource.keypoints.begin(), Dsource.keypoints.end());
    D.voting.insert(D.voting.end(), Dsource.voting.begin(), Dsource.voting.end());
    Dsource.voting_matrix.copyTo(D.voting_matrix);
    Dsource.descriptors.copyTo(D.descriptors);
}

void TRD_Tracker::ThreeLayersMatching()
{
    Searching_rect = Dynamitic_Dictionary.tracking_rect;
    if(Searching_rect.width > 50 && Searching_rect.height > 50)
    {
        SearchingRegionChange(0);
        SearchingRegionChange(0);
    }
    FixRect(Searching_rect,current_frame);
    FeatureMatching(Static_Dictionary);
    if(good_matches.size() > 3)
    {
//        std::cout << "recover to static dictionary..." << std::endl;
        DictionaryCover(Dynamitic_Dictionary, Static_Dictionary);
        recover_flag = 2;
        tracking_confidence = 1;
    }
    else
    {
        FeatureMatching(Confidence_Dictionary);
        if(good_matches.size() > 6)
        {
//            std::cout << "recover to confidence dictionary..." << std::endl;
            DictionaryCover(Dynamitic_Dictionary, Confidence_Dictionary);
            recover_flag = 1;
            tracking_confidence = 0.5;
        }
        else
        {
            FeatureMatching(Dynamitic_Dictionary);
            recover_flag = 0;
            tracking_confidence = 0;
        }
    }
}

//************************************************************************************************************************************
// function for testing
//************************************************************************************************************************************

void TRD_Tracker::TrackerDebug()
{
    std::cout << "-------------Result info-----------------" << std::endl;
    std::cout << "ShiftVector = {" << ShiftVector[0] << ", " << ShiftVector[1] << "} " << std::endl;
    std::cout << "shift = " << ShiftVector[0]*ShiftVector[0] + ShiftVector[1]*ShiftVector[1] << std::endl;
    std::cout << "Scale = {" << scale[0] << ", " << scale[1] << "} " << std::endl;
    std::cout << "-------------Corrent frame info-------------" << std::endl;
    std::cout << "detected feature size = " << candidate.size() << std::endl;
    std::cout << "detected feature descriptor size = " << candidate_descriptors.cols << std::endl;
    std::cout << "good matches size = " << good_matches.size() << std::endl;
    std::cout << "outliers size = " << outliers.size() << std::endl;
    std::cout << "-------------Dictionary info----------------" << std::endl;
    std::cout << "recover flag = " << recover_flag << std::endl;
    std::cout << "voting vector size = " << Dynamitic_Dictionary.voting.size() << " voting matrix size = { " << Dynamitic_Dictionary.voting_matrix.rows << ", " << Dynamitic_Dictionary.voting_matrix.cols << " }" << std::endl;
    std::cout << "feature: keypoints size = " << Dynamitic_Dictionary.keypoints.size() << " <--> descriptor size = " << Dynamitic_Dictionary.descriptors.rows << std::endl;
    std::cout << "rect = " << Dynamitic_Dictionary.tracking_rect << std::endl;
    std::cout << "matching confidence = " << matching_confidence << std::endl;
}

void TRD_Tracker::FeatureMatchingTest(cv::Mat img)
{
    UpdatingFrame(img);
    std::cout << "New frame loaded..." << std::endl;
    FeatureDetecting();
    std::cout << "Features detected..." << std::endl;
    if(candidate.size() > 0)
    {
        double t1 = (double)cv::getTickCount();
        ThreeLayersMatching();
        double t2 = (double)cv::getTickCount();
        double time = (t2 - t1)/cv::getTickFrequency();
        std::cout << "Feature matching done...time cost = " << time << std::endl;
        t1 = (double)cv::getTickCount();
        FeaturePositionShift();
        t2 = (double)cv::getTickCount();
        time = (t2 - t1)/cv::getTickFrequency();
        std::cout << "Position shift has been found...time cost = " << time << std::endl;

//        FeatureScaleUpdate();

        t1 = (double)cv::getTickCount();
        DynamiticDictionaryUpdating();
        t2 = (double)cv::getTickCount();
        time = (t2 - t1)/cv::getTickFrequency();
        std::cout << "Dictionary updated...time cost = " << time << std::endl;
    }
    else
    {
        std::cout << "No feature detected..." << std::endl;
    }
    results.push_back(Dynamitic_Dictionary.tracking_rect);
    std::cout << "Tracker information report:" << std::endl;
    TrackerDebug();
}

void TRD_Tracker::RunTracker(cv::Mat img)
{
//    cv::Rect tracking_results;
    UpdatingFrame(img);
    FeatureDetecting();
    if(candidate.size() > 0)
    {
        ThreeLayersMatching();
        FeaturePositionShift();
//        FeatureScaleUpdate();
        DynamiticDictionaryUpdating();
    }
//    tracking_results = Dynamitic_Dictionary.tracking_rect;
//    out_view_flag = FixRect(tracking_results,current_frame);
//    results.push_back(tracking_results);
    results.push_back(Dynamitic_Dictionary.tracking_rect);
}

float TRD_Tracker::Matching_Test(cv::Rect Searching)
{
//    if(Searching.width > 50 && Searching.height > 50)
//    {
//        SearchingRegionChange(0);
//        SearchingRegionChange(0);
//    }
    FixRect(Searching,current_frame);
    FeatureMatching(Static_Dictionary);
    if(good_matches.size() > 3)
    {
//        std::cout << "recover to static dictionary..." << std::endl;
        DictionaryCover(Dynamitic_Dictionary, Static_Dictionary);
        recover_flag = 2;
        tracking_confidence = 1;
    }
    else
    {
        tracking_confidence = good_matches.size()*0.2;
        FeatureMatching(Confidence_Dictionary);
        if(good_matches.size() > 3)
        {
//            std::cout << "recover to confidence dictionary..." << std::endl;
            DictionaryCover(Dynamitic_Dictionary, Confidence_Dictionary);
            recover_flag = 1;
            tracking_confidence += good_matches.size()*0.1;
        }
        else
        {
            FeatureMatching(Dynamitic_Dictionary);
            recover_flag = 0;
            tracking_confidence += good_matches.size()*0.05;
        }
    }

    return tracking_confidence;
}

void TRD_Tracker::Tracking_Recovery(cv::Rect New_location)
{
    Dynamitic_Dictionary.tracking_rect = New_location;
}
