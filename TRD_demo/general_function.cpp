#include "general_function.h"

void myFixRect(cv::Rect &r, cv::Mat I)
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
}

std::vector<cv::Point2i> findSuperpixelCenters(cv::Rect rect, int* label)
{
    std::vector<cv::Point2i> centers;
    std::vector< std::vector<cv::Point2i> > points_by_label;
//    centers.push_back(cv::Point2i(0,0));
    int max_num = -1;

    for( int j = 0; j < rect.height; j++ )
    {
        for( int k = 0; k < rect.width; k++ )
        {
           int index = j*rect.width + k;
           int current_label = label[index];
           if(current_label > max_num)
           {
               int addnum = current_label - max_num;
               max_num = current_label;
               for(int i = 0; i < addnum; i ++)
               {
                   std::vector<cv::Point2i> new_label;
                   points_by_label.push_back(new_label);
//                   std::cout << "new label added, start point " << new_label[0] << std::endl;
               }
               points_by_label[current_label].push_back(cv::Point2i(j,k));
           }
           else
           {
                points_by_label[current_label].push_back(cv::Point2i(j,k));
//                std::cout << "add point " << points_by_label[current_label].back() << " to label " << current_label << std::endl;
           }
        }
    }

    for( int ii = 0; ii <= max_num; ii ++)
    {
//        std::cout << std::endl << "--------------------------------label: " << ii << "----------------------------------------" << std::endl;
        int sum_x = 0;
        int sum_y = 0;
        for(int it = 0; it < points_by_label[ii].size(); it ++)
        {
//            std::cout << *it << " ";
            sum_x += points_by_label[ii][it].x;
            sum_y += points_by_label[ii][it].y;
        }
        centers.push_back(cv::Point2i(sum_y/points_by_label[ii].size() + rect.x, sum_x/points_by_label[ii].size() + rect.y));
    }

    return centers;
}

std::vector<int> findIndexOfMax(std::vector<int> array, int top)
{
    std::vector<int> topNindex;
    std::vector<int> copy_array = array;
    int max = copy_array[0];
    topNindex.push_back(0);
    for(int ii = 1; ii < copy_array.size(); ii ++)
    {
        if(max < copy_array[ii])
        {
            max = copy_array[ii];
            topNindex.pop_back();
            topNindex.push_back(ii);
        }
    }
    while(topNindex.size() < top)
    {
        copy_array[topNindex.back()] = 0;
        max = copy_array[0];
        topNindex.push_back(0);
        for(int ii = 1; ii < copy_array.size(); ii ++)
        {
            if(max < copy_array[ii])
            {
                max = copy_array[ii];
                topNindex.pop_back();
                topNindex.push_back(ii);
            }
        }
        copy_array[topNindex.back()] = 0;
    }
    return topNindex;
}

cv::Point2i findCenterOfRect(cv::Rect sub_rect)
{
    cv::Point2i rect_center;
    rect_center.x = sub_rect.x + sub_rect.width/2;
    rect_center.y = sub_rect.y + sub_rect.height/2;
}

cv::Mat TplMatch( cv::Mat &img, cv::Mat &mytemplate )
{
  cv::Mat result;

  cv::matchTemplate( img, mytemplate, result, CV_TM_SQDIFF_NORMED );
  cv::normalize( result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );

  return result;
}

cv::Point minmax( cv::Mat &result )
{
  double minVal, maxVal;
  cv::Point  minLoc, maxLoc, matchLoc;

  cv::minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );
  matchLoc = minLoc;

  return matchLoc;
}

std::vector<float> findDistancesVector(std::vector<cv::Point2i> PointList)
{
    std::vector<float> DistancesVector;
    float dist;
    for(int ii = 0; ii < PointList.size(); ii ++)
    {
        dist = findDistances(PointList,PointList[ii]);
//        std::cout << "dist push back " << dist << std::endl;
        DistancesVector.push_back(dist);
    }
    return DistancesVector;
}

float findDistances(std::vector<cv::Point2i> PointList, cv::Point2i TargetPoint)
{
    float Distances = 0;
    float distance;

    for(int ii = 0; ii < PointList.size(); ii ++)
    {
        distance = distance2points(PointList[ii], TargetPoint);
        Distances += distance;
    }

    return Distances;
}

float distance2points(cv::Point2i p1,cv::Point2i p2)
{
    cv::Point2i diff;
    diff = p1 - p2;
    return cv::sqrt((float)(diff.x*diff.x + diff.y*diff.y));
}

int findInxOfMax(std::vector<float> v)
{
    int idx;
    std::vector<float>::iterator result;
    result = std::max_element(v.begin(), v.end());
    idx = std::distance(v.begin(), result);
    return idx;
}

int findInxOfMin(std::vector<float> v)
{
    int idx;
    std::vector<float>::iterator result;
    result = std::min_element(v.begin(), v.end());
    idx = std::distance(v.begin(), result);
    return idx;
}

float getDistThreshold(std::vector<float> distVector)
{
    float threshold = 0;
    int centre_node_idx = findInxOfMin(distVector);
    int far_node_idx = findInxOfMax(distVector);
    for(int ii = 0; ii < distVector.size(); ii ++)
    {
        threshold += distVector[ii];
    }
    threshold = threshold - distVector[centre_node_idx] - distVector[far_node_idx];
//    std::cout << "threshold = " << threshold << "-->";
    threshold = threshold / (float)(distVector.size() - 2);
//    std::cout << threshold << std::endl;
    return threshold;
}

cv::Point2i findResamplePoint(std::vector<cv::Point2i> original_points, std::vector<cv::Point2i> points_list, std::vector<int> best3index, int resample_id)
{
    cv::Point2i resample;
    cv::Point2i shift0, shift1, shift2;
    shift0 = points_list[best3index[0]] - original_points[best3index[0]];
    shift1 = points_list[best3index[1]] - original_points[best3index[1]];
    shift2 = points_list[best3index[2]] - original_points[best3index[2]];
    resample.x = original_points[resample_id].x + (shift0.x + shift1.x + shift2.x)/3;
    resample.y = original_points[resample_id].y + (shift0.y + shift1.y + shift2.y)/3;
    return resample;
}

//void denyResult(cv::Mat &result, cv::Rect sub_rect)
//{
//    result(sub_rect) = cv::Mat::ones(sub_rect.height,sub_rect.width,CV_32FC1);
//}

//cv::Rect reSample(std::vector<cv::Point2i> rect_Centers, std::vector<int> trackingCounter, cv::Mat image)
//{
//    cv::Rect resample_rect;
//    std::vector<int> main_point_index = findIndexOfMax(trackingCounter,3);
//    return resample_rect;
//}
