#ifndef HOMOGRAPHY_DECOMP_H
#define HOMOGRAPHY_DECOMP_H

#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ocl/matrix_operations.hpp>
#include <opencv2/ocl/ocl.hpp>

#include <cstdio>
#include <cstdarg>
#include <sstream>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv_modules.hpp>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/mat.hpp>

#include <memory>

//struct to hold solutions of homography decomposition
typedef struct _CameraMotion {
    cv::Matx33d R; //!< rotation matrix
    cv::Vec3d n; //!< normal of the plane the camera is looking at
    cv::Vec3d t; //!< translation vector
} CameraMotion;

inline int signd(const double x)
{
    return ( x >= 0 ? 1 : -1 );
}

class HomographyDecomp {

public:
    HomographyDecomp() {}
    virtual ~HomographyDecomp() {}
    virtual void decomposeHomography(const cv::Matx33d& H, const cv::Matx33d& K,
                                     std::vector<CameraMotion>& camMotions);
    bool isRotationValid(const cv::Matx33d& R,  const double epsilon=0.01);

protected:
    bool passesSameSideOfPlaneConstraint(CameraMotion& motion);
    virtual void decompose(std::vector<CameraMotion>& camMotions) = 0;
    const cv::Matx33d& getHnorm() const {
        return _Hnorm;
    }

private:
    cv::Matx33d normalize(const cv::Matx33d& H, const cv::Matx33d& K);
    void removeScale();
    cv::Matx33d _Hnorm;
};

class HomographyDecompZhang : public HomographyDecomp {

public:
    HomographyDecompZhang():HomographyDecomp() {}
    virtual ~HomographyDecompZhang() {}

private:
    virtual void decompose(std::vector<CameraMotion>& camMotions);
    bool findMotionFrom_tstar_n(const cv::Vec3d& tstar, const cv::Vec3d& n, CameraMotion& motion);
};

class HomographyDecompInria : public HomographyDecomp {

public:
    HomographyDecompInria():HomographyDecomp() {}
    virtual ~HomographyDecompInria() {}

private:
    virtual void decompose(std::vector<CameraMotion>& camMotions);
    double oppositeOfMinor(const cv::Matx33d& M, const int row, const int col);
    void findRmatFrom_tstar_n(const cv::Vec3d& tstar, const cv::Vec3d& n, const double v, cv::Matx33d& R);
};

int decomposeHomographyMat(cv::InputArray, cv::InputArray, cv::OutputArrayOfArrays, cv::OutputArrayOfArrays, cv::OutputArrayOfArrays);

#endif // HOMOGRAPHY_DECOMP_H
