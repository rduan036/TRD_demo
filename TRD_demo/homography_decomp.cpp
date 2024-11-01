#include "homography_decomp.h"

using namespace cv;

// normalizes homography with intrinsic camera parameters
cv::Matx33d HomographyDecomp::normalize(const Matx33d& H, const Matx33d& K)
{
    return K.inv() * H * K;
}

void HomographyDecomp::removeScale()
{
    Mat W;
    SVD::compute(_Hnorm, W);
    _Hnorm = _Hnorm * (1.0/W.at<double>(1));
}

/*! This checks that the input is a pure rotation matrix 'm'.
 * The conditions for this are: R' * R = I and det(R) = 1 (proper rotation matrix)
 */
bool HomographyDecomp::isRotationValid(const Matx33d& R, const double epsilon)
{
    Matx33d RtR = R.t() * R;
    Matx33d I(1,0,0, 0,1,0, 0,0,1);
    if (norm(RtR, I, NORM_INF) > epsilon)
        return false;
    return (fabs(determinant(R) - 1.0) < epsilon);
}

bool HomographyDecomp::passesSameSideOfPlaneConstraint(CameraMotion& motion)
{
    typedef Matx<double, 1, 1> Matx11d;
    Matx31d t = Matx31d(motion.t);
    Matx31d n = Matx31d(motion.n);
    Matx11d proj = n.t() * motion.R.t() * t;
    if ( (1 + proj(0, 0) ) <= 0 )
        return false;
    return true;
}

//!main routine to decompose homography
void HomographyDecomp::decomposeHomography(const Matx33d& H, const cv::Matx33d& K,
                                           std::vector<CameraMotion>& camMotions)
{
    //normalize homography matrix with intrinsic camera matrix
    _Hnorm = normalize(H, K);
    //remove scale of the normalized homography
    removeScale();
    //apply decomposition
    decompose(camMotions);
}

/* function computes R&t from tstar, and plane normal(n) using
 R = H * inv(I + tstar*transpose(n) );
 t = R * tstar;
 returns true if computed R&t is a valid solution
 */
bool HomographyDecompZhang::findMotionFrom_tstar_n(const cv::Vec3d& tstar, const cv::Vec3d& n, CameraMotion& motion)
{
    Matx31d tstar_m = Mat(tstar);
    Matx31d n_m = Mat(n);
    Matx33d temp = tstar_m * n_m.t();
    temp(0, 0) += 1.0;
    temp(1, 1) += 1.0;
    temp(2, 2) += 1.0;
    motion.R = getHnorm() * temp.inv();
    motion.t = motion.R * tstar;
    motion.n = n;
    return passesSameSideOfPlaneConstraint(motion);
}

void HomographyDecompZhang::decompose(std::vector<CameraMotion>& camMotions)
{
    Mat W, U, Vt;
    SVD::compute(getHnorm(), W, U, Vt);
    double lambda1=W.at<double>(0);
    double lambda3=W.at<double>(2);
    double lambda1m3 =  (lambda1-lambda3);
    double lambda1m3_2 = lambda1m3*lambda1m3;
    double lambda1t3 = lambda1*lambda3;

    double t1 = 1.0/(2.0*lambda1t3);
    double t2 = sqrt(1.0+4.0*lambda1t3/lambda1m3_2);
    double t12 = t1*t2;

    double e1 = -t1 + t12; //t1*(-1.0f + t2 );
    double e3 = -t1 - t12; //t1*(-1.0f - t2);
    double e1_2 = e1*e1;
    double e3_2 = e3*e3;

    double nv1p = sqrt(e1_2*lambda1m3_2 + 2*e1*(lambda1t3-1) + 1.0);
    double nv3p = sqrt(e3_2*lambda1m3_2 + 2*e3*(lambda1t3-1) + 1.0);
    double v1p[3], v3p[3];

    v1p[0]=Vt.at<double>(0)*nv1p, v1p[1]=Vt.at<double>(1)*nv1p, v1p[2]=Vt.at<double>(2)*nv1p;
    v3p[0]=Vt.at<double>(6)*nv3p, v3p[1]=Vt.at<double>(7)*nv3p, v3p[2]=Vt.at<double>(8)*nv3p;

    /*The eight solutions are
     (A): tstar = +- (v1p - v3p)/(e1 -e3), n = +- (e1*v3p - e3*v1p)/(e1-e3)
     (B): tstar = +- (v1p + v3p)/(e1 -e3), n = +- (e1*v3p + e3*v1p)/(e1-e3)
     */
    double v1pmv3p[3], v1ppv3p[3];
    double e1v3me3v1[3], e1v3pe3v1[3];
    double inv_e1me3 = 1.0/(e1-e3);

    for(int kk=0;kk<3;++kk){
        v1pmv3p[kk] = v1p[kk]-v3p[kk];
        v1ppv3p[kk] = v1p[kk]+v3p[kk];
    }

    for(int kk=0; kk<3; ++kk){
        double e1v3 = e1*v3p[kk];
        double e3v1=e3*v1p[kk];
        e1v3me3v1[kk] = e1v3-e3v1;
        e1v3pe3v1[kk] = e1v3+e3v1;
    }

    Vec3d tstar_p, tstar_n;
    Vec3d n_p, n_n;

    ///Solution group A
    for(int kk=0; kk<3; ++kk) {
        tstar_p[kk] = v1pmv3p[kk]*inv_e1me3;
        tstar_n[kk] = -tstar_p[kk];
        n_p[kk] = e1v3me3v1[kk]*inv_e1me3;
        n_n[kk] = -n_p[kk];
    }

    CameraMotion cmotion;
    //(A) Four different combinations for solution A
    // (i)  (+, +)
    if (findMotionFrom_tstar_n(tstar_p, n_p, cmotion))
        camMotions.push_back(cmotion);

    // (ii)  (+, -)
    if (findMotionFrom_tstar_n(tstar_p, n_n, cmotion))
        camMotions.push_back(cmotion);

    // (iii)  (-, +)
    if (findMotionFrom_tstar_n(tstar_n, n_p, cmotion))
        camMotions.push_back(cmotion);

    // (iv)  (-, -)
    if (findMotionFrom_tstar_n(tstar_n, n_n, cmotion))
        camMotions.push_back(cmotion);
    //////////////////////////////////////////////////////////////////
    ///Solution group B
    for(int kk=0;kk<3;++kk){
        tstar_p[kk] = v1ppv3p[kk]*inv_e1me3;
        tstar_n[kk] = -tstar_p[kk];
        n_p[kk] = e1v3pe3v1[kk]*inv_e1me3;
        n_n[kk] = -n_p[kk];
    }

    //(B) Four different combinations for solution B
    // (i)  (+, +)
    if (findMotionFrom_tstar_n(tstar_p, n_p, cmotion))
        camMotions.push_back(cmotion);

    // (ii)  (+, -)
    if (findMotionFrom_tstar_n(tstar_p, n_n, cmotion))
        camMotions.push_back(cmotion);

    // (iii)  (-, +)
    if (findMotionFrom_tstar_n(tstar_n, n_p, cmotion))
        camMotions.push_back(cmotion);

    // (iv)  (-, -)
    if (findMotionFrom_tstar_n(tstar_n, n_n, cmotion))
        camMotions.push_back(cmotion);
}

double HomographyDecompInria::oppositeOfMinor(const Matx33d& M, const int row, const int col)
{
    int x1 = col == 0 ? 1 : 0;
    int x2 = col == 2 ? 1 : 2;
    int y1 = row == 0 ? 1 : 0;
    int y2 = row == 2 ? 1 : 2;

    return (M(y1, x2) * M(y2, x1) - M(y1, x1) * M(y2, x2));
}

//computes R = H( I - (2/v)*te_star*ne_t )
void HomographyDecompInria::findRmatFrom_tstar_n(const cv::Vec3d& tstar, const cv::Vec3d& n, const double v, cv::Matx33d& R)
{
    Matx31d tstar_m = Matx31d(tstar);
    Matx31d n_m = Matx31d(n);
    Matx33d I(1.0, 0.0, 0.0,
              0.0, 1.0, 0.0,
              0.0, 0.0, 1.0);

    R = getHnorm() * (I - (2/v) * tstar_m * n_m.t() );
}

void HomographyDecompInria::decompose(std::vector<CameraMotion>& camMotions)
{
    const double epsilon = 0.001;
    Matx33d S;

    //S = H'H - I
    S = getHnorm().t() * getHnorm();
    S(0, 0) -= 1.0;
    S(1, 1) -= 1.0;
    S(2, 2) -= 1.0;

    //check if H is rotation matrix
    if( norm(S, NORM_INF) < epsilon) {
        CameraMotion motion;
        motion.R = Matx33d(getHnorm());
        motion.t = Vec3d(0, 0, 0);
        motion.n = Vec3d(0, 0, 0);
        camMotions.push_back(motion);
        return;
    }

    //! Compute nvectors
    Vec3d npa, npb;

    double M00 = oppositeOfMinor(S, 0, 0);
    double M11 = oppositeOfMinor(S, 1, 1);
    double M22 = oppositeOfMinor(S, 2, 2);

    double rtM00 = sqrt(M00);
    double rtM11 = sqrt(M11);
    double rtM22 = sqrt(M22);

    double M01 = oppositeOfMinor(S, 0, 1);
    double M12 = oppositeOfMinor(S, 1, 2);
    double M02 = oppositeOfMinor(S, 0, 2);

    int e12 = signd(M12);
    int e02 = signd(M02);
    int e01 = signd(M01);

    double nS00 = std::abs(S(0, 0));
    double nS11 = std::abs(S(1, 1));
    double nS22 = std::abs(S(2, 2));

    //find max( |Sii| ), i=0, 1, 2
    int indx = 0;
    if(nS00 < nS11){
        indx = 1;
        if( nS11 < nS22 )
            indx = 2;
    }
    else {
        if(nS00 < nS22 )
            indx = 2;
    }

    switch (indx) {
        case 0:
            npa[0] = S(0, 0),               npb[0] = S(0, 0);
            npa[1] = S(0, 1) + rtM22,       npb[1] = S(0, 1) - rtM22;
            npa[2] = S(0, 2) + e12 * rtM11, npb[2] = S(0, 2) - e12 * rtM11;
            break;
        case 1:
            npa[0] = S(0, 1) + rtM22,       npb[0] = S(0, 1) - rtM22;
            npa[1] = S(1, 1),               npb[1] = S(1, 1);
            npa[2] = S(1, 2) - e02 * rtM00, npb[2] = S(1, 2) + e02 * rtM00;
            break;
        case 2:
            npa[0] = S(0, 2) + e01 * rtM11, npb[0] = S(0, 2) - e01 * rtM11;
            npa[1] = S(1, 2) + rtM00,       npb[1] = S(1, 2) - rtM00;
            npa[2] = S(2, 2),               npb[2] = S(2, 2);
            break;
        default:
            break;
    }

    double traceS = S(0, 0) + S(1, 1) + S(2, 2);
    double v = 2.0 * sqrt(1 + traceS - M00 - M11 - M22);

    double ESii = signd(S(indx, indx)) ;
    double r_2 = 2 + traceS + v;
    double nt_2 = 2 + traceS - v;

    double r = sqrt(r_2);
    double n_t = sqrt(nt_2);

    Vec3d na = npa / norm(npa);
    Vec3d nb = npb / norm(npb);

    double half_nt = 0.5 * n_t;
    double esii_t_r = ESii * r;

    Vec3d ta_star = half_nt * (esii_t_r * nb - n_t * na);
    Vec3d tb_star = half_nt * (esii_t_r * na - n_t * nb);

    camMotions.resize(4);

    Matx33d Ra, Rb;
    Vec3d ta, tb;

    //Ra, ta, na
    findRmatFrom_tstar_n(ta_star, na, v, Ra);
    ta = Ra * ta_star;

    camMotions[0].R = Ra;
    camMotions[0].t = ta;
    camMotions[0].n = na;

    //Ra, -ta, -na
    camMotions[1].R = Ra;
    camMotions[1].t = -ta;
    camMotions[1].n = -na;

    //Rb, tb, nb
    findRmatFrom_tstar_n(tb_star, nb, v, Rb);
    tb = Rb * tb_star;

    camMotions[2].R = Rb;
    camMotions[2].t = tb;
    camMotions[2].n = nb;

    //Rb, -tb, -nb
    camMotions[3].R = Rb;
    camMotions[3].t = -tb;
    camMotions[3].n = -nb;
}

// function decomposes image-to-image homography to rotation and translation matrices
int decomposeHomographyMat(InputArray _H,
                       InputArray _K,
                       OutputArrayOfArrays _rotations,
                       OutputArrayOfArrays _translations,
                       OutputArrayOfArrays _normals)
{
    using namespace std;

    Mat H = _H.getMat().reshape(1, 3);
    CV_Assert(H.cols == 3 && H.rows == 3);

    Mat K = _K.getMat().reshape(1, 3);
    CV_Assert(K.cols == 3 && K.rows == 3);

    cv::Ptr<HomographyDecomp> hdecomp(new HomographyDecompInria);

    vector<CameraMotion> motions;
    hdecomp->decomposeHomography(H, K, motions);

    int nsols = static_cast<int>(motions.size());
    int depth = CV_64F; //double precision matrices used in CameraMotion struct

    if (_rotations.needed()) {
        _rotations.create(nsols, 1, depth);
        for (int k = 0; k < nsols; ++k ) {
            _rotations.getMatRef(k) = Mat(motions[k].R);
        }
    }

    if (_translations.needed()) {
        _translations.create(nsols, 1, depth);
        for (int k = 0; k < nsols; ++k ) {
            _translations.getMatRef(k) = Mat(motions[k].t);
        }
    }

    if (_normals.needed()) {
        _normals.create(nsols, 1, depth);
        for (int k = 0; k < nsols; ++k ) {
            _normals.getMatRef(k) = Mat(motions[k].n);
        }
    }

    return nsols;
}
