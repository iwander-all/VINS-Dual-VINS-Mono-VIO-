#ifndef FEATURE_MANAGER_H
#define FEATURE_MANAGER_H

#include <list>
#include <algorithm>
#include <vector>
#include <numeric>
using namespace std;

#include <eigen3/Eigen/Dense>
using namespace Eigen;

#include <ros/console.h>
#include <ros/assert.h>

#include "parameters.h"

class FeaturePerFrame
{
  public:
    FeaturePerFrame(const Eigen::Matrix<double, 7, 1> &_point, double td)
    {
        point.x() = _point(0);
        point.y() = _point(1);
        point.z() = _point(2);
        uv.x() = _point(3);
        uv.y() = _point(4);
        velocity.x() = _point(5); 
        velocity.y() = _point(6); 
        cur_td = td;
    }

    double cur_td;
    Vector3d point;
    Vector2d uv;
    Vector2d velocity;
    double z;
    bool is_used;
    double parallax;
    MatrixXd A;
    VectorXd b;
    double dep_gradient;
};

class FeaturePerId
{
  public:
    // @param the global ID of the feature
    const int feature_id;

    // @param the first frame where the feature occurs
    int start_frame;
    int start_frame1;

    // @param all the parameters of one feature in the sliding windows
    vector<FeaturePerFrame> feature_per_frame;//first one is start frame
    vector<FeaturePerFrame> feature_per_frame1;

    // @param the size of vector<FeaturePerFrame>
    int used_num; 
    int used_num1;

    bool is_outlier;
    bool is_margin;

    // @param the depth of current feature on the normalized plane of the start frame
    double estimated_depth; // in start frame
    double estimated_depth1; // in start frame

    int solve_flag; // 0 haven't solve yet; 1 solve succ; 2 solve fail;

    Vector3d gt_p;

    FeaturePerId(int _feature_id, int _start_frame)
        : feature_id(_feature_id), start_frame(_start_frame),start_frame1(_start_frame),
          used_num(0), estimated_depth(-1.0),estimated_depth1(-1.0), solve_flag(0)
    {
    }

    // @param the last frame where the feature occur
    int endFrame();
    int endFrame1();
};

class FeatureManager
{
  public:
    FeatureManager(Matrix3d _Rs[]);

    // @brief get rotation from camera0/1 to IMU from estimator
    void setRic(Matrix3d _ric[]);

    void clearState();

    // @brief count the number of active features
    int getFeatureCount();

    // @brief add features to f_manager and decide marg_old or marg_new
    bool addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double td);
    
    void debugShow();

    vector<pair<Vector3d, Vector3d>> getCorresponding(int frame_count_l, int frame_count_r);

    // @brief set estimated_depth in left
    void setDepth(const VectorXd &x);

    // @brief set estimated_depth1 in right
    void setDepthInRight(const VectorXd &x);

    // @brief delete unactive features in f_manager
    void removeFailures();

    // @brief set estimated_depth = -1.0 in left
    void clearDepth(const VectorXd &x);

    // @brief set estimated_depth1 = -1.0 in right
    void clearDepthInRight(const VectorXd &x);

    // @brief get estimated_depth in left
    VectorXd getDepthVector();

    // @brief get estimated_depth in left
    VectorXd getDepthVectorInRight();

    // @brief calculate estimated_depth0/1 in left and right
    void triangulate(Vector3d Ps[], Vector3d tic[], Matrix3d ric[]);

    // @brief delete features in oldest frame and transform the depth (non-linear)
    void removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P,
                              Eigen::Matrix3d marg_R1, Eigen::Vector3d marg_P1, Eigen::Matrix3d new_R1, Eigen::Vector3d new_P1);

    // @brief delete features in oldest frame (initial)                                             
    void removeBack();
    
    // @brief delete features in second newest frame
    void removeFront(int frame_count);
    
    // @brief delete outlier features
    void removeOutlier();
    
    // @param all the features in the sliding windows
    list<FeaturePerId> feature;

    int last_track_num;

  private:
    double compensatedParallax2(const FeaturePerId &it_per_id, int frame_count);
    const Matrix3d *Rs;
    Matrix3d ric[NUM_OF_CAM];
};

#endif