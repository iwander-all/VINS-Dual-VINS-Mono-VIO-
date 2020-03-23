#include "../include/feature_manager.h"

int FeaturePerId::endFrame()
{
    return start_frame + feature_per_frame.size() - 1;
}

int FeaturePerId::endFrame1()
{
    return start_frame1 + feature_per_frame1.size() - 1;
}

FeatureManager::FeatureManager(Matrix3d _Rs[])
    : Rs(_Rs)
{
    for (int i = 0; i < NUM_OF_CAM; i++)
        ric[i].setIdentity();
}

void FeatureManager::setRic(Matrix3d _ric[])
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ric[i] = _ric[i];
    }
}

void FeatureManager::clearState()
{
    feature.clear();
}

int FeatureManager::getFeatureCount()
{
    //ROS_INFO("FeatureManager::getFeatureCount");
    int cnt = 0;
    for (auto &it : feature)
    {

        it.used_num = it.feature_per_frame.size();
        // it.used_num1 = it.feature_per_frame1.size();
        // assert(it.used_num == it.used_num1);
        // assert(it.start_frame == it.start_frame1);

        if (it.used_num >= 2 && it.start_frame < WINDOW_SIZE - 2)
        {
            cnt++;
        }
    }
    return cnt;
}


bool FeatureManager::addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double td)
{
    // ROS_INFO("FeatureManager::addFeatureCheckParallax");
    // ROS_DEBUG("input feature: %d", (int)image.size());
    // ROS_DEBUG("num of feature: %d", getFeatureCount());
    double parallax_sum = 0;
    int parallax_num = 0;
    last_track_num = 0;
    for (auto &id_pts : image)
    {

        FeaturePerFrame f_per_fra(id_pts.second[0].second, td);
        FeaturePerFrame f_per_fra1(id_pts.second[1].second, td);

        int feature_id = id_pts.first;
        auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId &it)
                          {
            return it.feature_id == feature_id;
                          });

        if (it == feature.end())
        {
            feature.push_back(FeaturePerId(feature_id, frame_count));
            feature.back().feature_per_frame.push_back(f_per_fra);
            feature.back().feature_per_frame1.push_back(f_per_fra1);
        }
        else if (it->feature_id == feature_id)
        {
            it->feature_per_frame.push_back(f_per_fra);
            it->feature_per_frame1.push_back(f_per_fra1);
            last_track_num++;
        }
    }

    if (frame_count < 2 || last_track_num < 20)
        return true;

    for (auto &it_per_id : feature)
    {
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1)
        {
            parallax_sum += compensatedParallax2(it_per_id, frame_count);
            parallax_num++;
        }
        // if (it_per_id.feature_per_frame.size()>=1 || it_per_id.feature_per_frame1.size()>=1)
        // {
        //     ROS_INFO("the size of one feature in left:%d",it_per_id.feature_per_frame.size());
        //     ROS_INFO("the size of one feature in right:%d",it_per_id.feature_per_frame1.size());
        //     ROS_INFO("                                                     ");
        //     assert(it_per_id.feature_per_frame.size() == it_per_id.feature_per_frame1.size());
        // }
    }

    if (parallax_num == 0)
    {
        return true;
    }
    else
    {
        // ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        // ROS_DEBUG("current parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
        return parallax_sum / parallax_num >= MIN_PARALLAX;
    }
}

void FeatureManager::debugShow()
{
    ROS_DEBUG("debug show");
    for (auto &it : feature)
    {
        ROS_ASSERT(it.feature_per_frame.size() != 0);
        ROS_ASSERT(it.start_frame >= 0);
        ROS_ASSERT(it.used_num >= 0);

        ROS_DEBUG("%d,%d,%d ", it.feature_id, it.used_num, it.start_frame);
        int sum = 0;
        for (auto &j : it.feature_per_frame)
        {
            ROS_DEBUG("%d,", int(j.is_used));
            sum += j.is_used;
            printf("(%lf,%lf) ",j.point(0), j.point(1));
        }
        ROS_ASSERT(it.used_num == sum);
    }
}

vector<pair<Vector3d, Vector3d>> FeatureManager::getCorresponding(int frame_count_l, int frame_count_r)
{
    vector<pair<Vector3d, Vector3d>> corres;
    for (auto &it : feature)
    {
        if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r)
        {
            Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
            int idx_l = frame_count_l - it.start_frame;
            int idx_r = frame_count_r - it.start_frame;

            a = it.feature_per_frame[idx_l].point;

            b = it.feature_per_frame[idx_r].point;
            
            corres.push_back(make_pair(a, b));
        }
    }
    return corres;
}

void FeatureManager::setDepth(const VectorXd &x)
{
    // ROS_INFO("FeatureManager::setDepth");
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        // it_per_id.used_num1 = it_per_id.feature_per_frame1.size();
        // assert(it_per_id.used_num == it_per_id.used_num1);
        // assert(it_per_id.start_frame == it_per_id.start_frame1);

        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        it_per_id.estimated_depth = 1.0 / x(++feature_index);
        //ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
        if (it_per_id.estimated_depth < 0)
            it_per_id.solve_flag = 2;
        else
            it_per_id.solve_flag = 1;
    }
}

void FeatureManager::setDepthInRight(const VectorXd &x)
{
    // ROS_INFO("FeatureManager::setDepthInRight");
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num1 = it_per_id.feature_per_frame1.size();
        
        if (!(it_per_id.used_num1 >= 2 && it_per_id.start_frame1 < WINDOW_SIZE - 2))
            continue;

        it_per_id.estimated_depth1 = 1.0 / x(++feature_index);
        //ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
        if (it_per_id.estimated_depth1 < 0)
            it_per_id.solve_flag = 2;
        else
            it_per_id.solve_flag = 1;
    }
}

void FeatureManager::removeFailures()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        if (it->solve_flag == 2)
            feature.erase(it);
    }
}

void FeatureManager::clearDepth(const VectorXd &x)
{
    // ROS_INFO("FeatureManager::clearDepth");
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        // it_per_id.used_num1 = it_per_id.feature_per_frame1.size();
        // assert(it_per_id.used_num == it_per_id.used_num1);
        // assert(it_per_id.start_frame == it_per_id.start_frame1);

        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        it_per_id.estimated_depth = 1.0 / x(++feature_index);
    }
}

void FeatureManager::clearDepthInRight(const VectorXd &x)
{
    // ROS_INFO("FeatureManager::clearDepthInRight");
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num1 = it_per_id.feature_per_frame1.size();
        
        if (!(it_per_id.used_num1 >= 2 && it_per_id.start_frame1 < WINDOW_SIZE - 2))
            continue;
        it_per_id.estimated_depth1 = 1.0 / x(++feature_index);
    }
}

VectorXd FeatureManager::getDepthVector()
{
    // ROS_INFO("FeatureManager::getDepthVector");
    VectorXd dep_vec(getFeatureCount());
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        // it_per_id.used_num1 = it_per_id.feature_per_frame1.size();
        // assert(it_per_id.used_num == it_per_id.used_num1);

        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        dep_vec(++feature_index) = 1. / it_per_id.estimated_depth;
    }
    return dep_vec;
}

VectorXd FeatureManager::getDepthVectorInRight()
{
    // ROS_INFO("FeatureManager::getDepthVectorInRight");
    VectorXd dep_vec(getFeatureCount());
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num1 = it_per_id.feature_per_frame1.size();
        if (!(it_per_id.used_num1 >= 2 && it_per_id.start_frame1 < WINDOW_SIZE - 2))
            continue;
        dep_vec(++feature_index) = 1. / it_per_id.estimated_depth1;
    }
    return dep_vec;
}

void FeatureManager::triangulate(Vector3d Ps[], Vector3d tic[], Matrix3d ric[])
{
    // ROS_INFO("FeatureManager::triangulate");
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        it_per_id.used_num1 = it_per_id.feature_per_frame1.size();
        // assert(it_per_id.used_num == it_per_id.used_num1);

        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        if (it_per_id.estimated_depth > 0)
            continue;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        int imu_i1 = it_per_id.start_frame1, imu_j1 = imu_i1 - 1;
        // assert(imu_i == imu_i1);

        // ROS_ASSERT(NUM_OF_CAM == 2);
        Eigen::MatrixXd svd_A(2 * it_per_id.feature_per_frame.size(), 4);
        int svd_idx = 0;
        Eigen::MatrixXd svd_A1(2 * it_per_id.feature_per_frame1.size(), 4);
        int svd_idx1 = 0;

        Eigen::Matrix<double, 3, 4> P0;
        Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
        Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
        P0.leftCols<3>() = Eigen::Matrix3d::Identity();
        P0.rightCols<1>() = Eigen::Vector3d::Zero();

        Eigen::Matrix<double, 3, 4> P0r;
        Eigen::Vector3d t0r = Ps[imu_i1] + Rs[imu_i1] * tic[1];
        Eigen::Matrix3d R0r = Rs[imu_i1] * ric[1];
        P0r.leftCols<3>() = Eigen::Matrix3d::Identity();
        P0r.rightCols<1>() = Eigen::Vector3d::Zero();

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;

            Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];
            Eigen::Vector3d t = R0.transpose() * (t1 - t0);
            Eigen::Matrix3d R = R0.transpose() * R1;
            Eigen::Matrix<double, 3, 4> P;
            P.leftCols<3>() = R.transpose();
            P.rightCols<1>() = -R.transpose() * t;
            Eigen::Vector3d f = it_per_frame.point.normalized();
            svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);
            svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);

            if (imu_i == imu_j)
                continue;
        }

        for (auto &it_per_frame : it_per_id.feature_per_frame1)
        {
            imu_j1++;

            Eigen::Vector3d t1 = Ps[imu_j1] + Rs[imu_j1] * tic[1];
            Eigen::Matrix3d R1 = Rs[imu_j1] * ric[1];
            Eigen::Vector3d t = R0r.transpose() * (t1 - t0r);
            Eigen::Matrix3d R = R0r.transpose() * R1;
            Eigen::Matrix<double, 3, 4> P;
            P.leftCols<3>() = R.transpose();
            P.rightCols<1>() = -R.transpose() * t;
            Eigen::Vector3d f = it_per_frame.point.normalized();
            svd_A1.row(svd_idx1++) = f[0] * P.row(2) - f[2] * P.row(0);
            svd_A1.row(svd_idx1++) = f[1] * P.row(2) - f[2] * P.row(1);

            if (imu_i1 == imu_j1)
                continue;
        }

        // ROS_ASSERT(svd_idx == svd_A.rows());
        Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
        double svd_method = svd_V[2] / svd_V[3];
        //it_per_id->estimated_depth = -b / A;
        //it_per_id->estimated_depth = svd_V[2] / svd_V[3];

        it_per_id.estimated_depth = svd_method;
        //it_per_id->estimated_depth = INIT_DEPTH;

        if (it_per_id.estimated_depth < 0.1)
            it_per_id.estimated_depth = INIT_DEPTH;

        // ROS_ASSERT(svd_idx1 == svd_A1.rows());
        Eigen::Vector4d svd_V1 = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A1, Eigen::ComputeThinV).matrixV().rightCols<1>();
        double svd_method1 = svd_V1[2] / svd_V1[3];
        it_per_id.estimated_depth1 = svd_method1;

        if (it_per_id.estimated_depth1 < 0.1)
            it_per_id.estimated_depth1 = INIT_DEPTH;
    }
}

void FeatureManager::removeOutlier()
{
    ROS_BREAK();
    int i = -1;
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        i += it->used_num != 0;
        if (it->used_num != 0 && it->is_outlier == true)
        {
            feature.erase(it);
        }
    }
}

void FeatureManager::removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P,
                                          Eigen::Matrix3d marg_R1, Eigen::Vector3d marg_P1, Eigen::Matrix3d new_R1, Eigen::Vector3d new_P1)
{
    // ROS_INFO("removeBackShiftDepth");
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
        {
            it->start_frame--;
            it->start_frame1--;
            // assert(it->start_frame == it->start_frame1);
        }
        else
        {
            Eigen::Vector3d uv_i = it->feature_per_frame[0].point;
            Eigen::Vector3d uv_i1 = it->feature_per_frame1[0].point;

            it->feature_per_frame.erase(it->feature_per_frame.begin());
            it->feature_per_frame1.erase(it->feature_per_frame1.begin());
            // assert(it->feature_per_frame.size() == it->feature_per_frame1.size());

            if (it->feature_per_frame.size() < 2)
            {
                feature.erase(it);
                continue;
            }
            else
            {
                Eigen::Vector3d pts_i = uv_i * it->estimated_depth;
                Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;
                Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P);
                double dep_j = pts_j(2);
                if (dep_j > 0)
                    it->estimated_depth = dep_j;
                else
                    it->estimated_depth = INIT_DEPTH;

                Eigen::Vector3d pts_i1 = uv_i1 * it->estimated_depth1;
                Eigen::Vector3d w_pts_i1 = marg_R1 * pts_i1 + marg_P1;
                Eigen::Vector3d pts_j1 = new_R1.transpose() * (w_pts_i1 - new_P1);
                double dep_j1 = pts_j1(2);
                if (dep_j1 > 0)
                    it->estimated_depth1 = dep_j1;
                else
                    it->estimated_depth1 = INIT_DEPTH;
            }
        }
    }
}

void FeatureManager::removeBack()
{
    // ROS_INFO("removeBack");
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
        {
            it->start_frame--;
            it->start_frame1--;
            // assert(it->start_frame == it->start_frame1);
        }
        else
        {
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            it->feature_per_frame1.erase(it->feature_per_frame1.begin());
            // assert(it->feature_per_frame.size() == it->feature_per_frame1.size());

            if (it->feature_per_frame.size() == 0)
                feature.erase(it);
        }
    }
}

void FeatureManager::removeFront(int frame_count)
{
    // ROS_INFO("removeFront");
    for (auto it = feature.begin(), it_next = feature.begin(); it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame == frame_count)
        {
            it->start_frame--;
            it->start_frame1--;
            //assert(it->start_frame == it->start_frame1);
        }
        else
        {
            int j = WINDOW_SIZE - 1 - it->start_frame;
            if (it->endFrame() < frame_count - 1)
                continue;
            it->feature_per_frame.erase(it->feature_per_frame.begin() + j);
            it->feature_per_frame1.erase(it->feature_per_frame1.begin() + j);
            //assert(it->feature_per_frame.size() == it->feature_per_frame1.size());

            if (it->feature_per_frame.size() == 0)
                feature.erase(it);
        }
    }
}

double FeatureManager::compensatedParallax2(const FeaturePerId &it_per_id, int frame_count)
{
    //check the second last frame is keyframe or not
    //parallax betwwen seconde last frame and third last frame
    const FeaturePerFrame &frame_i = it_per_id.feature_per_frame[frame_count - 2 - it_per_id.start_frame];
    const FeaturePerFrame &frame_j = it_per_id.feature_per_frame[frame_count - 1 - it_per_id.start_frame];

    double ans = 0;
    Vector3d p_j = frame_j.point;

    double u_j = p_j(0);
    double v_j = p_j(1);

    Vector3d p_i = frame_i.point;
    Vector3d p_i_comp;

    p_i_comp = p_i;
    double dep_i = p_i(2);
    double u_i = p_i(0) / dep_i;
    double v_i = p_i(1) / dep_i;
    double du = u_i - u_j, dv = v_i - v_j;

    double dep_i_comp = p_i_comp(2);
    double u_i_comp = p_i_comp(0) / dep_i_comp;
    double v_i_comp = p_i_comp(1) / dep_i_comp;
    double du_comp = u_i_comp - u_j, dv_comp = v_i_comp - v_j;

    ans = max(ans, sqrt(min(du * du + dv * dv, du_comp * du_comp + dv_comp * dv_comp)));

    return ans;
}