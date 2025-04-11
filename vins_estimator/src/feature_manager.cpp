#include "feature_manager.h"

int FeaturePerId::endFrame()
{
    return start_frame + feature_per_frame.size() - 1;
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
    int cnt = 0;
    for (auto &it : feature)
    {

        it.used_num = it.feature_per_frame.size();

        if (it.used_num >= 2 && it.start_frame < WINDOW_SIZE - 2)
        {
            cnt++;
        }
    }
    return cnt;
}


bool FeatureManager::addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double td, ros::Publisher* parallax_pub, const std_msgs::Header& header)
{
    ROS_DEBUG("input feature: %d", (int)image.size());  // image.size()：當前幀中的特徵點數量
    ROS_DEBUG("num of feature: %d", getFeatureCount()); // getFeatureCount()：目前 FeatureManager 內部追蹤的特徵點總數。
    double parallax_sum = 0;    // 累積所有特徵點的視差。
    int parallax_num = 0;   // 計算視差的數量。
    last_track_num = 0; // 上一幀繼續被追蹤的特徵點數量。
    for (auto &id_pts : image)  // 遍歷當前幀的所有特徵點    
    {
        // FeaturePerFrame：每帧基本的数据-特征点[x,y,z,u,v,vx,vy]和td：IMU与cam同步时间差 
        //特征点管理器，存储特征点格式：首先按照特征点ID，一个一个存储，每个ID会包含其在不同帧上的位置
        //这里id_pts.second[0].second获取的信息为：xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y
        FeaturePerFrame f_per_fra(id_pts.second[0].second, td);

        //迭代器寻找feature list中是否有这feature_id
        //获取feature_id
        int feature_id = id_pts.first;
        auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId &it)
                          {
            return it.feature_id == feature_id;
                          });
        
        //如果没有则新建一个，并添加这图像帧
        if (it == feature.end()) // 新特徵，加入 FeatureManager
        {
            feature.push_back(FeaturePerId(feature_id, frame_count));
            feature.back().feature_per_frame.push_back(f_per_fra);
        }
        //有的话把图像帧添加进去
        else if (it->feature_id == feature_id)
        {
             /**
             * 如果找到了相同ID特征点，就在其FeaturePerFrame内增加此特征点在此帧的位置以及其他信息，
             * it的feature_per_frame容器中存放的是该feature能够被哪些帧看到，存放的是在这些帧中该特征点的信息
             * 所以，feature_per_frame.size的大小就表示有多少个帧可以看到该特征点
             * */
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num++;
        }
    }

    //若追踪次数小于20或者窗口内帧的数目小于2，是关键帧 
    if (frame_count < 2 || last_track_num < 20){
        if (parallax_pub)
        {
            std_msgs::Float64MultiArray msg;
            msg.data.push_back(header.stamp.toSec());
            msg.data.push_back(frame_count);
            msg.data.push_back(last_track_num);
            msg.data.push_back(std::numeric_limits<double>::quiet_NaN());
            parallax_pub->publish(msg);
        }
        return true;
    }
        
    // 计算每个特征在次新帧和次次新帧中的视差
    for (auto &it_per_id : feature) // 計算視差
    {   
        // 观测该特征点的：起始帧小于倒数第三帧，终止帧要大于倒数第二帧，保证至少有两帧能观测到。
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1)
        {
            // 总视差：该特征点在两帧的归一化平面上的坐标点的距离ans
            parallax_sum += compensatedParallax2(it_per_id, frame_count);
            parallax_num++;
        }
    }

    // add pub
    
    double avg_parallax = 0.0;
    if (parallax_num > 0)
        avg_parallax = parallax_sum / parallax_num;

    if (parallax_pub)
    {
        std_msgs::Float64MultiArray msg;
        msg.data.push_back(header.stamp.toSec());
        msg.data.push_back(frame_count);
        msg.data.push_back(last_track_num);
        msg.data.push_back(avg_parallax);
        parallax_pub->publish(msg);
    }
    //第一次加进去的，是关键帧
    if (parallax_num == 0)
    {
        return true;
    }
    else
    {
        // 平均视差大于阈值的是关键帧
        ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        ROS_DEBUG("current parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
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
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        it_per_id.estimated_depth = 1.0 / x(++feature_index);
        //ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
        if (it_per_id.estimated_depth < 0)
        {
            it_per_id.solve_flag = 2;
        }
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
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        it_per_id.estimated_depth = 1.0 / x(++feature_index);
    }
}

VectorXd FeatureManager::getDepthVector()
{
    VectorXd dep_vec(getFeatureCount());
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
#if 1
        dep_vec(++feature_index) = 1. / it_per_id.estimated_depth;
#else
        dep_vec(++feature_index) = it_per_id->estimated_depth;
#endif
    }
    return dep_vec;
}

void FeatureManager::triangulate(Vector3d Ps[], Vector3d tic[], Matrix3d ric[])
{
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2)) //這個特徵至少被追蹤到兩個 frame。
            continue;

        if (it_per_id.estimated_depth > 0)//如果這個特徵點已經有深度估計，就跳過不處理
            continue;

        //只支援單相機系統（VINS-Mono 預設就是），初始化三角測量的參照幀是 imu_i
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        ROS_ASSERT(NUM_OF_CAM == 1);
        Eigen::MatrixXd svd_A(2 * it_per_id.feature_per_frame.size(), 4);
        int svd_idx = 0;

        //這是第一幀的投影矩陣 P0（設為世界座標系原點），方便之後做相對位姿
        Eigen::Matrix<double, 3, 4> P0;
        Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
        Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
        P0.leftCols<3>() = Eigen::Matrix3d::Identity();
        P0.rightCols<1>() = Eigen::Vector3d::Zero();

        //組建 SVD 所需的線性方程組 A
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            //計算第 j 幀與參照幀之間的相對位姿（轉換到參照幀座標系下）
            imu_j++;

            Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];

            //建立相對幀的投影矩陣
            Eigen::Vector3d t = R0.transpose() * (t1 - t0);
            Eigen::Matrix3d R = R0.transpose() * R1;
            Eigen::Matrix<double, 3, 4> P;
            P.leftCols<3>() = R.transpose();
            P.rightCols<1>() = -R.transpose() * t;

            //使用線性三角測量公式，對每個觀測建立兩個線性方程，加到 svd_A 中。
            Eigen::Vector3d f = it_per_frame.point.normalized();
            svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);
            svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);

            if (imu_i == imu_j)
                continue;
        }
        ROS_ASSERT(svd_idx == svd_A.rows());
        //使用 SVD 解三角測量
        Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
        double svd_method = svd_V[2] / svd_V[3];
        //it_per_id->estimated_depth = -b / A;
        //it_per_id->estimated_depth = svd_V[2] / svd_V[3];

        it_per_id.estimated_depth = svd_method;
        //it_per_id->estimated_depth = INIT_DEPTH;

        if (it_per_id.estimated_depth < 0.1)
        {
            it_per_id.estimated_depth = INIT_DEPTH;
        }

    }
}
// void FeatureManager::triangulate(Vector3d Ps[], Vector3d tic[], Matrix3d ric[])
// {
//     constexpr double FIXED_DEPTH = 30.0; // 依據你 UAV 的實際高度決定

//     for (auto &it_per_id : feature)
//     {
//         it_per_id.used_num = it_per_id.feature_per_frame.size();
//         if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
//             continue;

//         // 直接指定固定深度
//         it_per_id.estimated_depth = FIXED_DEPTH;
//     }
// }



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

void FeatureManager::removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P)
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            Eigen::Vector3d uv_i = it->feature_per_frame[0].point;  
            it->feature_per_frame.erase(it->feature_per_frame.begin());
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
            }
        }
        // remove tracking-lost feature after marginalize
        /*
        if (it->endFrame() < WINDOW_SIZE - 1)
        {
            feature.erase(it);
        }
        */
    }
}

void FeatureManager::removeBack()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() == 0)
                feature.erase(it);
        }
    }
}

void FeatureManager::removeFront(int frame_count)
{
    for (auto it = feature.begin(), it_next = feature.begin(); it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame == frame_count)
        {
            it->start_frame--;
        }
        else
        {
            int j = WINDOW_SIZE - 1 - it->start_frame;
            if (it->endFrame() < frame_count - 1)
                continue;
            it->feature_per_frame.erase(it->feature_per_frame.begin() + j);
            if (it->feature_per_frame.size() == 0)
                feature.erase(it);
        }
    }
}

// 计算视差：该特征点在两帧的归一化平面上的坐标点的距离ans
double FeatureManager::compensatedParallax2(const FeaturePerId &it_per_id, int frame_count)
{
    //check the second last frame is keyframe or not
    //parallax betwwen seconde last frame and third last frame
    const FeaturePerFrame &frame_i = it_per_id.feature_per_frame[frame_count - 2 - it_per_id.start_frame];
    const FeaturePerFrame &frame_j = it_per_id.feature_per_frame[frame_count - 1 - it_per_id.start_frame];

    double ans = 0; // 视差
    Vector3d p_j = frame_j.point;   // 3D路标点（倒数第二帧j）

    double u_j = p_j(0);    // 归一化坐标
    double v_j = p_j(1);

    Vector3d p_i = frame_i.point;   // 3D路标点 (倒数第三帧i)
    Vector3d p_i_comp;

    //int r_i = frame_count - 2;
    //int r_j = frame_count - 1;
    //p_i_comp = ric[camera_id_j].transpose() * Rs[r_j].transpose() * Rs[r_i] * ric[camera_id_i] * p_i;
    p_i_comp = p_i;
    double dep_i = p_i(2);  // Z
    double u_i = p_i(0) / dep_i;    // 归一化坐标
    double v_i = p_i(1) / dep_i;
    double du = u_i - u_j, dv = v_i - v_j;

    double dep_i_comp = p_i_comp(2);
    double u_i_comp = p_i_comp(0) / dep_i_comp;
    double v_i_comp = p_i_comp(1) / dep_i_comp;
    double du_comp = u_i_comp - u_j, dv_comp = v_i_comp - v_j;

    ans = max(ans, sqrt(min(du * du + dv * dv, du_comp * du_comp + dv_comp * dv_comp)));

    return ans;
}


// double FeatureManager::compensatedParallax2(const FeaturePerId &it_per_id, int frame_count)
// {
//     int idx_i = frame_count - 2 - it_per_id.start_frame;
//     int idx_j = frame_count - 1 - it_per_id.start_frame;

//     // 明確檢查索引範圍（非常重要）
//     if (idx_i < 0 || idx_j >= it_per_id.feature_per_frame.size())
//         return 0.0;

//     const FeaturePerFrame &frame_i = it_per_id.feature_per_frame[idx_i];
//     const FeaturePerFrame &frame_j = it_per_id.feature_per_frame[idx_j];

//     const Vector3d &p_i = frame_i.point;
//     const Vector3d &p_j = frame_j.point;

//     // 正確做法：p_i, p_j已經是歸一化座標，不需再除以depth
//     double du = p_i(0) - p_j(0);
//     double dv = p_i(1) - p_j(1);
    
//     double ans = sqrt(du * du + dv * dv);

//     return ans;
// }
