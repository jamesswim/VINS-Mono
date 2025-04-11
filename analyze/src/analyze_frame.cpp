#include "analyze_frame.h"

// save path
std::string rawImages_path = "/home/rvl/catkin_ws/src/VINS-Mono/analyze/raw_images/";
std::string featureImages_path = "/home/rvl/catkin_ws/src/VINS-Mono/analyze/feature_images/";

std::vector<std::string> csv_headers = {
    "timestamp",
    "raw_image",
    "feature_image",
    // 新增 odometry (no loop)
    "failure_reason",
    "z_debug_tmp","z_debug_last",
    "last_P_x","last_P_y","last_P_z",
    "tmp_P_x","tmp_P_y","tmp_P_z",
    "odom_noLoop_x", "odom_noLoop_y", "odom_noLoop_z",
    "odom_noLoop_qx", "odom_noLoop_qy", "odom_noLoop_qz", "odom_noLoop_qw",
    // 新增 relocalization odometry (no loop)
    // "relo_noLoop_x", "relo_noLoop_y", "relo_noLoop_z",
    // "relo_noLoop_qx", "relo_noLoop_qy", "relo_noLoop_qz", "relo_noLoop_qw",
    "loop_x","loop_y","loop_z",
    "loop_qx","loop_qy","loop_qz","loop_qw",
    "num_features",
    "frame_count",
    "last_track_num",
    "avg_parallax",
    "keyframe",
    "initial_staus"
    // 未來新增欄位直接加在這即可
};

void AnalyzeFrame::clearPreviousData()
{
    std::vector<std::string> folders = {
        rawImages_path,
        featureImages_path
    };

    // 刪除影像資料夾內所有檔案
    for (const auto& folder : folders)
    {
        for (const auto& entry : std::filesystem::directory_iterator(folder))
            std::filesystem::remove_all(entry.path());
    }

}


AnalyzeFrame::AnalyzeFrame(ros::NodeHandle &nh)
{
    clearPreviousData();  // 每次啟動節點時自動清除舊資料

    sub_parallax = nh.subscribe("/vins_estimator/parallax_info", 2000, &AnalyzeFrame::parallaxCallback, this);
    image_sub = nh.subscribe("/feature_tracker/raw_image", 2000, &AnalyzeFrame::imageCallback, this);
    feature_sub = nh.subscribe("/feature_tracker/feature_image", 2000, &AnalyzeFrame::featureImageCallback, this);
    feature_points_sub = nh.subscribe("/feature_tracker/feature", 2000, &AnalyzeFrame::featurePointsCallback, this);
    initial_sub = nh.subscribe("/vins_estimator/initialization_info", 2000, &AnalyzeFrame::initialStatusCallback, this);
    odom_noLoop_sub = nh.subscribe("/vins_estimator/odometry", 2000, &AnalyzeFrame::odomNoLoopCallback, this);
    // relo_noLoop_sub = nh.subscribe("/vins_estimator/relocalization_path", 2000, &AnalyzeFrame::reloNoLoopCallback, this);
    loop_pose_sub = nh.subscribe("/pose_graph/loop_pose", 2000, &AnalyzeFrame::loopPoseCallback, this);
    estimator_fail_sub = nh.subscribe("/vins_estimator/estimator_failure", 2000, &AnalyzeFrame::estimatorFailCallback, this);
    sub_z_debug = nh.subscribe("/vins_estimator/z_debug", 2000, &AnalyzeFrame::zDebugCallback, this);
    sub_tmp_p = nh.subscribe("/vins_estimator/tmp_p", 2000, &AnalyzeFrame::tmpPCallback, this);
    sub_last_p = nh.subscribe("/vins_estimator/last_p", 2000, &AnalyzeFrame::lastPCallback, this);

}

AnalyzeFrame::~AnalyzeFrame()
{
    if (log_file.is_open()) {
        log_file.close();
    }
}

void AnalyzeFrame::parallaxCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    if (msg->data.size() < 4)
        return;

    ros::Time timestamp = ros::Time(msg->data[0]);
    int frame_count = static_cast<int>(msg->data[1]);
    int last_track_num = static_cast<int>(msg->data[2]);
    double avg_parallax = msg->data[4];


    std::string keyframe = "no";
    if (frame_count < 2 || last_track_num < 20)
    {
        keyframe = "yes";  // 條件1: 初期或特徵不足
    }
    else if (!std::isnan(avg_parallax) && avg_parallax == 0)
    {
        keyframe = "yes";  // 條件2: 沒有有效視差
    }
    else if (!std::isnan(avg_parallax) && avg_parallax >= (10.0 / 460.0))
    {
        keyframe = "yes";  // 條件3: 視差足夠
    }

    insertData(timestamp, "frame_count", std::to_string(frame_count));
    insertData(timestamp, "last_track_num", std::to_string(last_track_num));
    std::ostringstream ss;
    ss << std::scientific << std::setprecision(6) << avg_parallax;
    insertData(timestamp, "avg_parallax", ss.str());
    insertData(timestamp, "keyframe", keyframe);
    
}

void AnalyzeFrame::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ros::Time timestamp = msg->header.stamp;
    cv::Mat raw_image = cv_bridge::toCvCopy(msg, "mono8")->image;

    // 設定儲存路徑 (記得建立這個資料夾)
    std::string image_folder = rawImages_path;
    std::string filename = "raw_" + std::to_string(timestamp.toNSec()) + ".png";
    std::string full_path = image_folder + filename;

    // 儲存影像
    cv::imwrite(full_path, raw_image);

    // 寫入CSV
    insertData(timestamp, "raw_image", filename);
}

void AnalyzeFrame::featureImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ros::Time timestamp = msg->header.stamp;
    std::string filename = "feature_" + std::to_string(timestamp.toNSec()) + ".png";
    cv::imwrite(featureImages_path + filename, cv_bridge::toCvCopy(msg, "bgr8")->image);

    insertData(timestamp, "feature_image", filename);
}


void AnalyzeFrame::featurePointsCallback(const sensor_msgs::PointCloudConstPtr& points)
{
    ros::Time timestamp = points->header.stamp;  // 直接用訊息header內時間戳
    int total_feature_count = points->points.size();

    insertData(timestamp, "num_features", std::to_string(total_feature_count));
}

void AnalyzeFrame::initialStatusCallback(const std_msgs::Header::ConstPtr& msg)
{
    ros::Time timestamp = msg->stamp;
    std::string initialStatus = msg->frame_id;
    insertData(timestamp, "initial_staus", initialStatus);
}

void AnalyzeFrame::odomNoLoopCallback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
    ros::Time timestamp = odom_msg->header.stamp;

    insertData(timestamp, "odom_noLoop_x", std::to_string(odom_msg->pose.pose.position.x));
    insertData(timestamp, "odom_noLoop_y", std::to_string(odom_msg->pose.pose.position.y));
    insertData(timestamp, "odom_noLoop_z", std::to_string(odom_msg->pose.pose.position.z));
    insertData(timestamp, "odom_noLoop_qx", std::to_string(odom_msg->pose.pose.orientation.x));
    insertData(timestamp, "odom_noLoop_qy", std::to_string(odom_msg->pose.pose.orientation.y));
    insertData(timestamp, "odom_noLoop_qz", std::to_string(odom_msg->pose.pose.orientation.z));
    insertData(timestamp, "odom_noLoop_qw", std::to_string(odom_msg->pose.pose.orientation.w));
}

// void AnalyzeFrame::reloNoLoopCallback(const nav_msgs::Path::ConstPtr &relo_msg)
// {
//     if (relo_msg->poses.empty())
//         return;

//     const geometry_msgs::PoseStamped& pose_stamped = relo_msg->poses.front();
//     ros::Time timestamp = pose_stamped.header.stamp;

//     const geometry_msgs::Pose& pose = pose_stamped.pose;

//     insertData(timestamp, "relo_noLoop_x", std::to_string(pose.position.x));
//     insertData(timestamp, "relo_noLoop_y", std::to_string(pose.position.y));
//     insertData(timestamp, "relo_noLoop_z", std::to_string(pose.position.z));
//     insertData(timestamp, "relo_noLoop_qx", std::to_string(pose.orientation.x));
//     insertData(timestamp, "relo_noLoop_qy", std::to_string(pose.orientation.y));
//     insertData(timestamp, "relo_noLoop_qz", std::to_string(pose.orientation.z));
//     insertData(timestamp, "relo_noLoop_qw", std::to_string(pose.orientation.w));
// }

void AnalyzeFrame::loopPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{

    ros::Time timestamp = msg->header.stamp;

    insertData(timestamp, "loop_x", std::to_string(msg->pose.position.x));
    insertData(timestamp, "loop_y", std::to_string(msg->pose.position.y));
    insertData(timestamp, "loop_z", std::to_string(msg->pose.position.z));
    insertData(timestamp, "loop_qx", std::to_string(msg->pose.orientation.x));
    insertData(timestamp, "loop_qy", std::to_string(msg->pose.orientation.y));
    insertData(timestamp, "loop_qz", std::to_string(msg->pose.orientation.z));
    insertData(timestamp, "loop_qw", std::to_string(msg->pose.orientation.w));
}

void AnalyzeFrame::estimatorFailCallback(const std_msgs::Header::ConstPtr& msg)
{
    insertData(msg->stamp, "failure_reason", msg->frame_id);
}

void AnalyzeFrame::zDebugCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    insertData(msg->header.stamp, "z_debug_tmp", std::to_string(msg->pose.position.x));
    insertData(msg->header.stamp, "z_debug_last", std::to_string(msg->pose.position.y));
}

void AnalyzeFrame::tmpPCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    insertData(msg->header.stamp, "tmp_P_x", std::to_string(msg->pose.position.x));
    insertData(msg->header.stamp, "tmp_P_y", std::to_string(msg->pose.position.y));
    insertData(msg->header.stamp, "tmp_P_z", std::to_string(msg->pose.position.z));
}

void AnalyzeFrame::lastPCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    insertData(msg->header.stamp, "last_P_x", std::to_string(msg->pose.position.x));
    insertData(msg->header.stamp, "last_P_y", std::to_string(msg->pose.position.y));
    insertData(msg->header.stamp, "last_P_z", std::to_string(msg->pose.position.z));
}


void AnalyzeFrame::insertData(const ros::Time& timestamp, const std::string& key, const std::string& value)
{
    frame_data_map[timestamp][key] = value;
    saveCsv();  // 每次插入都即時存入CSV
}

void AnalyzeFrame::saveCsv()
{
    std::ofstream outfile("/home/rvl/catkin_ws/src/VINS-Mono/analyze/analyze_frames.csv");

    // 先寫入 header（依照你事先定義的順序）
    for(size_t i = 0; i < csv_headers.size(); ++i)
        outfile << csv_headers[i] << (i < csv_headers.size() - 1 ? "," : "\n");

    // 再寫入資料
    for (const auto& frame : frame_data_map)
    {
        for(size_t i = 0; i < csv_headers.size(); ++i) {
            if (i > 0)
                outfile << ",";

            if(csv_headers[i] == "timestamp")
                outfile << frame.first;
            else {
                auto col = frame.second.find(csv_headers[i]);
                if(col != frame.second.end())
                    outfile << col->second;
            }
        }
        outfile << "\n";
    }

    outfile.close();
}