#pragma once
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <fstream>
#include <string>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <filesystem>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>



class AnalyzeFrame {
public:
    AnalyzeFrame(ros::NodeHandle &nh);
    ~AnalyzeFrame();

private:
    std::map<ros::Time, std::unordered_map<std::string, std::string>> frame_data_map;
    void parallaxCallback(const std_msgs::Float64MultiArray::ConstPtr &msg);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void featureImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void insertData(const ros::Time& timestamp, const std::string& key, const std::string& value);
    void saveCsv();
    void clearPreviousData();
    void featurePointsCallback(const sensor_msgs::PointCloudConstPtr& points);
    void initialStatusCallback(const std_msgs::Header::ConstPtr& msg);
    void odomNoLoopCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);
    // void reloNoLoopCallback(const nav_msgs::Path::ConstPtr &relo_msg);
    void loopPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void estimatorFailCallback(const std_msgs::Header::ConstPtr& msg);
    void zDebugCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void tmpPCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void lastPCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    ros::Subscriber sub_parallax;
    ros::Subscriber image_sub;
    ros::Subscriber feature_sub;
    ros::Subscriber feature_points_sub;
    ros::Subscriber initial_sub;
    ros::Subscriber odom_noLoop_sub;
    // ros::Subscriber relo_noLoop_sub;
    ros::Subscriber loop_pose_sub;
    ros::Subscriber estimator_fail_sub;
    ros::Subscriber sub_z_debug;
    ros::Subscriber sub_tmp_p;
    ros::Subscriber sub_last_p;
    std::ofstream log_file;
    int log_index = 0;
};