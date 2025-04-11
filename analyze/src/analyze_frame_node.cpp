#include "analyze_frame.h"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "analyze_frame_node");
    ros::NodeHandle nh;

    AnalyzeFrame analyzer(nh);
    ros::spin();
}