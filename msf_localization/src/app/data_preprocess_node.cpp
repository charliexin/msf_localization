
#include <ros/ros.h>
#include "glog/logging.h"

#include "msf_localization/global_defination/global_defination.h"
#include "msf_localization/data_process/data_preprocess_flow.hpp"

using namespace msf_localization;

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "data_preprocess_node");
    ros::NodeHandle nh;

    std::string cloud_topic;
    nh.param<std::string>("cloud_topic", cloud_topic, "/synced_cloud");

    // subscribe to
    // a. raw Velodyne measurement
    // b. raw GNSS/IMU measurement
    // publish
    // a. undistorted Velodyne measurement
    // b. lidar pose in map frame
    std::shared_ptr<DataPreprocessFlow> data_preprocess_flow_ptr = std::make_shared<DataPreprocessFlow>(nh, cloud_topic);

    // pre-process lidar point cloud at 100Hz:
    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        data_preprocess_flow_ptr->Run();

        rate.sleep();
    }

    return 0;
}