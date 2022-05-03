#ifndef  MSF_LOCALIZATION_DATA_PREPROCESS_DATA_PREPROCESS_FLOW_HPP_
#define MSF_LOCALIZATION_DATA_PREPROCESS_DATA_PREPROCESS_FLOW_HPP_

#include <ros/ros.h>
// subscriber
#include "msf_localization/subscriber/cloud_subscriber.hpp"
#include "msf_localization/subscriber/imu_subscriber.hpp"
#include "msf_localization/subscriber/velocity_subscriber.hpp"
#include "msf_localization/subscriber/gnss_subscriber.hpp"
#include "msf_localization/tf_listener/tf_listener.hpp"
// publisher
// a. synced lidar measurement
#include "msf_localization/publisher/cloud_publisher.hpp"
// b. synced IMU measurement
#include "msf_localization/publisher/imu_publisher.hpp"
// c. synced GNSS-odo measurement:
#include "msf_localization/publisher/pos_vel_publisher.hpp"
// d. synced reference trajectory:
#include "msf_localization/publisher/odometry_publisher.hpp"

// models
#include "msf_localization/models/scan_adjust/distortion_adjust.hpp"

namespace msf_localization {
class DataPreprocessFlow {
  public:
    DataPreprocessFlow(ros::NodeHandle& nh, std::string cloud_topic);

    bool Run();

  private:
    bool ReadData();
    bool InitCalibration();
    bool InitGNSS();
    bool HasData();
    bool ValidData();
    bool TransformData();
    bool PublishData();

  private:
    // subscriber
    std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
    std::shared_ptr<VelocitySubscriber> velocity_sub_ptr_;
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::shared_ptr<GNSSSubscriber> gnss_sub_ptr_;
    std::shared_ptr<TFListener> lidar_to_imu_ptr_;
    // publisher
    std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
    std::shared_ptr<IMUPublisher> imu_pub_ptr_;
    std::shared_ptr<PosVelPublisher> pos_vel_pub_ptr_;
    std::shared_ptr<OdometryPublisher> gnss_pub_ptr_;
    // models
    std::shared_ptr<DataPreprocess> data_preprocess_ptr_;

    Eigen::Matrix4f lidar_to_imu_ = Eigen::Matrix4f::Identity();

    std::deque<CloudData> cloud_data_buff_;
    std::deque<IMUData> imu_data_buff_;
    std::deque<VelocityData> velocity_data_buff_;
    std::deque<GNSSData> gnss_data_buff_;

    CloudData current_cloud_data_;
    IMUData current_imu_data_;
    VelocityData current_velocity_data_;
    GNSSData current_gnss_data_;

    PosVelData pos_vel_;
    Eigen::Matrix4f gnss_pose_ = Eigen::Matrix4f::Identity();
};
}

#endif