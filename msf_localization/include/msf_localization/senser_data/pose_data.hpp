#ifndef  MSF_LOCALIZATION_SENSOR_DATA_POSE_DATA_HPP_
#define MSF_LOCALIZATION_SENSOR_DATA_POSE_DATA_HPP_

//存放处理后的IMU姿态以及GNSS位置



#include <Eigen/Dense>

namespace msf_localization {

class PoseData {
  public:
    double time = 0.0;
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    Eigen::Vector3f vel = Eigen::Vector3f::Zero();

  public:
    Eigen::Quaternionf GetQuaternion();
};

}

#endif