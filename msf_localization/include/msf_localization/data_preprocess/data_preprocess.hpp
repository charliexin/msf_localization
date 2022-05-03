#ifndef  MSF_LOCALIZATION_DATA_PREPROCESS_DATA_PREPROCESS_HPP_
#define MSF_LOCALIZATION_DATA_PREPROCESS_DATA_PREPROCESS_HPP_

#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include "glog/logging.h"


#include "msf_localization/sensor_data/velocity_data.hpp"
#include "msf_localization/sensor_data/cloud_data.hpp"

namespace msf_localization {
class DataPreprocess {
  public:
    void SetMotionInfo(float scan_period, VelocityData velocity_data);
    bool AdjustCloud(CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& output_cloud_ptr);

  private:
    inline Eigen::Matrix3f UpdateMatrix(float real_time);

  private:
    float scan_period_;
    Eigen::Vector3f velocity_;
    Eigen::Vector3f angular_rate_;
};
}
#endif