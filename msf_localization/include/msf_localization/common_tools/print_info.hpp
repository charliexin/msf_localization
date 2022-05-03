
#ifndef MSF_LOCALIZATION_COMMON_TOOLS_PRINT_INFO_HPP_
#define MSF_LOCALIZATION_COMMON_TOOLS_PRINT_INFO_HPP_

#include <cmath>
#include <string>
#include <Eigen/Dense>
#include "pcl/common/eigen.h"

namespace msf_localization {
class PrintInfo {
  public:
    static void PrintPose(std::string head, Eigen::Matrix4f pose);
};
}
#endif