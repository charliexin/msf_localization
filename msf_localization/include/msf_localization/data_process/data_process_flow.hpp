#ifndef MSF_LOCALIZATION_DATA_PROCESS_DATA_PROCESS_FLOW_HPP_
#define MSF_LOCALIZATION_DATA_PROCESS_DATA_PROCESS_FLOW_HPP_


#include <ros/ros.h>
#include <deque>
#include "msf_localization/data_process/data_process.hpp"

namespace msf_Localization{

class DataProcessFlow{
public:
    DataProcessFlow();
    ~DataProcessFlow();
    bool Run();


private:
    bool InitCalibration();
    bool ReadData();
    bool HasData();

//数据判断
    bool ValidIMUData();
    bool ValidLidarData();
    bool ValidPosVelMagData();

//融合定位
    bool InitializeLocalization();
    bool PredictLocalization();
    bool CorrectLocalization0();
    bool CorrectLocalization1();
    bool PublishLocalization();
private:

    std::shared_ptr<DataProcess> data_process_ptr_;

    imu_raw_sub_ptr_;
    imu_synced_sub_ptr_;
    pose_vel_sub_ptr_;
    cloud_sub_ptr_;


    std::deque<IMUData> imu_raw_data_buff_;
    
    std::deque<IMUData> imu_synced_data_buff_;
    std::deque<CloudData> cloud_data_buff_;
    std::deque<PosVelData> pos_vel_data_buff_;




    Eigen::Matrix4f lidar_to_imu_ = Eigen::Matrix4f::Identity();
};



}


#endif