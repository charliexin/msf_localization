
#include "msf_localization/data_process/data_process_flow.h"

namespace msf_Localization{

DataProcessFlow::DataProcessFlow() {


    data_process_ptr_ = std::make_shared<DataProcess>();
}
DataProcessFlow::~DataProcessFlow() {

}


bool DataProcessFlow::Run(){

    if (!InitCalibration()) {
        return false;
    }

    ReadData();

    if (HasData()) {
        if (!HasInited()) {
            //0.以组合导航的数据进行全局重定位，后面可以加激光
            if (ValidIMUData() && ValidPosVelMagData()) {
                InitializeLocalization();
            }
            
        }

        else{
            //1.IMU数据进行预测估计,且将imu数据保存备份用于ekf
            while (ValidIMUData()) {
                PredictLocalization();
            }

            //2.考虑把gnss组合导航的数据也加进来 或者只是gps
            //while (ValidPosVelMagData()) {
            //    CorrectLocalization0();
            //}
            //3.考虑cloud和gnss之间的时间戳顺序，先来的先观测更新
            while (ValidLidarData()) {
                CorrectLocalization1();
            }

        }
    }

    return true;
}


bool DataProcessFlow::InitCalibration(){
    static bool calibration_received = false;
    if (!calibration_received && lidar_to_imu_ptr_->LookupData(lidar_to_imu_)) {
        calibration_received = true;
    }

    return calibration_received;
}

bool DataProcessFlow::ReadData() {
    //imu原始数据
    imu_raw_sub_ptr_->ParseData(imu_raw_data_buff_);

    //组合导航+lidar
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    imu_synced_sub_ptr_->ParseData(imu_synced_data_buff_);
    pose_vel_sub_ptr_->ParseData(pos_vel_data_buff_);
    gnss_sub_ptr_->ParseData(gnss_data_buff_);

    //单单GPS数据，做一个松耦合的融合
    return true;
}

bool DataProcessFlow::HasData() {
    return  (!imu_raw_data_buff_.empty()) || (!cloud_data_buff_.empty()) || (!pos_vel_mag_data_buff_.empty());
}


bool DataProcessFlow::ValidIMUData() {
    current_imu_raw_data_ = imu_raw_data_buff_.front();
    imu_raw_data_buff_.pop_front();
    return true;
}

bool DataProcessFlow::ValidLidarData() {
    current_cloud_data_ = cloud_data_buff_.front();
    current_imu_synced_data_ = imu_synced_data_buff_.front();
    current_pos_vel_data_ = pos_vel_data_buff_.front();


    double diff_imu_time = current_cloud_data_.time - current_imu_synced_data_.time;
    double diff_pos_vel_time = current_cloud_data_.time - current_pos_vel_data_.time;


    if ( diff_imu_time < -0.05 || diff_pos_vel_time < -0.05 ) {
        cloud_data_buff_.pop_front();
        return false;
    }

    if (diff_imu_time > 0.05) {
        imu_synced_data_buff_.pop_front();
        return false;
    }

    if (diff_pos_vel_time > 0.05) {
        pos_vel_data_buff_.pop_front();
        return false;
    }

    cloud_data_buff_.pop_front();
    imu_synced_data_buff_.pop_front();
    pos_vel_data_buff_.pop_front();

    return true;
}

bool DataProcessFlow::ValidPosVelMagData() {
    current_pos_vel_mag_data_ = pos_vel_mag_data_buff_.front();
    pos_vel_mag_data_buff_.pop_front();
    return true;
}





bool DataProcessFlow::InitializeLocalization() {
    Eigen::Vector3f init_vel = current_pos_vel_data_.vel;
    current_gnss_data_ =  gnss_data_buff_.at(3); 
    if  (fusion_senbsers_ptr_->Initialize(current_gnss_data_.pose, init_vel, current_imu_synced_data_)){
        PublishLocalization();
        return true;
    }
    return false;
}

bool DataProcessFlow::PredictLocalization() {
    if ( fusion_senbsers_ptr_->Predict(current_imu_raw_data_) ) {
        PublishLocalization();
        return true;
    }
    return false;
}

bool DataProcessFlow::CorrectLocalization0() {
    static int gnss_imu_odom_count = 0;
    if ( 0 == (++gnss_imu_odom_count % 40)) {
        if (fusion_senbsers_ptr_->Correct(current_imu_data_, current_pos_vel_mag_data_)) {
            PublishLocalization();
            gnss_imu_odom_count = 0;
            return true;
        }
    }

    return false;
}

bool DataProcessFlow::CorrectLocalization1() {

    if ( fusion_senbsers_ptr_->Correct( current_imu_synced_data_, current_cloud_data_, current_pos_vel_data_)) {
        PublishLocalization();
        return true;
    }
    return false;
}

bool DataProcessFlow::PublishLocalization() {
    
    fusion_senbsers_ptr_->GetOdometry(fused_pose_, fused_vel_);
    return true;
}


}

