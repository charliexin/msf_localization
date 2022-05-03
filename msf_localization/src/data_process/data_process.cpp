
#include "msf_localization/data_process/data_process.hpp"




namespace msf_localization{



DataProcess::DataProcess() {

}
DataProcess::~DataProcess() {

}

bool DataProcess::Update( const IMUData &imu_data ) {
    if ( kalman_filter_ptr_->Update(imu_data) ) {
        kalman_filter_ptr_->GetOdometry(current_pose_, current_vel_ );
        return true;
    }

    return false;
}


bool DataProcess::Correct(const IMUData &imu_data,const CloudData& cloud_data,   const PosVelData &pos_vel_data,Eigen::Matrix4f& cloud_pose) {
    static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
    static Eigen::Matrix4f last_pose = init_pose_;
    static Eigen::Matrix4f predict_pose = init_pose_;

    // remove invalid measurements:
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr, *cloud_data.cloud_ptr, indices);

    // downsample:
    CloudData::SSS filtered_cloud_ptr(new CloudData::CLOUD());
    current_scan_filter_ptr_->Filter(cloud_data.cloud_ptr, filtered_cloud_ptr);

    if (!has_inited_) {
        predict_pose = current_gnss_pose_;
    }

    // matching:
    CloudData::CLOUD_PTR result_cloud_ptr(new CloudData::CLOUD());
    registration_ptr_->ScanMatch(filtered_cloud_ptr, predict_pose, result_cloud_ptr, cloud_pose);
    pcl::transformPointCloud(*cloud_data.cloud_ptr, *current_scan_ptr_, cloud_pose);

    // update predicted pose:
    step_pose = last_pose.inverse() * cloud_pose;
    predict_pose = cloud_pose * step_pose;
    last_pose = cloud_pose;

    // shall the local map be updated:
    std::vector<float> edge = local_map_segmenter_ptr_->GetEdge();
    for (int i = 0; i < 3; i++) {
        if ( fabs(cloud_pose(i, 3) - edge.at(2 * i)) > 50.0 && fabs(cloud_pose(i, 3) - edge.at(2 * i + 1)) > 50.0) {
            continue;
        }
            
        ResetLocalMap(
            cloud_pose(0,3), 
            cloud_pose(1,3), 
            cloud_pose(2,3)
        );
        break;
    }

    // set lidar measurement:
    current_measurement_.time = cloud_data.time;
    current_measurement_.T_nb = (init_pose_.inverse() * cloud_pose).cast<double>();
    current_measurement_.v_b = pos_vel_data.vel.cast<double>();
    current_measurement_.w_b = Eigen::Vector3d(
        imu_data.angular_velocity.x,
        imu_data.angular_velocity.y,
        imu_data.angular_velocity.z
    );

    // Kalman correction:
    if (kalman_filter_ptr_->Correct(imu_data, CONFIG.FUSION_STRATEGY, current_measurement_ ) ) {
        kalman_filter_ptr_->GetOdometry(current_pose_, current_vel_ );
        return true;
    }

    return false;
}

}