#ifndef MSF_LOCALIZATION_KALMAN_FILTER_ERROR_STATE_KALMAN_FILTER_HPP_
#define MSF_LOCALIZATION_KALMAN_FILTER_ERROR_STATE_KALMAN_FILTER_HPP_

#include "msf_localization/kalman_filter/kalman_filter.hpp"

namespace msf_localization {

class ErrorStateKalmanFilter : public KalmanFilter {
public:
  ErrorStateKalmanFilter(const YAML::Node &node);

  void Init(const Eigen::Vector3d &vel, const IMUData &imu_data);


  bool Update(const IMUData &imu_data);

  bool Correct(const IMUData &imu_data, const MeasurementType &measurement_type,
               const Measurement &measurement);

  bool Correct(const IMUData &imu_data, const double &time, const MeasurementType &measurement_type,
               const Eigen::Matrix4f &T_nb, const Eigen::Vector3f &v_b);


  void GetOdometry(Eigen::Matrix4f &pose, Eigen::Vector3f &vel);


  void GetCovariance(Cov &cov);


  void UpdateObservabilityAnalysis(const double &time, const MeasurementType &measurement_type);


  bool SaveObservabilityAnalysis(const MeasurementType &measurement_type);

private:
  static constexpr int kDimState{15};

  static constexpr int kIndexErrorPos{0};
  static constexpr int kIndexErrorVel{3};
  static constexpr int kIndexErrorOri{6};
  static constexpr int kIndexErrorAccel{9};
  static constexpr int kIndexErrorGyro{12};

  static constexpr int kDimProcessNoise{12};

  static constexpr int kIndexNoiseAccel{0};
  static constexpr int kIndexNoiseGyro{3};
  static constexpr int kIndexNoiseBiasAccel{6};
  static constexpr int kIndexNoiseBiasGyro{9};

  // dimensions:
  static constexpr int kDimMeasurementPose{6};
  static constexpr int kDimMeasurementPoseNoise{6};

  static constexpr int kDimMeasurementPoseVel{9};
  static constexpr int kDimMeasurementPoseVelNoise{9};

  static constexpr int kDimMeasurementPosiVel{6};
  static constexpr int kDimMeasurementPosiVelNoise{6};

  // state:
  using VectorX=Eigen::Matrix<double, kDimState, 1>;
  using MatrixP=Eigen::Matrix<double, kDimState, kDimState>;

  // process equation:
  using MatrixF=Eigen::Matrix<double, kDimState, kDimState>;
  using MatrixB=Eigen::Matrix<double, kDimState, kDimProcessNoise>;
  using MatrixQ=Eigen::Matrix<double, kDimProcessNoise, kDimProcessNoise>;

  // measurement equation:
  using MatrixGPose=Eigen::Matrix<double, kDimMeasurementPose,kDimState> ;
  using MatrixCPose=Eigen::Matrix<double, kDimMeasurementPose,kDimMeasurementPoseNoise>;
  using MatrixRPose=Eigen::Matrix<double, kDimMeasurementPoseNoise,kDimMeasurementPoseNoise>;

  using MatrixGPoseVel=Eigen::Matrix<double, kDimMeasurementPoseVel,kDimState> ;
  using MatrixCPoseVel=Eigen::Matrix<double, kDimMeasurementPoseVel,kDimMeasurementPoseVelNoise>;
  using MatrixRPoseVel=Eigen::Matrix<double, kDimMeasurementPoseVelNoise,kDimMeasurementPoseVelNoise>;

  using MatrixGPosiVel=Eigen::Matrix<double, kDimMeasurementPosiVel,kDimState> ;
  using MatrixCPosiVel=Eigen::Matrix<double, kDimMeasurementPosiVel,kDimMeasurementPosiVelNoise>;
  using MatrixRPosiVel=Eigen::Matrix<double, kDimMeasurementPosiVelNoise,kDimMeasurementPosiVelNoise>;

  // measurement:
  using VectorYPose=Eigen::Matrix<double, kDimMeasurementPose, 1>;
  using VectorYPoseVel=Eigen::Matrix<double, kDimMeasurementPoseVel, 1>;
  using VectorYPosiVel=Eigen::Matrix<double, kDimMeasurementPosiVel, 1>;

  // Kalman gain:
  using MatrixKPose=Eigen::Matrix<double, kDimState, kDimMeasurementPose>;
  using MatrixKPoseVel=Eigen::Matrix<double, kDimState, kDimMeasurementPoseVel>;
  using MatrixKPosiVel=Eigen::Matrix<double, kDimState, kDimMeasurementPosiVel>;

  // state observality matrix:
  using MatrixQPose=Eigen::Matrix<double, kDimState * kDimMeasurementPose, kDimState>;
  using MatrixQPoseVel=Eigen::Matrix<double, kDimState * kDimMeasurementPoseVel, kDimState>;
  using MatrixQPosiVel=Eigen::Matrix<double, kDimState * kDimMeasurementPosiVel, kDimState>;


  Eigen::Vector3d GetUnbiasedAngularVel(const Eigen::Vector3d &angular_vel,
                                        const Eigen::Matrix3d &R);

  Eigen::Vector3d GetUnbiasedLinearAcc(const Eigen::Vector3d &linear_acc,
                                       const Eigen::Matrix3d &R);


  bool GetAngularDelta(const size_t index_curr, const size_t index_prev,
                       Eigen::Vector3d &angular_delta,
                       Eigen::Vector3d &angular_vel_mid);

  bool GetVelocityDelta(const size_t index_curr, const size_t index_prev,
                        const Eigen::Matrix3d &R_curr,
                        const Eigen::Matrix3d &R_prev, double &T,
                        Eigen::Vector3d &velocity_delta,
                        Eigen::Vector3d &linear_acc_mid);

  void UpdateOrientation(const Eigen::Vector3d &angular_delta,
                         Eigen::Matrix3d &R_curr, Eigen::Matrix3d &R_prev);

  void UpdatePosition(const double &T, const Eigen::Vector3d &velocity_delta);

  void UpdateOdomEstimation(Eigen::Vector3d &linear_acc_mid,
                            Eigen::Vector3d &angular_vel_mid);


  void SetProcessEquation(const Eigen::Matrix3d &C_nb,
                          const Eigen::Vector3d &f_n,
                          const Eigen::Vector3d &w_n);

  void UpdateProcessEquation(const Eigen::Vector3d &linear_acc_mid,
                             const Eigen::Vector3d &angular_vel_mid);


  void UpdateErrorEstimation(const double &T,
                             const Eigen::Vector3d &linear_acc_mid,
                             const Eigen::Vector3d &angular_vel_mid);


  void CorrectErrorEstimationPose(const Eigen::Matrix4d &T_nb,
                                  Eigen::VectorXd &Y, Eigen::MatrixXd &G,
                                  Eigen::MatrixXd &K);


  void CorrectErrorEstimationPoseVel(
      const Eigen::Matrix4d &T_nb, const Eigen::Vector3d &v_b, const Eigen::Vector3d &w_b,
      Eigen::VectorXd &Y, Eigen::MatrixXd &G, Eigen::MatrixXd &K
  );


  void CorrectErrorEstimationPosiVel(
      const Eigen::Matrix4d &T_nb, const Eigen::Vector3d &v_b, const Eigen::Vector3d &w_b,
      Eigen::VectorXd &Y, Eigen::MatrixXd &G, Eigen::MatrixXd &K
  );


  void CorrectErrorEstimation(const MeasurementType &measurement_type,
                              const Measurement &measurement);


  void EliminateError(void);


  bool IsCovStable(const int INDEX_OFSET, const double THRESH = 1.0e-5);


  void ResetState(void);

  void ResetCovariance(void);


  void GetQPose(Eigen::MatrixXd &Q, Eigen::VectorXd &Y);

  // odometry estimation from IMU integration:
  Eigen::Matrix4d init_pose_ = Eigen::Matrix4d::Identity();

  Eigen::Matrix4d pose_ = Eigen::Matrix4d::Identity();
  Eigen::Vector3d vel_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d accl_bias_ = Eigen::Vector3d::Zero();

  // state:
  VectorX X_ = VectorX::Zero();
  MatrixP P_ = MatrixP::Zero();
  // process & measurement equations:
  MatrixF F_ = MatrixF::Zero();
  MatrixB B_ = MatrixB::Zero();
  MatrixQ Q_ = MatrixQ::Zero();

  MatrixGPose GPose_ = MatrixGPose::Zero();
  MatrixCPose CPose_ = MatrixCPose::Zero();
  MatrixRPose RPose_ = MatrixRPose::Zero();
  MatrixQPose QPose_ = MatrixQPose::Zero();

  MatrixGPoseVel GPoseVel_ = MatrixGPoseVel::Zero();
  MatrixCPoseVel CPoseVel_ = MatrixCPoseVel::Zero();
  MatrixRPoseVel RPoseVel_ = MatrixRPoseVel::Zero();
  MatrixQPoseVel QPoseVel_ = MatrixQPoseVel::Zero();

  MatrixGPosiVel GPosiVel_ = MatrixGPosiVel::Zero();
  MatrixCPosiVel CPosiVel_ = MatrixCPosiVel::Zero();
  MatrixRPosiVel RPosiVel_ = MatrixRPosiVel::Zero();
  MatrixQPosiVel QPosiVel_ = MatrixQPosiVel::Zero();

  // measurement:
  VectorYPose YPose_ = VectorYPose::Zero();
  VectorYPoseVel YPoseVel_ = VectorYPoseVel::Zero(); 
  VectorYPosiVel YPosiVel_ = VectorYPosiVel::Zero(); 
};

} // namespace lidar_localization

#endif // MSF_LOCALIZATION_KALMAN_FILTER_ERROR_STATE_KALMAN_FILTER_HPP_