#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <Eigen/Dense>
#include <deque>

namespace amr_imu_filters {

class IMUFilterCore {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    IMUFilterCore(const Eigen::Vector3d& offset = Eigen::Vector3d::Zero());

    void setMovingAverageWindowSize(size_t size);
    void setKalmanNoiseParams(double process_noise, double measurement_noise);
    void setOffsetVector(const Eigen::Vector3d& offset);

    sensor_msgs::Imu filterNoise(const sensor_msgs::Imu& imu_msg);
    sensor_msgs::Imu compensatePosition(const sensor_msgs::Imu& imu_msg);

   private:
    Eigen::Vector3d offset_;
    Eigen::Matrix<double, 6, 6> Q_, R_;
    Eigen::Matrix<double, 6, 1> state_;
    Eigen::Matrix<double, 6, 6> covariance_;
    size_t window_size_;
    std::deque<Eigen::Vector3d> acc_buffer_;
    std::deque<Eigen::Vector3d> gyro_buffer_;
    double last_timestamp_;

    Eigen::Vector3d movingAverage(std::deque<Eigen::Vector3d>& buffer,
                                  const Eigen::Vector3d& new_data);
    Eigen::Matrix<double, 6, 1> kalmanFilter(
        const Eigen::Matrix<double, 6, 1>& measurement);
};

}  // namespace amr_imu_filters