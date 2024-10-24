#include "amr_imu_filters/imu_filter_core.hpp"

namespace amr_imu_filters {

IMUFilterCore::IMUFilterCore(const Eigen::Vector3d& offset)
    : offset_(offset), window_size_(10), last_timestamp_(0.0) {
    state_.setZero();
    covariance_.setIdentity();
    Q_.setIdentity() * 0.01;  // 默认过程噪声
    R_.setIdentity() * 0.1;   // 默认测量噪声
}

void IMUFilterCore::setMovingAverageWindowSize(size_t size) {
    window_size_ = size;
    // 清空缓存以适应新的窗口大小
    acc_buffer_.clear();
    gyro_buffer_.clear();
}

void IMUFilterCore::setKalmanNoiseParams(double process_noise,
                                         double measurement_noise) {
    Q_.setIdentity() * process_noise;
    R_.setIdentity() * measurement_noise;
}

void IMUFilterCore::setOffsetVector(const Eigen::Vector3d& offset) {
    offset_ = offset;
}

Eigen::Vector3d IMUFilterCore::movingAverage(
    std::deque<Eigen::Vector3d>& buffer, const Eigen::Vector3d& new_data) {
    buffer.push_back(new_data);
    if (buffer.size() > window_size_) {
        buffer.pop_front();
    }

    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    for (const auto& data : buffer) {
        sum += data;
    }
    return sum / static_cast<double>(buffer.size());
}

Eigen::Matrix<double, 6, 1> IMUFilterCore::kalmanFilter(
    const Eigen::Matrix<double, 6, 1>& measurement) {
    // 预测步骤
    Eigen::Matrix<double, 6, 1> predicted_state = state_;
    Eigen::Matrix<double, 6, 6> predicted_covariance = covariance_ + Q_;

    // 计算卡尔曼增益
    Eigen::Matrix<double, 6, 6> K =
        predicted_covariance * (predicted_covariance + R_).inverse();

    // 更新步骤
    state_ = predicted_state + K * (measurement - predicted_state);
    covariance_ =
        (Eigen::Matrix<double, 6, 6>::Identity() - K) * predicted_covariance;

    return state_;
}

sensor_msgs::Imu IMUFilterCore::filterNoise(const sensor_msgs::Imu& imu_msg) {
    // 转换ROS消息到Eigen向量
    Eigen::Vector3d acc(imu_msg.linear_acceleration.x,
                        imu_msg.linear_acceleration.y,
                        imu_msg.linear_acceleration.z);

    Eigen::Vector3d gyro(imu_msg.angular_velocity.x, imu_msg.angular_velocity.y,
                         imu_msg.angular_velocity.z);

    // 应用移动平均滤波
    Eigen::Vector3d filtered_acc = movingAverage(acc_buffer_, acc);
    Eigen::Vector3d filtered_gyro = movingAverage(gyro_buffer_, gyro);

    // 应用卡尔曼滤波
    Eigen::Matrix<double, 6, 1> measurement;
    measurement << filtered_acc, filtered_gyro;
    Eigen::Matrix<double, 6, 1> filtered_state = kalmanFilter(measurement);

    // 创建滤波后的IMU消息
    sensor_msgs::Imu filtered_msg = imu_msg;
    filtered_msg.linear_acceleration.x = filtered_state(0);
    filtered_msg.linear_acceleration.y = filtered_state(1);
    filtered_msg.linear_acceleration.z = filtered_state(2);
    filtered_msg.angular_velocity.x = filtered_state(3);
    filtered_msg.angular_velocity.y = filtered_state(4);
    filtered_msg.angular_velocity.z = filtered_state(5);

    return filtered_msg;
}

sensor_msgs::Imu IMUFilterCore::compensatePosition(
    const sensor_msgs::Imu& imu_msg) {
    // 提取滤波后的状态
    Eigen::Vector3d acc(imu_msg.linear_acceleration.x,
                        imu_msg.linear_acceleration.y,
                        imu_msg.linear_acceleration.z);

    Eigen::Vector3d gyro(imu_msg.angular_velocity.x, imu_msg.angular_velocity.y,
                         imu_msg.angular_velocity.z);

    // 计算离心加速度: a_centripetal = ω × (ω × r)
    Eigen::Vector3d centripetal_acc = gyro.cross(gyro.cross(offset_));

    // 计算角加速度（如果有时间戳）
    Eigen::Vector3d tangential_acc = Eigen::Vector3d::Zero();
    if (last_timestamp_ > 0.0) {
        double dt = imu_msg.header.stamp.toSec() - last_timestamp_;
        if (dt > 0.0) {
            Eigen::Vector3d angular_acc =
                (gyro - Eigen::Vector3d(state_.tail(3))) / dt;
            // 计算切向加速度: a_tangential = α × r
            tangential_acc = angular_acc.cross(offset_);
        }
    }
    last_timestamp_ = imu_msg.header.stamp.toSec();

    // 补偿加速度
    Eigen::Vector3d compensated_acc = acc - centripetal_acc - tangential_acc;

    // 创建补偿后的IMU消息
    sensor_msgs::Imu compensated_msg = imu_msg;
    compensated_msg.linear_acceleration.x = compensated_acc.x();
    compensated_msg.linear_acceleration.y = compensated_acc.y();
    compensated_msg.linear_acceleration.z = compensated_acc.z();

    return compensated_msg;
}

}  // namespace amr_imu_filters