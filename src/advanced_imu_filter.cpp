/**
 * @file advanced_imu_filter.cpp
 * @brief Advanced IMU filter with EKF and complementary filter options
 * @details Supports dynamic reconfigure, diagnostics, and parameter tuning
 */

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Dense>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <dynamic_reconfigure/server.h>
#include "amr_imu_filters/ImuFilterConfig.h"

class ImuFilter {
public:
    // 定义过滤器状态结构体
    struct FilterState {
        Eigen::Vector4d orientation;  // 四元数 [w,x,y,z]
        Eigen::Vector3d gyro_bias;    // 陀螺仪偏差

        FilterState() {
            orientation << 1, 0, 0, 0;  // 单位四元数
            gyro_bias.setZero();
        }
    };

    ImuFilter(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
        : nh_(nh)
        , private_nh_(private_nh)
        , diagnostic_updater_(nh, private_nh)
        , initialized_(false)
        , has_mag_data_(false) {

        loadParameters();
        initializeFilter();
        setupPublishersSubscribers();
        setupDiagnostics();
        setupDynamicReconfigure();

        ROS_INFO("IMU Filter initialized with following parameters:");
        ROS_INFO("Input topic: %s", input_topic_.c_str());
        ROS_INFO("Output topic: %s", output_topic_.c_str());
        ROS_INFO("Filter type: %s", filter_type_.c_str());
        ROS_INFO("Use magnetometer: %s", use_mag_ ? "true" : "false");
        ROS_INFO("Fixed frame: %s", fixed_frame_.c_str());
        ROS_INFO("Expected publish rate: %.2f Hz", publish_freq_);
    }

private:
    void loadParameters() {
        // 话题配置
        private_nh_.param<std::string>("input_topic", input_topic_, "imu");
        private_nh_.param<std::string>("output_topic", output_topic_, "imu_filtered");

        // 基本参数
        private_nh_.param<std::string>("filter_type", filter_type_, "EKF");
        private_nh_.param<bool>("use_mag", use_mag_, false);
        private_nh_.param<std::string>("fixed_frame", fixed_frame_, "odom");

        // 发布频率设置 - 默认与输入频率相同
        private_nh_.param<double>("expected_publish_freq", publish_freq_, 20.0);

        // 互补滤波器参数
        private_nh_.param<double>("alpha", alpha_, 0.96);

        // EKF参数
        private_nh_.param<double>("process_noise_gyro", process_noise_gyro_, 0.0001);
        private_nh_.param<double>("process_noise_accel", process_noise_accel_, 0.001);
        private_nh_.param<double>("process_noise_bias", process_noise_bias_, 0.00001);
        private_nh_.param<double>("measurement_noise_accel", measurement_noise_accel_, 0.1);
        private_nh_.param<double>("measurement_noise_mag", measurement_noise_mag_, 0.01);

        // 静态检测参数
        private_nh_.param<double>("static_threshold", static_threshold_, 0.005);
        int temp_samples;
        private_nh_.param<int>("static_samples", temp_samples, 100);
        static_samples_ = static_cast<size_t>(temp_samples);
    }

    void initializeFilter() {
        state_ = FilterState();

        if (filter_type_ == "EKF") {
            // 初始化EKF协方差矩阵
            P_ = Eigen::MatrixXd::Identity(7, 7) * 0.1;  // 7x7: 4(四元数) + 3(陀螺仪偏差)

            // 初始化过程噪声协方差
            Q_ = Eigen::MatrixXd::Zero(7, 7);
            Q_.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * process_noise_gyro_;
            Q_.block<3,3>(4,4) = Eigen::Matrix3d::Identity() * process_noise_bias_;

            // 初始化测量噪声协方差
            R_accel_ = Eigen::Matrix3d::Identity() * measurement_noise_accel_;
            if (use_mag_) {
                R_mag_ = Eigen::Matrix3d::Identity() * measurement_noise_mag_;
                mag_reference_ << 1, 0, 0;  // 假设磁北为x轴正方向
            }
        } else {
            // 互补滤波器初始化
            q_comp_.setRPY(0, 0, 0);
        }
    }

    void setupPublishersSubscribers() {
        // 订阅原始IMU数据
        imu_sub_ = nh_.subscribe(input_topic_, 10, &ImuFilter::imuCallback, this);

        // 发布过滤后的IMU数据
        filtered_imu_pub_ = nh_.advertise<sensor_msgs::Imu>(output_topic_, 10);

        if (use_mag_) {
            mag_sub_ = nh_.subscribe("magnetic", 10, &ImuFilter::magCallback, this);
            filtered_mag_pub_ = nh_.advertise<sensor_msgs::MagneticField>("mag_filtered", 10);
        }

        // 创建发布定时器 - 可选的基于时间的发布机制
        if (publish_freq_ > 0) {
            publish_timer_ = nh_.createTimer(
                ros::Duration(1.0/publish_freq_),
                &ImuFilter::publishTimerCallback, this);
        }
    }

    void setupDiagnostics() {
        diagnostic_updater_.setHardwareID("IMU Filter");
        diagnostic_updater_.add("Filter Status", this, &ImuFilter::updateDiagnostics);

        // 设置频率诊断
        min_freq_ = publish_freq_ * 0.9;
        max_freq_ = publish_freq_ * 1.1;

        freq_diagnostic_ = std::make_shared<diagnostic_updater::HeaderlessTopicDiagnostic>(
            "IMU Filter Output", diagnostic_updater_,
            diagnostic_updater::FrequencyStatusParam(&min_freq_, &max_freq_, 0.1, 10));
    }

    void setupDynamicReconfigure() {
        dynamic_reconfigure::Server<amr_imu_filters::ImuFilterConfig>::CallbackType cb;
        cb = boost::bind(&ImuFilter::configCallback, this, _1, _2);
        config_server_.setCallback(cb);
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        if (!initialized_) {
            if (!initializeState(msg)) {
                return;
            }
        }

        std::lock_guard<std::mutex> lock(data_mutex_);

        // 检查数据有效性
        if (!validateImuData(msg)) {
            ROS_WARN_THROTTLE(1.0, "Invalid IMU data received");
            return;
        }

        // 更新时间间隔
        ros::Time current_time = msg->header.stamp;
        if (last_imu_time_.isZero()) {
            dt_ = 1.0 / publish_freq_;
        } else {
            dt_ = (current_time - last_imu_time_).toSec();
        }
        last_imu_time_ = current_time;

        // 处理IMU数据
        if (filter_type_ == "EKF") {
            processEKF(msg);
        } else {
            processComplementary(msg);
        }

        // 如果没有使用定时发布，则在每次接收到数据时直接发布
        if (publish_freq_ <= 0) {
            publishFilteredData();
        }
    }

    void magCallback(const sensor_msgs::MagneticField::ConstPtr& msg) {
        if (!use_mag_) return;

        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_mag_data_ = *msg;
        has_mag_data_ = true;
    }

    void publishTimerCallback(const ros::TimerEvent& /*event*/) {
        // 如果使用定时发布，检查是否有新数据
        if (!initialized_ || filtered_imu_pub_.getNumSubscribers() == 0) {
            return;
        }

        // 检查数据是否过期（超过2个周期未更新）
        if ((ros::Time::now() - last_imu_time_).toSec() > 2.0/publish_freq_) {
            ROS_WARN_THROTTLE(1.0, "No recent IMU data available");
            return;
        }

        std::lock_guard<std::mutex> lock(data_mutex_);
        publishFilteredData();
        diagnostic_updater_.update();
    }

    bool initializeState(const sensor_msgs::Imu::ConstPtr& msg) {
        static std::vector<Eigen::Vector3d> gyro_samples;
        static std::vector<Eigen::Vector3d> accel_samples;

        if (gyro_samples.size() < static_samples_) {
            gyro_samples.push_back(Eigen::Vector3d(
                msg->angular_velocity.x,
                msg->angular_velocity.y,
                msg->angular_velocity.z));

            accel_samples.push_back(Eigen::Vector3d(
                msg->linear_acceleration.x,
                msg->linear_acceleration.y,
                msg->linear_acceleration.z));
            return false;
        }

        // 计算初始陀螺仪偏差
        Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
        Eigen::Vector3d mean_accel = Eigen::Vector3d::Zero();

        for (size_t i = 0; i < static_samples_; ++i) {
            gyro_bias += gyro_samples[i];
            mean_accel += accel_samples[i];
        }
        gyro_bias /= static_cast<double>(static_samples_);
        mean_accel /= static_cast<double>(static_samples_);

        // 使用平均加速度计算初始姿态
        double roll = atan2(mean_accel.y(),
            sqrt(mean_accel.x()*mean_accel.x() + mean_accel.z()*mean_accel.z()));
        double pitch = atan2(-mean_accel.x(), mean_accel.z());
        double yaw = 0.0;  // 初始偏航角设为0

        if (filter_type_ == "EKF") {
            state_.gyro_bias = gyro_bias;
            tf2::Quaternion q;
            q.setRPY(roll, pitch, yaw);
            state_.orientation << q.w(), q.x(), q.y(), q.z();
        } else {
            q_comp_.setRPY(roll, pitch, yaw);
        }

        initialized_ = true;
        last_imu_time_ = msg->header.stamp;

        ROS_INFO("Filter initialized:");
        ROS_INFO("Initial orientation (RPY): %.2f, %.2f, %.2f",
            roll*180/M_PI, pitch*180/M_PI, yaw*180/M_PI);
        ROS_INFO("Initial gyro bias: %.4f, %.4f, %.4f",
            gyro_bias.x(), gyro_bias.y(), gyro_bias.z());

        return true;
    }

    bool validateImuData(const sensor_msgs::Imu::ConstPtr& msg) {
        const double MAX_ACCELERATION = 16.0;  // 16g
        const double MAX_ANGULAR_VEL = 7.0;    // 7 rad/s

        // 检查加速度大小
        double acc_magnitude = sqrt(
            pow(msg->linear_acceleration.x, 2) +
            pow(msg->linear_acceleration.y, 2) +
            pow(msg->linear_acceleration.z, 2));

        // 检查角速度大小
        double gyro_magnitude = sqrt(
            pow(msg->angular_velocity.x, 2) +
            pow(msg->angular_velocity.y, 2) +
            pow(msg->angular_velocity.z, 2));

        return (acc_magnitude < MAX_ACCELERATION &&
                gyro_magnitude < MAX_ANGULAR_VEL);
    }

    void processEKF(const sensor_msgs::Imu::ConstPtr& msg) {
        // 提取IMU数据
        Eigen::Vector3d gyro(msg->angular_velocity.x,
                            msg->angular_velocity.y,
                            msg->angular_velocity.z);

        Eigen::Vector3d accel(msg->linear_acceleration.x,
                             msg->linear_acceleration.y,
                             msg->linear_acceleration.z);

        // 应用陀螺仪偏差校正
        gyro -= state_.gyro_bias;

        // EKF预测步骤
        // 1. 预测状态
        Eigen::Matrix4d Omega = getOmegaMatrix(gyro);
        state_.orientation += (Omega * state_.orientation) * dt_ / 2.0;
        state_.orientation.normalize();

        // 2. 预测协方差
        Eigen::MatrixXd F = getStateTransitionMatrix(gyro);
        P_ = F * P_ * F.transpose() + Q_ * dt_;

        // 3. 更新步骤
        // 使用加速度计更新
        updateWithAccelerometer(accel);

        // 如果有磁力计数据，使用磁力计更新
        if (use_mag_ && has_mag_data_) {
            Eigen::Vector3d mag(latest_mag_data_.magnetic_field.x,
                              latest_mag_data_.magnetic_field.y,
                              latest_mag_data_.magnetic_field.z);
            updateWithMagnetometer(mag);
        }
    }

    void processComplementary(const sensor_msgs::Imu::ConstPtr& msg) {
        // 提取IMU数据
        double gx = msg->angular_velocity.x;
        double gy = msg->angular_velocity.y;
        double gz = msg->angular_velocity.z;

        double ax = msg->linear_acceleration.x;
        double ay = msg->linear_acceleration.y;
        double az = msg->linear_acceleration.z;

        // 从当前四元数获取欧拉角
        double roll, pitch, yaw;
        tf2::Matrix3x3(q_comp_).getRPY(roll, pitch, yaw);

        // 计算加速度估计的姿态
        double roll_acc = atan2(ay, sqrt(ax*ax + az*az));
        double pitch_acc = atan2(-ax, sqrt(ay*ay + az*az));

        // 使用陀螺仪数据更新姿态
        roll += gx * dt_;
        pitch += gy * dt_;
        yaw += gz * dt_;

        // 互补滤波
        roll = alpha_ * roll + (1.0 - alpha_) * roll_acc;
        pitch = alpha_ * pitch + (1.0 - alpha_) * pitch_acc;

        // 如果有磁力计数据，更新偏航角
        if (use_mag_ && has_mag_data_) {
            double yaw_mag = calculateMagneticYaw();
            yaw = alpha_ * yaw + (1.0 - alpha_) * yaw_mag;
        }

        // 更新四元数
        q_comp_.setRPY(roll, pitch, yaw);
    }

    double calculateMagneticYaw() {
        // 获取当前姿态角
        double roll, pitch, yaw;
        tf2::Matrix3x3(q_comp_).getRPY(roll, pitch, yaw);

        // 磁力计数据
        double mx = latest_mag_data_.magnetic_field.x;
        double my = latest_mag_data_.magnetic_field.y;
        double mz = latest_mag_data_.magnetic_field.z;

        // 进行倾斜补偿
        double cos_roll = cos(roll);
        double sin_roll = sin(roll);
        double cos_pitch = cos(pitch);
        double sin_pitch = sin(pitch);

        // 计算水平面投影分量
        double bx = mx * cos_pitch + my * sin_roll * sin_pitch + mz * cos_roll * sin_pitch;
        double by = my * cos_roll - mz * sin_roll;

        // 计算偏航角
        return atan2(-by, bx);
    }

    void publishFilteredData() {
        // 创建并填充过滤后的IMU消息
        sensor_msgs::Imu filtered_msg;
        filtered_msg.header.stamp = ros::Time::now();
        filtered_msg.header.frame_id = fixed_frame_;

        // 根据滤波器类型设置姿态数据
        if (filter_type_ == "EKF") {
            // EKF状态转换为消息格式
            filtered_msg.orientation.w = state_.orientation(0);
            filtered_msg.orientation.x = state_.orientation(1);
            filtered_msg.orientation.y = state_.orientation(2);
            filtered_msg.orientation.z = state_.orientation(3);

            // 设置协方差
            Eigen::Matrix3d orientation_cov = P_.block<3,3>(0,0);
            for (int i = 0; i < 9; i++) {
                filtered_msg.orientation_covariance[i] = orientation_cov(i/3, i%3);
            }
        } else {
            // 互补滤波器状态转换为消息格式
            filtered_msg.orientation.w = q_comp_.w();
            filtered_msg.orientation.x = q_comp_.x();
            filtered_msg.orientation.y = q_comp_.y();
            filtered_msg.orientation.z = q_comp_.z();

            // 设置固定协方差
            std::fill(filtered_msg.orientation_covariance.begin(),
                     filtered_msg.orientation_covariance.end(),
                     0.01);
        }

        // 发布过滤后的IMU数据
        filtered_imu_pub_.publish(filtered_msg);

        // 更新发布频率诊断
        if (freq_diagnostic_) {
            freq_diagnostic_->tick();
        }

        // 如果启用磁力计，发布过滤后的磁力计数据
        if (use_mag_ && has_mag_data_ && filtered_mag_pub_.getNumSubscribers() > 0) {
            publishFilteredMagData();
        }
    }

    void publishFilteredMagData() {
        sensor_msgs::MagneticField filtered_mag_msg;
        filtered_mag_msg.header.stamp = ros::Time::now();
        filtered_mag_msg.header.frame_id = fixed_frame_;

        // 转换磁力计数据到全局坐标系
        Eigen::Vector3d mag_data(latest_mag_data_.magnetic_field.x,
                                latest_mag_data_.magnetic_field.y,
                                latest_mag_data_.magnetic_field.z);

        Eigen::Vector3d mag_global;
        if (filter_type_ == "EKF") {
            mag_global = quatToRotMat(state_.orientation) * mag_data;
        } else {
            tf2::Vector3 mag_vec(mag_data.x(), mag_data.y(), mag_data.z());
            tf2::Vector3 rotated = tf2::quatRotate(q_comp_, mag_vec);
            mag_global = Eigen::Vector3d(rotated.x(), rotated.y(), rotated.z());
        }

        filtered_mag_msg.magnetic_field.x = mag_global.x();
        filtered_mag_msg.magnetic_field.y = mag_global.y();
        filtered_mag_msg.magnetic_field.z = mag_global.z();

        // 复制原始协方差
        std::copy(latest_mag_data_.magnetic_field_covariance.begin(),
                 latest_mag_data_.magnetic_field_covariance.end(),
                 filtered_mag_msg.magnetic_field_covariance.begin());

        filtered_mag_pub_.publish(filtered_mag_msg);
    }

    void updateDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat) {
        if (!initialized_) {
            stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Not initialized");
            return;
        }

        if ((ros::Time::now() - last_imu_time_).toSec() > 1.0/publish_freq_ * 2) {
            stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "No IMU data received");
        } else {
            stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Operating normally");
        }

        stat.add("Filter type", filter_type_);
        stat.add("Using magnetometer", use_mag_);
        stat.add("Update frequency", publish_freq_);
        stat.add("Last update", last_imu_time_.toSec());

        // 添加滤波器特定的状态信息
        if (filter_type_ == "EKF") {
            stat.add("EKF orientation",
                    formatQuaternion(state_.orientation));
            stat.add("Gyro bias",
                    formatVector3d(state_.gyro_bias));
        } else {
            double roll, pitch, yaw;
            tf2::Matrix3x3(q_comp_).getRPY(roll, pitch, yaw);
            stat.add("Complementary filter RPY",
                    formatRPY(roll, pitch, yaw));
        }
    }

    void configCallback(amr_imu_filters::ImuFilterConfig &config, uint32_t /*level*/) {
        std::lock_guard<std::mutex> lock(data_mutex_);

        // 更新滤波器参数
        alpha_ = config.alpha;
        process_noise_gyro_ = config.process_noise_gyro;
        process_noise_accel_ = config.process_noise_accel;
        process_noise_bias_ = config.process_noise_bias;
        measurement_noise_accel_ = config.measurement_noise_accel;
        measurement_noise_mag_ = config.measurement_noise_mag;

        // 更新EKF噪声矩阵
        if (filter_type_ == "EKF") {
            Q_.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * process_noise_gyro_;
            Q_.block<3,3>(4,4) = Eigen::Matrix3d::Identity() * process_noise_bias_;
            R_accel_ = Eigen::Matrix3d::Identity() * measurement_noise_accel_;
            if (use_mag_) {
                R_mag_ = Eigen::Matrix3d::Identity() * measurement_noise_mag_;
            }
        }

        ROS_INFO("Filter parameters updated via dynamic reconfigure");
    }

    // EKF辅助函数
    Eigen::Matrix4d getOmegaMatrix(const Eigen::Vector3d& w) {
        Eigen::Matrix4d Omega;
        Omega <<     0, -w(0), -w(1), -w(2),
                 w(0),     0,  w(2), -w(1),
                 w(1), -w(2),     0,  w(0),
                 w(2),  w(1), -w(0),     0;
        return Omega;
    }

    Eigen::MatrixXd getStateTransitionMatrix(const Eigen::Vector3d& w) {
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(7, 7);
        F.block<4,4>(0,0) = Eigen::Matrix4d::Identity() +
                            getOmegaMatrix(w) * dt_ / 2.0;
        return F;
    }

    void updateWithAccelerometer(const Eigen::Vector3d& accel) {
        // 归一化加速度
        Eigen::Vector3d accel_norm = accel.normalized();

        // 预测的重力方向
        Eigen::Vector3d gravity_pred = quatToRotMat(state_.orientation) *
                                     Eigen::Vector3d(0, 0, 1);

        // 计算创新向量
        Eigen::Vector3d innovation = accel_norm - gravity_pred;

        // 计算测量雅可比矩阵
        Eigen::MatrixXd H = getAccelMeasurementJacobian();

        // 计算卡尔曼增益
        Eigen::MatrixXd S = H * P_ * H.transpose() + R_accel_;
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

        // 更新状态和协方差
        Eigen::VectorXd delta_x = K * innovation;
        updateState(delta_x);
        P_ = (Eigen::MatrixXd::Identity(7, 7) - K * H) * P_;
    }

    void updateWithMagnetometer(const Eigen::Vector3d& mag) {
        // 归一化磁场向量
        Eigen::Vector3d mag_norm = mag.normalized();

        // 预测的磁场方向
        Eigen::Vector3d mag_pred = quatToRotMat(state_.orientation) * mag_reference_;

        // 计算创新向量
        Eigen::Vector3d innovation = mag_norm - mag_pred;

        // 计算测量雅可比矩阵
        Eigen::MatrixXd H = getMagMeasurementJacobian();

        // 计算卡尔曼增益
        Eigen::MatrixXd S = H * P_ * H.transpose() + R_mag_;
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

        // 更新状态和协方差
        Eigen::VectorXd delta_x = K * innovation;
        updateState(delta_x);
        P_ = (Eigen::MatrixXd::Identity(7, 7) - K * H) * P_;
    }

    Eigen::MatrixXd getAccelMeasurementJacobian() {
        Eigen::Matrix3d R = quatToRotMat(state_.orientation);
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 7);
        H.block<3,3>(0,0) = R;
        return H;
    }

    Eigen::MatrixXd getMagMeasurementJacobian() {
        Eigen::Matrix3d R = quatToRotMat(state_.orientation);
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 7);
        H.block<3,3>(0,0) = R * skewSymmetric(mag_reference_);
        return H;
    }

    void updateState(const Eigen::VectorXd& delta_x) {
        // 更新姿态四元数
        Eigen::Vector3d delta_theta = delta_x.segment<3>(0);
        Eigen::Vector4d delta_q;
        delta_q << 1, delta_theta.x()/2, delta_theta.y()/2, delta_theta.z()/2;
        state_.orientation = quaternionMultiply(state_.orientation, delta_q);
        state_.orientation.normalize();

        // 更新陀螺仪偏差
        state_.gyro_bias += delta_x.segment<3>(4);
    }

    // 工具函数
    Eigen::Matrix3d quatToRotMat(const Eigen::Vector4d& q) {
        double q0 = q(0), q1 = q(1), q2 = q(2), q3 = q(3);
        Eigen::Matrix3d R;

        R << 1-2*(q2*q2+q3*q3),   2*(q1*q2-q0*q3),   2*(q1*q3+q0*q2),
             2*(q1*q2+q0*q3), 1-2*(q1*q1+q3*q3),   2*(q2*q3-q0*q1),
             2*(q1*q3-q0*q2),   2*(q2*q3+q0*q1), 1-2*(q1*q1+q2*q2);

        return R;
    }

    Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v) {
        Eigen::Matrix3d m;
        m <<     0, -v(2),  v(1),
             v(2),     0, -v(0),
            -v(1),  v(0),     0;
        return m;
    }

    Eigen::Vector4d quaternionMultiply(const Eigen::Vector4d& q1,
                                     const Eigen::Vector4d& q2) {
        Eigen::Vector4d q;
        q(0) = q1(0)*q2(0) - q1(1)*q2(1) - q1(2)*q2(2) - q1(3)*q2(3);
        q(1) = q1(0)*q2(1) + q1(1)*q2(0) + q1(2)*q2(3) - q1(3)*q2(2);
        q(2) = q1(0)*q2(2) - q1(1)*q2(3) + q1(2)*q2(0) + q1(3)*q2(1);
        q(3) = q1(0)*q2(3) + q1(1)*q2(2) - q1(2)*q2(1) + q1(3)*q2(0);
        return q;
    }

    // 格式化函数用于诊断输出
    std::string formatQuaternion(const Eigen::Vector4d& q) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(4)
            << "w:" << q(0) << " x:" << q(1)
            << " y:" << q(2) << " z:" << q(3);
        return oss.str();
    }

    std::string formatVector3d(const Eigen::Vector3d& v) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(6)
            << "x:" << v(0) << " y:" << v(1) << " z:" << v(2);
        return oss.str();
    }

    std::string formatRPY(double roll, double pitch, double yaw) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2)
            << "r:" << roll*180/M_PI
            << " p:" << pitch*180/M_PI
            << " y:" << yaw*180/M_PI;
        return oss.str();
    }

private:
    // ROS相关成员
    ros::NodeHandle& nh_;
    ros::NodeHandle& private_nh_;
    ros::Subscriber imu_sub_;
    ros::Subscriber mag_sub_;
    ros::Publisher filtered_imu_pub_;
    ros::Publisher filtered_mag_pub_;
    ros::Timer publish_timer_;

    // 话题参数
    std::string input_topic_;
    std::string output_topic_;

    // 诊断工具
    diagnostic_updater::Updater diagnostic_updater_;
    std::shared_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> freq_diagnostic_;
    double min_freq_;
    double max_freq_;

    // 动态重配置
    dynamic_reconfigure::Server<amr_imu_filters::ImuFilterConfig> config_server_;

    // 滤波器参数
    std::string filter_type_;
    std::string fixed_frame_;
    bool use_mag_;
    double publish_freq_;
    double dt_;

    // 互补滤波器参数
    double alpha_;
    tf2::Quaternion q_comp_;

    // EKF参数和状态
    FilterState state_;
    Eigen::MatrixXd P_;    // 状态协方差矩阵
    Eigen::MatrixXd Q_;    // 过程噪声协方差
    Eigen::Matrix3d R_accel_;  // 加速度计测量噪声
    Eigen::Matrix3d R_mag_;    // 磁力计测量噪声
    Eigen::Vector3d mag_reference_;  // 参考磁场向量

    // 噪声参数
    double process_noise_gyro_;
    double process_noise_accel_;
    double process_noise_bias_;
    double measurement_noise_accel_;
    double measurement_noise_mag_;
    double static_threshold_;
    size_t static_samples_{100};

    // 运行时状态
    std::mutex data_mutex_;
    bool initialized_;
    bool has_mag_data_;
    ros::Time last_imu_time_;
    sensor_msgs::MagneticField latest_mag_data_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "advanced_imu_filter");

    try {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");

        ROS_INFO("Starting IMU filter node...");

        // 创建滤波器实例
        ImuFilter filter(nh, private_nh);

        // 使用多线程异步处理器
        ros::AsyncSpinner spinner(2);
        spinner.start();

        ROS_INFO("IMU filter node is running.");

        // 等待关闭
        ros::waitForShutdown();

        spinner.stop();

        ROS_INFO("IMU filter node shutting down.");
    }
    catch (const std::exception& e) {
        ROS_ERROR_STREAM("Exception in IMU filter node: " << e.what());
        return 1;
    }
    catch (...) {
        ROS_ERROR("Unknown exception in IMU filter node");
        return 1;
    }

    return 0;
}