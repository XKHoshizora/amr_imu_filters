/**
 * @file advanced_imu_filter.cpp
 * @brief 高级IMU滤波器，支持EKF和互补滤波
 * @details 支持动态参数配置、诊断功能和参数调优
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
    // 滤波器状态结构体
    struct FilterState {
        Eigen::Vector4d orientation;  // 四元数 [w,x,y,z]
        Eigen::Vector3d gyro_bias;    // 陀螺仪偏差

        FilterState() {
            orientation << 1, 0, 0, 0;  // 初始化为单位四元数
            gyro_bias.setZero();
        }
    };

    // 构造函数
    ImuFilter(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
        : nh_(nh)
        , private_nh_(private_nh)
        , diagnostic_updater_(nh, private_nh)
        , initialized_(false)
        , has_mag_data_(false) {

        loadParameters();       // 加载参数
        initializeFilter();     // 初始化滤波器
        setupPublishersSubscribers();  // 设置发布者和订阅者
        setupDiagnostics();    // 设置诊断
        setupDynamicReconfigure(); // 设置动态参数配置

        ROS_INFO("IMU Filter initialized successfully");
    }

private:
    // 参数加载函数
    void loadParameters() {
        // 加载话题名称和基本参数
        private_nh_.param<std::string>("input_topic", input_topic_, "imu/data");
        private_nh_.param<std::string>("output_topic", output_topic_, "imu/filtered");
        private_nh_.param<int>("subscriber_queue_size", subscriber_queue_size_, 50);
        private_nh_.param<int>("num_threads", num_threads_, 2);

        // 加载滤波器配置
        private_nh_.param<std::string>("filter_type", filter_type_, "EKF");
        private_nh_.param<bool>("use_mag", use_mag_, false);
        private_nh_.param<std::string>("fixed_frame", fixed_frame_, "odom");
        private_nh_.param<double>("expected_publish_freq", publish_freq_, 0.0);

        // 加载验证阈值
        private_nh_.param<double>("max_acceleration", max_acceleration_, 32.0);
        private_nh_.param<double>("max_angular_vel", max_angular_vel_, 14.0);

        // 加载滤波器参数
        private_nh_.param<double>("alpha", alpha_, 0.96);
        private_nh_.param<double>("process_noise_gyro", process_noise_gyro_, 0.0001);
        private_nh_.param<double>("process_noise_accel", process_noise_accel_, 0.001);
        private_nh_.param<double>("process_noise_bias", process_noise_bias_, 0.00001);
        private_nh_.param<double>("measurement_noise_accel", measurement_noise_accel_, 0.1);
        private_nh_.param<double>("measurement_noise_mag", measurement_noise_mag_, 0.01);

        // 加载静态检测参数
        private_nh_.param<double>("static_threshold", static_threshold_, 0.005);
        int temp_samples;
        private_nh_.param<int>("static_samples", temp_samples, 100);
        static_samples_ = static_cast<size_t>(temp_samples);

        // 加载初始状态参数
        private_nh_.param<double>("initial_roll", initial_roll_, 0.0);
        private_nh_.param<double>("initial_pitch", initial_pitch_, 0.0);
        private_nh_.param<double>("initial_yaw", initial_yaw_, 0.0);

        // 加载初始陀螺仪偏差
        double bias_x, bias_y, bias_z;
        private_nh_.param<double>("initial_gyro_bias_x", bias_x, 0.0);
        private_nh_.param<double>("initial_gyro_bias_y", bias_y, 0.0);
        private_nh_.param<double>("initial_gyro_bias_z", bias_z, 0.0);
        initial_gyro_bias_ << bias_x, bias_y, bias_z;

        // 输出加载的参数信息
        ROS_INFO_STREAM("Loaded parameters:\n" <<
            "Filter type: " << filter_type_ << "\n" <<
            "Use magnetometer: " << (use_mag_ ? "true" : "false") << "\n" <<
            "Queue size: " << subscriber_queue_size_ << "\n" <<
            "Max acceleration: " << max_acceleration_ << "\n" <<
            "Max angular velocity: " << max_angular_vel_);
    }

    // 设置发布者和订阅者
    void setupPublishersSubscribers() {
        // 设置IMU数据订阅者
        imu_sub_ = nh_.subscribe(input_topic_, subscriber_queue_size_,
            &ImuFilter::imuCallback, this,
            ros::TransportHints().tcpNoDelay());

        // 设置滤波后的IMU数据发布者
        filtered_imu_pub_ = nh_.advertise<sensor_msgs::Imu>(
            output_topic_, subscriber_queue_size_);

        // 如果使用磁力计，设置相关的订阅者和发布者
        if (use_mag_) {
            mag_sub_ = nh_.subscribe("magnetic", subscriber_queue_size_,
                &ImuFilter::magCallback, this,
                ros::TransportHints().tcpNoDelay());
            filtered_mag_pub_ = nh_.advertise<sensor_msgs::MagneticField>(
                "mag_filtered", subscriber_queue_size_);
        }
    }

    // 初始化滤波器
    void initializeFilter() {
        state_ = FilterState();

        if (filter_type_ == "EKF") {
            // 初始化EKF矩阵
            P_ = Eigen::MatrixXd::Identity(7, 7) * 0.1;
            Q_ = Eigen::MatrixXd::Zero(7, 7);
            Q_.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * process_noise_gyro_;
            Q_.block<3,3>(4,4) = Eigen::Matrix3d::Identity() * process_noise_bias_;
            R_accel_ = Eigen::Matrix3d::Identity() * measurement_noise_accel_;

            if (use_mag_) {
                R_mag_ = Eigen::Matrix3d::Identity() * measurement_noise_mag_;
                mag_reference_ << 1, 0, 0;  // 设置参考磁场方向
            }
        } else {
            // 初始化互补滤波器
            q_comp_.setRPY(0, 0, 0);
        }
    }

    // 设置诊断功能
    void setupDiagnostics() {
        diagnostic_updater_.setHardwareID("IMU Filter");

        // 添加基本诊断
        diagnostic_updater_.add("Filter Status", this,
            &ImuFilter::updateDiagnostics);

        // 添加详细的处理诊断
        diagnostic_updater_.add("IMU Processing",
            [this](diagnostic_updater::DiagnosticStatusWrapper &stat) {
                stat.add("Last IMU timestamp", last_imu_time_.toSec());
                stat.add("Current dt", dt_);
                stat.add("Filter initialized", initialized_);
                stat.add("Processing type", filter_type_);

                double data_delay = (ros::Time::now() - last_imu_time_).toSec();
                if (data_delay > 0.1) {
                    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN,
                        "Delayed IMU processing");
                    stat.add("Processing delay", data_delay);
                } else {
                    stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
                        "Processing normally");
                }
            });
    }

    // 设置动态参数配置
    void setupDynamicReconfigure() {
        dynamic_reconfigure::Server<amr_imu_filters::ImuFilterConfig>::CallbackType cb;
        cb = boost::bind(&ImuFilter::configCallback, this, _1, _2);
        config_server_.setCallback(cb);
    }

    // IMU数据回调函数
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);

        if (!initialized_) {
            if (!initializeState(msg)) {
                return;
            }
        }

        // 处理时间戳
        ros::Time current_time = msg->header.stamp;
        if (last_imu_time_.isZero()) {
            last_imu_time_ = current_time;
            dt_ = 1.0 / 100.0;  // 假设100Hz的标称频率
            return;
        }

        dt_ = (current_time - last_imu_time_).toSec();
        if (dt_ <= 0 || dt_ > 0.5) {
            ROS_WARN_THROTTLE(1.0, "Irregular timestamp detected, dt: %.3f", dt_);
            dt_ = 1.0 / 100.0;
        }
        last_imu_time_ = current_time;

        // 验证数据
        if (!validateImuData(msg)) {
            return;
        }

        // 根据滤波器类型处理数据
        if (filter_type_ == "EKF") {
            processEKF(msg);
        } else {
            processComplementary(msg);
        }

        // 发布滤波后的数据
        publishFilteredData();

        // 更新诊断信息
        diagnostic_updater_.update();
    }

    // 验证IMU数据
    bool validateImuData(const sensor_msgs::Imu::ConstPtr& msg) {
        // 计算加速度和角速度的模
        double acc_magnitude = sqrt(
            pow(msg->linear_acceleration.x, 2) +
            pow(msg->linear_acceleration.y, 2) +
            pow(msg->linear_acceleration.z, 2));

        double gyro_magnitude = sqrt(
            pow(msg->angular_velocity.x, 2) +
            pow(msg->angular_velocity.y, 2) +
            pow(msg->angular_velocity.z, 2));

        // 检查是否超过阈值
        if (acc_magnitude >= max_acceleration_ || gyro_magnitude >= max_angular_vel_) {
            ROS_WARN_THROTTLE(1.0,
                "Invalid IMU data: acc_mag=%.2f, gyro_mag=%.2f",
                acc_magnitude, gyro_magnitude);
            return false;
        }

        return true;
    }

    // 初始化状态
    bool initializeState(const sensor_msgs::Imu::ConstPtr& msg) {
        static std::vector<Eigen::Vector3d> gyro_samples;
        static std::vector<Eigen::Vector3d> accel_samples;

        // 收集足够的样本
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

        // 计算初始偏差和姿态
        Eigen::Vector3d measured_gyro_bias = Eigen::Vector3d::Zero();
        Eigen::Vector3d mean_accel = Eigen::Vector3d::Zero();

        for (size_t i = 0; i < static_samples_; ++i) {
            measured_gyro_bias += gyro_samples[i];
            mean_accel += accel_samples[i];
        }
        measured_gyro_bias /= static_cast<double>(static_samples_);
        mean_accel /= static_cast<double>(static_samples_);

        // 计算初始姿态
        double measured_roll = atan2(mean_accel.y(),
            sqrt(mean_accel.x()*mean_accel.x() + mean_accel.z()*mean_accel.z()));
        double measured_pitch = atan2(-mean_accel.x(), mean_accel.z());

        // 根据滤波器类型初始化状态
        if (filter_type_ == "EKF") {
            state_.gyro_bias = (initial_gyro_bias_.norm() > 1e-6) ?
                              initial_gyro_bias_ : measured_gyro_bias;

            double init_roll = (std::abs(initial_roll_) > 1e-6) ?
                             initial_roll_ : measured_roll;
            double init_pitch = (std::abs(initial_pitch_) > 1e-6) ?
                              initial_pitch_ : measured_pitch;
            double init_yaw = initial_yaw_;

            tf2::Quaternion q;
            q.setRPY(init_roll, init_pitch, init_yaw);
            state_.orientation << q.w(), q.x(), q.y(), q.z();
        } else {
            double init_roll = (std::abs(initial_roll_) > 1e-6) ?
                             initial_roll_ : measured_roll;
            double init_pitch = (std::abs(initial_pitch_) > 1e-6) ?
                              initial_pitch_ : measured_pitch;
            double init_yaw = initial_yaw_;

            q_comp_.setRPY(init_roll, init_pitch, init_yaw);
        }

        initialized_ = true;
        last_imu_time_ = msg->header.stamp;

        ROS_INFO("Filter initialized");
        return true;
    }

    // EKF处理函数
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

        // 1. 状态预测
        Eigen::Matrix4d Omega = getOmegaMatrix(gyro);
        state_.orientation += (Omega * state_.orientation) * dt_ / 2.0;
        state_.orientation.normalize();

        // 2. 协方差预测
        Eigen::MatrixXd F = getStateTransitionMatrix(gyro);
        P_ = F * P_ * F.transpose() + Q_ * dt_;

        // 3. 测量更新
        updateWithAccelerometer(accel);

        // 如果启用磁力计且有磁力计数据，进行磁力计更新
        if (use_mag_ && has_mag_data_) {
            Eigen::Vector3d mag(latest_mag_data_.magnetic_field.x,
                              latest_mag_data_.magnetic_field.y,
                              latest_mag_data_.magnetic_field.z);
            updateWithMagnetometer(mag);
        }
    }

    // 互补滤波处理函数
    void processComplementary(const sensor_msgs::Imu::ConstPtr& msg) {
        // 提取IMU数据
        double gx = msg->angular_velocity.x;
        double gy = msg->angular_velocity.y;
        double gz = msg->angular_velocity.z;

        double ax = msg->linear_acceleration.x;
        double ay = msg->linear_acceleration.y;
        double az = msg->linear_acceleration.z;

        // 获取当前欧拉角
        double roll, pitch, yaw;
        tf2::Matrix3x3(q_comp_).getRPY(roll, pitch, yaw);

        // 计算基于加速度计的姿态
        double roll_acc = atan2(ay, sqrt(ax*ax + az*az));
        double pitch_acc = atan2(-ax, az);

        // 使用陀螺仪数据更新角度
        roll += gx * dt_;
        pitch += gy * dt_;
        yaw += gz * dt_;

        // 互补滤波融合
        roll = alpha_ * roll + (1.0 - alpha_) * roll_acc;
        pitch = alpha_ * pitch + (1.0 - alpha_) * pitch_acc;

        // 更新四元数
        q_comp_.setRPY(roll, pitch, yaw);
        q_comp_.normalize();
    }

    // 使用加速度计更新EKF
    void updateWithAccelerometer(const Eigen::Vector3d& accel) {
        // 归一化加速度
        Eigen::Vector3d accel_norm = accel.normalized();

        // 预测重力方向
        Eigen::Vector3d gravity_pred = quatToRotMat(state_.orientation) *
                                     Eigen::Vector3d(0, 0, 1);

        // 计算创新向量
        Eigen::Vector3d innovation = accel_norm - gravity_pred;

        // 获取测量雅可比矩阵
        Eigen::MatrixXd H = getAccelMeasurementJacobian();

        // 计算卡尔曼增益
        Eigen::MatrixXd S = H * P_ * H.transpose() + R_accel_;
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

        // 更新状态和协方差
        Eigen::VectorXd delta_x = K * innovation;
        updateState(delta_x);
        P_ = (Eigen::MatrixXd::Identity(7, 7) - K * H) * P_;
    }

    // 使用磁力计更新EKF
    void updateWithMagnetometer(const Eigen::Vector3d& mag) {
        // 归一化磁场
        Eigen::Vector3d mag_norm = mag.normalized();

        // 预测磁场方向
        Eigen::Vector3d mag_pred = quatToRotMat(state_.orientation) * mag_reference_;

        // 计算创新向量
        Eigen::Vector3d innovation = mag_norm - mag_pred;

        // 获取测量雅可比矩阵
        Eigen::MatrixXd H = getMagMeasurementJacobian();

        // 计算卡尔曼增益
        Eigen::MatrixXd S = H * P_ * H.transpose() + R_mag_;
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

        // 更新状态和协方差
        Eigen::VectorXd delta_x = K * innovation;
        updateState(delta_x);
        P_ = (Eigen::MatrixXd::Identity(7, 7) - K * H) * P_;
    }

    // 磁力计数据回调函数
    void magCallback(const sensor_msgs::MagneticField::ConstPtr& msg) {
        if (!use_mag_) return;

        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_mag_data_ = *msg;
        has_mag_data_ = true;
    }

    // 发布滤波后的数据
    void publishFilteredData() {
        sensor_msgs::Imu filtered_msg;
        filtered_msg.header.stamp = ros::Time::now();
        filtered_msg.header.frame_id = fixed_frame_;

        // 根据滤波器类型设置方向数据
        if (filter_type_ == "EKF") {
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
            filtered_msg.orientation.w = q_comp_.w();
            filtered_msg.orientation.x = q_comp_.x();
            filtered_msg.orientation.y = q_comp_.y();
            filtered_msg.orientation.z = q_comp_.z();

            // 为互补滤波器设置固定协方差
            std::fill(filtered_msg.orientation_covariance.begin(),
                     filtered_msg.orientation_covariance.end(),
                     0.01);
        }

        // 发布滤波后的数据
        filtered_imu_pub_.publish(filtered_msg);
    }

    // 更新诊断信息
    void updateDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat) {
        double data_age = (ros::Time::now() - last_imu_time_).toSec();

        if (!initialized_) {
            stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Not initialized");
        } else if (data_age > 0.1) {
            stat.summary(diagnostic_msgs::DiagnosticStatus::WARN,
                        "Data delayed: " + std::to_string(data_age) + " seconds");
        } else {
            stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Operating normally");
        }

        // 添加基本信息
        stat.add("Filter type", filter_type_);
        stat.add("Using magnetometer", use_mag_);
        stat.add("Data age (s)", data_age);
        stat.add("dt (s)", dt_);

        // 添加EKF特定信息
        if (filter_type_ == "EKF") {
            double roll, pitch, yaw;
            Eigen::Quaterniond q(state_.orientation(0), state_.orientation(1),
                               state_.orientation(2), state_.orientation(3));
            tf2::Matrix3x3(tf2::Quaternion(q.x(), q.y(), q.z(), q.w())).getRPY(roll, pitch, yaw);

            stat.add("Roll (deg)", roll * 180.0 / M_PI);
            stat.add("Pitch (deg)", pitch * 180.0 / M_PI);
            stat.add("Yaw (deg)", yaw * 180.0 / M_PI);
            stat.add("Gyro bias X", state_.gyro_bias.x());
            stat.add("Gyro bias Y", state_.gyro_bias.y());
            stat.add("Gyro bias Z", state_.gyro_bias.z());
        }
    }

    // 动态参数配置回调函数
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

    // 辅助函数：获取Omega矩阵
    Eigen::Matrix4d getOmegaMatrix(const Eigen::Vector3d& w) {
        Eigen::Matrix4d Omega;
        Omega <<     0, -w(0), -w(1), -w(2),
                 w(0),     0,  w(2), -w(1),
                 w(1), -w(2),     0,  w(0),
                 w(2),  w(1), -w(0),     0;
        return Omega;
    }

    // 辅助函数：获取状态转移矩阵
    Eigen::MatrixXd getStateTransitionMatrix(const Eigen::Vector3d& w) {
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(7, 7);
        F.block<4,4>(0,0) = Eigen::Matrix4d::Identity() +
                            getOmegaMatrix(w) * dt_ / 2.0;
        return F;
    }

    // 辅助函数：四元数转旋转矩阵
    Eigen::Matrix3d quatToRotMat(const Eigen::Vector4d& q) {
        double q0 = q(0), q1 = q(1), q2 = q(2), q3 = q(3);
        Eigen::Matrix3d R;

        R << 1-2*(q2*q2+q3*q3),   2*(q1*q2-q0*q3),   2*(q1*q3+q0*q2),
             2*(q1*q2+q0*q3), 1-2*(q1*q1+q3*q3),   2*(q2*q3-q0*q1),
             2*(q1*q3-q0*q2),   2*(q2*q3+q0*q1), 1-2*(q1*q1+q2*q2);

        return R;
    }

    // 辅助函数：获取加速度计测量雅可比矩阵
    Eigen::MatrixXd getAccelMeasurementJacobian() {
        Eigen::Matrix3d R = quatToRotMat(state_.orientation);
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 7);
        H.block<3,3>(0,0) = R;
        return H;
    }

    // 辅助函数：获取磁力计测量雅可比矩阵
    Eigen::MatrixXd getMagMeasurementJacobian() {
        Eigen::Matrix3d R = quatToRotMat(state_.orientation);
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 7);
        H.block<3,3>(0,0) = R * skewSymmetric(mag_reference_);
        return H;
    }

    // 辅助函数：偏斜对称矩阵
    Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v) {
        Eigen::Matrix3d m;
        m <<     0, -v(2),  v(1),
             v(2),     0, -v(0),
            -v(1),  v(0),     0;
        return m;
    }

    // 辅助函数：四元数乘法
    Eigen::Vector4d quaternionMultiply(const Eigen::Vector4d& q1,
                                     const Eigen::Vector4d& q2) {
        Eigen::Vector4d q;
        q(0) = q1(0)*q2(0) - q1(1)*q2(1) - q1(2)*q2(2) - q1(3)*q2(3);
        q(1) = q1(0)*q2(1) + q1(1)*q2(0) + q1(2)*q2(3) - q1(3)*q2(2);
        q(2) = q1(0)*q2(2) - q1(1)*q2(3) + q1(2)*q2(0) + q1(3)*q2(1);
        q(3) = q1(0)*q2(3) + q1(1)*q2(2) - q1(2)*q2(1) + q1(3)*q2(0);
        return q;
    }

    // 辅助函数：更新EKF状态
    void updateState(const Eigen::VectorXd& delta_x) {
        // 更新方向
        Eigen::Vector3d delta_theta = delta_x.segment<3>(0);
        Eigen::Vector4d delta_q;
        delta_q << 1, delta_theta.x()/2, delta_theta.y()/2, delta_theta.z()/2;
        state_.orientation = quaternionMultiply(state_.orientation, delta_q);
        state_.orientation.normalize();

        // 更新陀螺仪偏差
        state_.gyro_bias += delta_x.segment<3>(4);
    }

private:
    // ROS相关成员
    ros::NodeHandle& nh_;
    ros::NodeHandle& private_nh_;
    diagnostic_updater::Updater diagnostic_updater_;

    // Communication
    ros::Subscriber imu_sub_;
    ros::Subscriber mag_sub_;
    ros::Publisher filtered_imu_pub_;
    ros::Publisher filtered_mag_pub_;

    // 话题参数
    std::string input_topic_;
    std::string output_topic_;
    std::string fixed_frame_;
    int subscriber_queue_size_;
    int num_threads_;

    // 滤波器参数
    std::string filter_type_;
    bool use_mag_;
    double publish_freq_;
    double dt_;
    double alpha_;
    double max_acceleration_;
    double max_angular_vel_;

    // EKF参数
    double process_noise_gyro_;
    double process_noise_accel_;
    double process_noise_bias_;
    double measurement_noise_accel_;
    double measurement_noise_mag_;
    double static_threshold_;
    size_t static_samples_;

    // 初始状态参数
    double initial_roll_;
    double initial_pitch_;
    double initial_yaw_;
    Eigen::Vector3d initial_gyro_bias_;

    // 滤波器状态和矩阵
    FilterState state_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd Q_;
    Eigen::Matrix3d R_accel_;
    Eigen::Matrix3d R_mag_;
    Eigen::Vector3d mag_reference_;
    tf2::Quaternion q_comp_;

    // 运行时状态
    std::mutex data_mutex_;
    bool initialized_;
    bool has_mag_data_;
    ros::Time last_imu_time_;
    sensor_msgs::MagneticField latest_mag_data_;

    // 诊断工具
    std::shared_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> freq_diagnostic_;
    dynamic_reconfigure::Server<amr_imu_filters::ImuFilterConfig> config_server_;
};

// 主函数
int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "advanced_imu_filter");

    try {
        // 创建节点句柄
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