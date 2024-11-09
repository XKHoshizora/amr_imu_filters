/**
 * @file advanced_imu_filter.cpp
 * @brief Advanced IMU filter node with EKF and complementary filter options
 *
 * @details This node subscribes to raw IMU data and optionally magnetometer data,
 *          applies either an EKF or complementary filter, and publishes filtered data.
 */

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Dense>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <dynamic_reconfigure/server.h>
#include "amr_imu_filters/ImuFilterConfig.h"

#include <memory>

class ImuFilter {
public:
    enum class FilterType {
        COMPLEMENTARY,
        EKF
    };

    struct State {
        Eigen::Vector4d orientation; // 四元数
        Eigen::Vector3d gyro_bias;   // 陀螺仪偏差

        State() {
            orientation << 1, 0, 0, 0;  // 单位四元数
            gyro_bias.setZero();
        }
    };

    ImuFilter(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
        : nh_(nh)
        , private_nh_(private_nh)
        , diagnostic_updater_(nh_, private_nh_)
        , min_freq_(15.0)
        , max_freq_(25.0)
        , expected_publish_freq_(20.0)
        , initialized_(false)
        , has_mag_data_(false)
        , last_imu_time_(0) {

        loadParameters();
        initializeFilter();
        setupDiagnostics();
        setupDynamicReconfigure();

        // 发布者和订阅者设置
        filtered_imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/filtered", 10);
        imu_sub_ = nh_.subscribe("imu", 10, &ImuFilter::imuCallback, this);

        if (use_mag_) {
            filtered_mag_pub_ = nh_.advertise<sensor_msgs::MagneticField>("mag/filtered", 10);
            mag_sub_ = nh_.subscribe("magnetic", 10, &ImuFilter::magCallback, this);
        }

        // 设置频率诊断
        freq_diagnostic_ = std::make_shared<diagnostic_updater::HeaderlessTopicDiagnostic>(
            "IMU Filter", diagnostic_updater_,
            diagnostic_updater::FrequencyStatusParam(&min_freq_, &max_freq_, 0.1, 10));

        diagnostic_updater_.add("IMU Filter Status", this, &ImuFilter::checkFilterFrequency);

        publish_timer_ = private_nh_.createTimer(
            ros::Duration(1.0 / expected_publish_freq_),
            &ImuFilter::publishDiagnostics, this);
    }

    ~ImuFilter() {}

private:
    void loadParameters() {
        private_nh_.param<std::string>("filter_type", filter_type_str_, "EKF");
        private_nh_.param<double>("alpha", alpha_, 0.96);
        private_nh_.param<double>("dt", dt_, 0.05);
        private_nh_.param<bool>("use_mag", use_mag_, false);
        private_nh_.param<std::string>("fixed_frame", fixed_frame_, "odom");
        private_nh_.param<double>("expected_publish_freq", expected_publish_freq_, 20.0);

        // EKF参数
        private_nh_.param<double>("process_noise_gyro", process_noise_gyro_, 0.001);
        private_nh_.param<double>("process_noise_accel", process_noise_accel_, 0.001);
        private_nh_.param<double>("process_noise_bias", process_noise_bias_, 0.0001);
        private_nh_.param<double>("measurement_noise_accel", measurement_noise_accel_, 0.01);
        private_nh_.param<double>("measurement_noise_mag", measurement_noise_mag_, 0.01);

        filter_type_ = (filter_type_str_ == "EKF") ? FilterType::EKF : FilterType::COMPLEMENTARY;
    }

    void initializeFilter() {
        // 初始化EKF状态和协方差矩阵
        state_ = State();
        P_ = Eigen::MatrixXd::Identity(7, 7) * 0.1;  // 7x7: 4(四元数) + 3(陀螺仪偏差)

        // 初始化过程噪声协方差
        Q_ = Eigen::MatrixXd::Zero(7, 7);
        Q_.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * process_noise_gyro_;
        Q_.block<3,3>(4,4) = Eigen::Matrix3d::Identity() * process_noise_bias_;

        // 初始化测量噪声协方差
        R_accel_ = Eigen::Matrix3d::Identity() * measurement_noise_accel_;
        if (use_mag_) {
            R_mag_ = Eigen::Matrix3d::Identity() * measurement_noise_mag_;
            mag_reference_ = Eigen::Vector3d(1, 0, 0);  // 假设磁北为x轴正方向
        }

        // 初始化互补滤波器变量
        roll_ = pitch_ = yaw_ = 0.0;
        q_est_.setRPY(0.0, 0.0, 0.0);
    }

    void setupDiagnostics() {
        diagnostic_updater_.setHardwareID("IMU Filter");
        diagnostic_updater_.add("IMU Status", this, &ImuFilter::updateDiagnostics);
    }

    void setupDynamicReconfigure() {
        dynamic_reconfigure::Server<amr_imu_filters::ImuFilterConfig>::CallbackType cb;
        cb = boost::bind(&ImuFilter::configCallback, this, _1, _2);
        config_server_.setCallback(cb);
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        if (!initialized_) {
            initializeState(msg);
            return;
        }

        // 检查时间戳
        ros::Time current_time = msg->header.stamp;
        if (last_imu_time_.toSec() > 0) {
            dt_ = (current_time - last_imu_time_).toSec();
        }
        last_imu_time_ = current_time;

        // 检查数据有效性
        if (!isValidImuData(msg)) {
            ROS_WARN_THROTTLE(1.0, "Invalid IMU data received");
            return;
        }

        // 根据选择的滤波器类型进行处理
        if (filter_type_ == FilterType::EKF) {
            processEKF(msg);
        } else {
            processComplementary(msg);
        }

        // 发布滤波后的数据
        publishFilteredImu(msg);

        // 更新诊断信息
        diagnostic_updater_.update();
    }

    void magCallback(const sensor_msgs::MagneticField::ConstPtr& msg) {
        if (!use_mag_) return;

        latest_mag_data_ = *msg;
        has_mag_data_ = true;
    }

    void publishFilteredImu(const sensor_msgs::Imu::ConstPtr& msg) {
        sensor_msgs::Imu filtered_msg;
        filtered_msg.header = msg->header;
        filtered_msg.header.frame_id = fixed_frame_;

        if (filter_type_ == FilterType::EKF) {
            // 从EKF状态获取姿态四元数
            filtered_msg.orientation.w = state_.orientation(0);
            filtered_msg.orientation.x = state_.orientation(1);
            filtered_msg.orientation.y = state_.orientation(2);
            filtered_msg.orientation.z = state_.orientation(3);

            // 设置协方差
            Eigen::Matrix3d orientation_cov = P_.block<3,3>(0,0);
            for (int i = 0; i < 9; i++) {
                filtered_msg.orientation_covariance[i] = orientation_cov(i/3, i%3);
            }

            // 补偿陀螺仪偏差后的角速度
            filtered_msg.angular_velocity.x = msg->angular_velocity.x - state_.gyro_bias(0);
            filtered_msg.angular_velocity.y = msg->angular_velocity.y - state_.gyro_bias(1);
            filtered_msg.angular_velocity.z = msg->angular_velocity.z - state_.gyro_bias(2);
        } else {
            // 从互补滤波器获取姿态四元数
            filtered_msg.orientation.w = q_est_.w();
            filtered_msg.orientation.x = q_est_.x();
            filtered_msg.orientation.y = q_est_.y();
            filtered_msg.orientation.z = q_est_.z();

            // 设置固定协方差
            for (int i = 0; i < 9; i++) {
                filtered_msg.orientation_covariance[i] = 0.001;
            }

            // 保持原始角速度
            filtered_msg.angular_velocity = msg->angular_velocity;
        }

        // 保持原始加速度数据
        filtered_msg.linear_acceleration = msg->linear_acceleration;
        filtered_msg.angular_velocity_covariance = msg->angular_velocity_covariance;
        filtered_msg.linear_acceleration_covariance = msg->linear_acceleration_covariance;

        // 发布过滤后的IMU数据
        filtered_imu_pub_.publish(filtered_msg);

        // 如果有磁力计数据，发布过滤后的磁力计数据
        if (use_mag_ && has_mag_data_) {
            sensor_msgs::MagneticField filtered_mag_msg;
            filtered_mag_msg.header = msg->header;
            filtered_mag_msg.header.frame_id = fixed_frame_;

            // 应用滤波器的旋转到磁力计数据
            Eigen::Vector3d mag_filtered;
            if (filter_type_ == FilterType::EKF) {
                mag_filtered = quatToRotMat(state_.orientation) *
                    Eigen::Vector3d(latest_mag_data_.magnetic_field.x,
                                  latest_mag_data_.magnetic_field.y,
                                  latest_mag_data_.magnetic_field.z);
            } else {
                tf2::Vector3 mag_vec(latest_mag_data_.magnetic_field.x,
                                   latest_mag_data_.magnetic_field.y,
                                   latest_mag_data_.magnetic_field.z);
                mag_vec = tf2::quatRotate(q_est_, mag_vec);
                mag_filtered << mag_vec.x(), mag_vec.y(), mag_vec.z();
            }

            filtered_mag_msg.magnetic_field.x = mag_filtered(0);
            filtered_mag_msg.magnetic_field.y = mag_filtered(1);
            filtered_mag_msg.magnetic_field.z = mag_filtered(2);

            // 复制协方差
            for(int i = 0; i < 9; i++) {
                filtered_mag_msg.magnetic_field_covariance[i] = latest_mag_data_.magnetic_field_covariance[i];
            }

            filtered_mag_pub_.publish(filtered_mag_msg);
        }

        // 更新发布频率统计
        publish_frequency_status_.tick();

        if (freq_diagnostic_) {
            freq_diagnostic_->tick();
        }
    }

    void checkFilterFrequency(diagnostic_updater::DiagnosticStatusWrapper &stat) {
        double freq = filtered_imu_pub_.getNumSubscribers() > 0 ? expected_publish_freq_ : 0.0;

        if (freq < min_freq_) {
            stat.summary(diagnostic_msgs::DiagnosticStatus::WARN,
                        "Filter frequency too low");
        } else if (freq > max_freq_) {
            stat.summary(diagnostic_msgs::DiagnosticStatus::WARN,
                        "Filter frequency too high");
        } else {
            stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
                        "Filter frequency nominal");
        }

        stat.add("Frequency", freq);
        stat.add("Target Frequency", expected_publish_freq_);
        stat.add("Minimum Frequency", min_freq_);
        stat.add("Maximum Frequency", max_freq_);
    }

    void updateDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat) {
        if (!initialized_) {
            stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Not initialized");
            return;
        }

        if ((ros::Time::now() - last_imu_time_).toSec() > 0.5) {
            stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "No IMU data received");
        } else {
            stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Operating normally");
        }

        stat.add("Filter type", filter_type_str_);
        stat.add("Using magnetometer", use_mag_);
        stat.add("Last update", last_imu_time_.toSec());
    }

    void processEKF(const sensor_msgs::Imu::ConstPtr& msg) {
        // 提取IMU数据
        Eigen::Vector3d gyro(msg->angular_velocity.x,
                            msg->angular_velocity.y,
                            msg->angular_velocity.z);

        Eigen::Vector3d accel(msg->linear_acceleration.x,
                             msg->linear_acceleration.y,
                             msg->linear_acceleration.z);

        // 1. 预测步骤
        // 修正陀螺仪偏差
        gyro -= state_.gyro_bias;

        // 预测四元数
        Eigen::Matrix4d Omega = getOmegaMatrix(gyro);
        state_.orientation += (Omega * state_.orientation) * dt_ / 2.0;
        state_.orientation.normalize();

        // 更新状态协方差
        Eigen::MatrixXd F = getStateTransitionMatrix(gyro);
        P_ = F * P_ * F.transpose() + Q_ * dt_;

        // 2. 更新步骤
        updateWithAccelerometer(accel);

        if (use_mag_ && has_mag_data_) {
            Eigen::Vector3d mag(latest_mag_data_.magnetic_field.x,
                              latest_mag_data_.magnetic_field.y,
                              latest_mag_data_.magnetic_field.z);
            updateWithMagnetometer(mag);
        }
    }

    void processComplementary(const sensor_msgs::Imu::ConstPtr& msg) {
            // 提取加速度数据
            double ax = msg->linear_acceleration.x;
            double ay = msg->linear_acceleration.y;
            double az = msg->linear_acceleration.z;

            // 提取角速度数据
            double gx = msg->angular_velocity.x;
            double gy = msg->angular_velocity.y;
            double gz = msg->angular_velocity.z;

            // 使用加速度计算姿态角
            double roll_acc = atan2(ay, sqrt(ax * ax + az * az));
            double pitch_acc = atan2(-ax, sqrt(ay * ay + az * az));

            // 使用陀螺仪数据进行姿态积分
            double roll_gyro = roll_ + gx * dt_;
            double pitch_gyro = pitch_ + gy * dt_;
            double yaw_gyro = yaw_ + gz * dt_;

            // 互补滤波
            roll_ = alpha_ * roll_gyro + (1.0 - alpha_) * roll_acc;
            pitch_ = alpha_ * pitch_gyro + (1.0 - alpha_) * pitch_acc;

            // 如果有磁力计数据，使用它来修正偏航角
            if (use_mag_ && has_mag_data_) {
                double yaw_mag = calculateYawFromMag();
                yaw_ = alpha_ * yaw_gyro + (1.0 - alpha_) * yaw_mag;
            } else {
                yaw_ = yaw_gyro;
            }

            // 更新四元数
            q_est_.setRPY(roll_, pitch_, yaw_);
        }

        double calculateYawFromMag() {
            // 使用磁力计数据计算偏航角
            // 需要考虑倾斜补偿
            double mx = latest_mag_data_.magnetic_field.x;
            double my = latest_mag_data_.magnetic_field.y;
            double mz = latest_mag_data_.magnetic_field.z;

            // 倾斜补偿
            double cos_roll = cos(roll_);
            double sin_roll = sin(roll_);
            double cos_pitch = cos(pitch_);
            double sin_pitch = sin(pitch_);

            double Xh = mx * cos_pitch + my * sin_roll * sin_pitch + mz * cos_roll * sin_pitch;
            double Yh = my * cos_roll - mz * sin_roll;

            return atan2(-Yh, Xh);
        }

        void configCallback(amr_imu_filters::ImuFilterConfig &config, uint32_t /*level*/) {
            alpha_ = config.alpha;
            process_noise_gyro_ = config.process_noise_gyro;
            process_noise_accel_ = config.process_noise_accel;
            process_noise_bias_ = config.process_noise_bias;
            measurement_noise_accel_ = config.measurement_noise_accel;
            measurement_noise_mag_ = config.measurement_noise_mag;

            // 更新滤波器类型
            filter_type_ = (config.filter_type == "EKF") ? FilterType::EKF : FilterType::COMPLEMENTARY;
            filter_type_str_ = config.filter_type;

            // 更新是否使用磁力计
            bool old_use_mag = use_mag_;
            use_mag_ = config.use_mag;

            // 如果磁力计使用状态改变，需要更新订阅器
            if (old_use_mag != use_mag_) {
                if (use_mag_) {
                    filtered_mag_pub_ = nh_.advertise<sensor_msgs::MagneticField>("mag/filtered", 10);
                    mag_sub_ = nh_.subscribe("magnetic", 10, &ImuFilter::magCallback, this);
                } else {
                    if (mag_sub_) {
                        mag_sub_.shutdown();
                    }
                    if (filtered_mag_pub_) {
                        filtered_mag_pub_.shutdown();
                    }
                }
            }

            // 更新EKF参数
            if (filter_type_ == FilterType::EKF) {
                Q_.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * process_noise_gyro_;
                Q_.block<3,3>(4,4) = Eigen::Matrix3d::Identity() * process_noise_bias_;
                R_accel_ = Eigen::Matrix3d::Identity() * measurement_noise_accel_;
                if (use_mag_) {
                    R_mag_ = Eigen::Matrix3d::Identity() * measurement_noise_mag_;
                }
            }

            ROS_INFO("IMU filter parameters updated");
        }

        bool isValidImuData(const sensor_msgs::Imu::ConstPtr& msg) {
            const double MAX_ACCELERATION = 16.0;  // 16g
            const double MAX_ANGULAR_VEL = 7.0;    // 7 rad/s

            double acc_magnitude = sqrt(
                pow(msg->linear_acceleration.x, 2) +
                pow(msg->linear_acceleration.y, 2) +
                pow(msg->linear_acceleration.z, 2));

            double gyro_magnitude = sqrt(
                pow(msg->angular_velocity.x, 2) +
                pow(msg->angular_velocity.y, 2) +
                pow(msg->angular_velocity.z, 2));

            return (acc_magnitude < MAX_ACCELERATION &&
                    gyro_magnitude < MAX_ANGULAR_VEL);
        }

        void publishDiagnostics(const ros::TimerEvent& /*event*/) {
            diagnostic_updater_.force_update();
        }

        // Helper functions for EKF
        Eigen::Matrix4d getOmegaMatrix(const Eigen::Vector3d& w) {
            Eigen::Matrix4d Omega;
            Omega <<  0, -w(0), -w(1), -w(2),
                    w(0),     0,  w(2), -w(1),
                    w(1), -w(2),     0,  w(0),
                    w(2),  w(1), -w(0),     0;
            return Omega;
        }

        Eigen::MatrixXd getStateTransitionMatrix(const Eigen::Vector3d& w) {
            Eigen::MatrixXd F = Eigen::MatrixXd::Identity(7, 7);
            F.block<4,4>(0,0) = Eigen::Matrix4d::Identity() + getOmegaMatrix(w) * dt_ / 2.0;
            return F;
        }

        void updateWithAccelerometer(const Eigen::Vector3d& accel) {
            // 计算预测的重力方向
            Eigen::Vector3d g_pred = quatToRotMat(state_.orientation) * Eigen::Vector3d(0, 0, 1);

            // 归一化测量值
            Eigen::Vector3d accel_norm = accel.normalized();

            // 计算创新向量
            Eigen::Vector3d innovation = accel_norm - g_pred;

            // 计算测量雅可比矩阵
            Eigen::MatrixXd H = getAccelMeasurementJacobian();

            // 计算卡尔曼增益
            Eigen::MatrixXd S = H * P_ * H.transpose() + R_accel_;
            Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

            // 更新状态
            Eigen::VectorXd delta_x = K * innovation;
            updateState(delta_x);

            // 更新协方差
            P_ = (Eigen::MatrixXd::Identity(7, 7) - K * H) * P_;
        }

        void updateWithMagnetometer(const Eigen::Vector3d& mag) {
            // 计算预测的磁场方向
            Eigen::Vector3d m_pred = quatToRotMat(state_.orientation) * mag_reference_;

            // 归一化测量值
            Eigen::Vector3d mag_norm = mag.normalized();

            // 计算创新向量
            Eigen::Vector3d innovation = mag_norm - m_pred;

            // 计算测量雅可比矩阵
            Eigen::MatrixXd H = getMagMeasurementJacobian();

            // 计算卡尔曼增益
            Eigen::MatrixXd S = H * P_ * H.transpose() + R_mag_;
            Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

            // 更新状态
            Eigen::VectorXd delta_x = K * innovation;
            updateState(delta_x);

            // 更新协方差
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
            // 更新四元数
            Eigen::Vector3d delta_theta = delta_x.segment<3>(0);
            Eigen::Vector4d delta_q;
            delta_q << 1, delta_theta(0)/2, delta_theta(1)/2, delta_theta(2)/2;
            state_.orientation = quaternionMultiply(state_.orientation, delta_q);
            state_.orientation.normalize();

            // 更新陀螺仪偏差
            state_.gyro_bias += delta_x.segment<3>(4);
        }

        Eigen::Matrix3d quatToRotMat(const Eigen::Vector4d& q) {
            double q0 = q(0), q1 = q(1), q2 = q(2), q3 = q(3);
            Eigen::Matrix3d R;
            R << 1-2*(q2*q2+q3*q3), 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2),
                2*(q1*q2+q0*q3), 1-2*(q1*q1+q3*q3), 2*(q2*q3-q0*q1),
                2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), 1-2*(q1*q1+q2*q2);
            return R;
        }

        Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v) {
            Eigen::Matrix3d m;
            m << 0, -v(2), v(1),
                v(2), 0, -v(0),
                -v(1), v(0), 0;
            return m;
        }

        Eigen::Vector4d quaternionMultiply(const Eigen::Vector4d& q1, const Eigen::Vector4d& q2) {
            Eigen::Vector4d q;
            q(0) = q1(0)*q2(0) - q1(1)*q2(1) - q1(2)*q2(2) - q1(3)*q2(3);
            q(1) = q1(0)*q2(1) + q1(1)*q2(0) + q1(2)*q2(3) - q1(3)*q2(2);
            q(2) = q1(0)*q2(2) - q1(1)*q2(3) + q1(2)*q2(0) + q1(3)*q2(1);
            q(3) = q1(0)*q2(3) + q1(1)*q2(2) - q1(2)*q2(1) + q1(3)*q2(0);
            return q;
        }

        void initializeState(const sensor_msgs::Imu::ConstPtr& msg) {
            // 使用初始加速度计数据估计初始姿态
            double ax = msg->linear_acceleration.x;
            double ay = msg->linear_acceleration.y;
            double az = msg->linear_acceleration.z;

            double roll = atan2(ay, sqrt(ax*ax + az*az));
            double pitch = atan2(-ax, sqrt(ay*ay + az*az));
            double yaw = 0.0;  // 无法从加速度计确定偏航角

            if (filter_type_ == FilterType::EKF) {
                tf2::Quaternion q;
                q.setRPY(roll, pitch, yaw);
                state_.orientation << q.w(), q.x(), q.y(), q.z();
                state_.gyro_bias.setZero();
            } else {
                roll_ = roll;
                pitch_ = pitch;
                yaw_ = yaw;
                q_est_.setRPY(roll, pitch, yaw);
            }

            initialized_ = true;
            last_imu_time_ = msg->header.stamp;
            ROS_INFO("IMU Filter initialized");
        }

    private:
        // NodeHandles
        ros::NodeHandle& nh_;
        ros::NodeHandle& private_nh_;

        // Publishers and Subscribers
        ros::Publisher filtered_imu_pub_;
        ros::Publisher filtered_mag_pub_;
        ros::Subscriber imu_sub_, mag_sub_;
        ros::Timer publish_timer_;

        // Diagnostic tools
        diagnostic_updater::Updater diagnostic_updater_;
        std::shared_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> freq_diagnostic_;
        double min_freq_;
        double max_freq_;

        // Dynamic reconfigure
        dynamic_reconfigure::Server<amr_imu_filters::ImuFilterConfig> config_server_;

        // Filter parameters
        FilterType filter_type_;
        std::string filter_type_str_;
        double alpha_;
        double dt_;
        bool use_mag_;
        std::string fixed_frame_;
        double expected_publish_freq_;
        double min_freq_, max_freq_;

        // EKF variables
        State state_;
        Eigen::MatrixXd P_, Q_;
        Eigen::Matrix3d R_accel_, R_mag_;
        Eigen::Vector3d mag_reference_;

        // Filter noise parameters
        double process_noise_gyro_;
        double process_noise_accel_;
        double process_noise_bias_;
        double measurement_noise_accel_;
        double measurement_noise_mag_;

        // Complementary filter variables
        double roll_, pitch_, yaw_;
        tf2::Quaternion q_est_;

        // Status variables
        bool initialized_;
        bool has_mag_data_;
        ros::Time last_imu_time_;
        sensor_msgs::MagneticField latest_mag_data_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "advanced_imu_filter");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // 设置异常处理
    try {
        // 创建滤波器对象
        ImuFilter filter(nh, private_nh);

        // 使用多线程异步处理器来提高性能
        ros::AsyncSpinner spinner(2); // 使用2个线程
        spinner.start();

        // 等待关闭
        ros::waitForShutdown();

        spinner.stop();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in IMU filter node: %s", e.what());
        return 1;
    } catch (...) {
        ROS_ERROR("Unknown exception in IMU filter node");
        return 1;
    }

    return 0;
}