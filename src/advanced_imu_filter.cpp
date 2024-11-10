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
    struct FilterState {
        Eigen::Vector4d orientation;  // quaternion [w,x,y,z]
        Eigen::Vector3d gyro_bias;    // gyroscope bias

        FilterState() {
            orientation << 1, 0, 0, 0;  // unit quaternion
            gyro_bias.setZero();
        }
    };

    ImuFilter(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
        : nh_(nh)
        , private_nh_(private_nh)
        , diagnostic_updater_(nh, private_nh)
        , initialized_(false)
        , has_mag_data_(false) {

        // Load all parameters
        loadParameters();

        // Initialize filter state and matrices
        initializeFilter();

        // Setup ROS communication
        setupPublishersSubscribers();

        // Setup diagnostics and dynamic reconfigure
        setupDiagnostics();
        setupDynamicReconfigure();

        ROS_INFO("IMU Filter initialized successfully");
    }

private:
    void loadParameters() {
        // Load topic names and basic parameters
        private_nh_.param<std::string>("input_topic", input_topic_, "imu/data");
        private_nh_.param<std::string>("output_topic", output_topic_, "imu/filtered");
        private_nh_.param<int>("subscriber_queue_size", subscriber_queue_size_, 50);
        private_nh_.param<int>("num_threads", num_threads_, 2);

        // Load filter configuration
        private_nh_.param<std::string>("filter_type", filter_type_, "EKF");
        private_nh_.param<bool>("use_mag", use_mag_, false);
        private_nh_.param<std::string>("fixed_frame", fixed_frame_, "odom");
        private_nh_.param<double>("expected_publish_freq", publish_freq_, 0.0);

        // Load validation thresholds
        private_nh_.param<double>("max_acceleration", max_acceleration_, 32.0);
        private_nh_.param<double>("max_angular_vel", max_angular_vel_, 14.0);

        // Load filter parameters
        private_nh_.param<double>("alpha", alpha_, 0.96);
        private_nh_.param<double>("process_noise_gyro", process_noise_gyro_, 0.0001);
        private_nh_.param<double>("process_noise_accel", process_noise_accel_, 0.001);
        private_nh_.param<double>("process_noise_bias", process_noise_bias_, 0.00001);
        private_nh_.param<double>("measurement_noise_accel", measurement_noise_accel_, 0.1);
        private_nh_.param<double>("measurement_noise_mag", measurement_noise_mag_, 0.01);

        // Load static detection parameters
        private_nh_.param<double>("static_threshold", static_threshold_, 0.005);
        int temp_samples;
        private_nh_.param<int>("static_samples", temp_samples, 100);
        static_samples_ = static_cast<size_t>(temp_samples);

        // Load initial state parameters
        private_nh_.param<double>("initial_roll", initial_roll_, 0.0);
        private_nh_.param<double>("initial_pitch", initial_pitch_, 0.0);
        private_nh_.param<double>("initial_yaw", initial_yaw_, 0.0);

        double bias_x, bias_y, bias_z;
        private_nh_.param<double>("initial_gyro_bias_x", bias_x, 0.0);
        private_nh_.param<double>("initial_gyro_bias_y", bias_y, 0.0);
        private_nh_.param<double>("initial_gyro_bias_z", bias_z, 0.0);
        initial_gyro_bias_ << bias_x, bias_y, bias_z;

        ROS_INFO_STREAM("Loaded parameters:\n" <<
            "Filter type: " << filter_type_ << "\n" <<
            "Use magnetometer: " << (use_mag_ ? "true" : "false") << "\n" <<
            "Queue size: " << subscriber_queue_size_ << "\n" <<
            "Max acceleration: " << max_acceleration_ << "\n" <<
            "Max angular velocity: " << max_angular_vel_);
    }

    void setupPublishersSubscribers() {
        // Set up subscribers with specified queue size
        imu_sub_ = nh_.subscribe(input_topic_, subscriber_queue_size_,
            &ImuFilter::imuCallback, this,
            ros::TransportHints().tcpNoDelay());

        // Set up publishers
        filtered_imu_pub_ = nh_.advertise<sensor_msgs::Imu>(
            output_topic_, subscriber_queue_size_);

        if (use_mag_) {
            mag_sub_ = nh_.subscribe("magnetic", subscriber_queue_size_,
                &ImuFilter::magCallback, this,
                ros::TransportHints().tcpNoDelay());
            filtered_mag_pub_ = nh_.advertise<sensor_msgs::MagneticField>(
                "mag_filtered", subscriber_queue_size_);
        }
    }

    void initializeFilter() {
        state_ = FilterState();

        if (filter_type_ == "EKF") {
            // Initialize EKF matrices
            P_ = Eigen::MatrixXd::Identity(7, 7) * 0.1;
            Q_ = Eigen::MatrixXd::Zero(7, 7);
            Q_.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * process_noise_gyro_;
            Q_.block<3,3>(4,4) = Eigen::Matrix3d::Identity() * process_noise_bias_;
            R_accel_ = Eigen::Matrix3d::Identity() * measurement_noise_accel_;

            if (use_mag_) {
                R_mag_ = Eigen::Matrix3d::Identity() * measurement_noise_mag_;
                mag_reference_ << 1, 0, 0;
            }
        } else {
            q_comp_.setRPY(0, 0, 0);
        }
    }

    void setupDiagnostics() {
        diagnostic_updater_.setHardwareID("IMU Filter");

        // Add basic diagnostics
        diagnostic_updater_.add("Filter Status", this,
            &ImuFilter::updateDiagnostics);

        // Add detailed processing diagnostics
        diagnostic_updater_.add("IMU Processing", this,
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

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);

        if (!initialized_) {
            if (!initializeState(msg)) {
                return;
            }
        }

        // Process timestamps
        ros::Time current_time = msg->header.stamp;
        if (last_imu_time_.isZero()) {
            last_imu_time_ = current_time;
            dt_ = 1.0 / 100.0;  // Assume 100Hz nominal rate
            return;
        }

        dt_ = (current_time - last_imu_time_).toSec();
        if (dt_ <= 0 || dt_ > 0.5) {
            ROS_WARN_THROTTLE(1.0, "Irregular timestamp detected, dt: %.3f", dt_);
            dt_ = 1.0 / 100.0;
        }
        last_imu_time_ = current_time;

        // Validate data
        if (!validateImuData(msg)) {
            return;
        }

        // Process data based on filter type
        if (filter_type_ == "EKF") {
            processEKF(msg);
        } else {
            processComplementary(msg);
        }

        // Publish filtered data
        publishFilteredData();

        // Update diagnostics
        diagnostic_updater_.update();
    }

    bool validateImuData(const sensor_msgs::Imu::ConstPtr& msg) {
        double acc_magnitude = sqrt(
            pow(msg->linear_acceleration.x, 2) +
            pow(msg->linear_acceleration.y, 2) +
            pow(msg->linear_acceleration.z, 2));

        double gyro_magnitude = sqrt(
            pow(msg->angular_velocity.x, 2) +
            pow(msg->angular_velocity.y, 2) +
            pow(msg->angular_velocity.z, 2));

        if (acc_magnitude >= max_acceleration_ || gyro_magnitude >= max_angular_vel_) {
            ROS_WARN_THROTTLE(1.0,
                "Invalid IMU data: acc_mag=%.2f, gyro_mag=%.2f",
                acc_magnitude, gyro_magnitude);
            return false;
        }

        return true;
    }

    [Rest of the implementation continues with the previously shown functions:
    - processEKF()
    - processComplementary()
    - updateWithAccelerometer()
    - updateWithMagnetometer()
    - publishFilteredData()
    And other utility functions...]

private:
    // ROS handles
    ros::NodeHandle& nh_;
    ros::NodeHandle& private_nh_;

    // Communication
    ros::Subscriber imu_sub_;
    ros::Subscriber mag_sub_;
    ros::Publisher filtered_imu_pub_;
    ros::Publisher filtered_mag_pub_;

    // Parameters
    // EKF helper functions
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

    void processEKF(const sensor_msgs::Imu::ConstPtr& msg) {
        // Extract IMU data
        Eigen::Vector3d gyro(msg->angular_velocity.x,
                            msg->angular_velocity.y,
                            msg->angular_velocity.z);

        Eigen::Vector3d accel(msg->linear_acceleration.x,
                             msg->linear_acceleration.y,
                             msg->linear_acceleration.z);

        // Apply gyro bias correction
        gyro -= state_.gyro_bias;

        // 1. State Prediction
        Eigen::Matrix4d Omega = getOmegaMatrix(gyro);
        state_.orientation += (Omega * state_.orientation) * dt_ / 2.0;
        state_.orientation.normalize();

        // 2. Covariance Prediction
        Eigen::MatrixXd F = getStateTransitionMatrix(gyro);
        P_ = F * P_ * F.transpose() + Q_ * dt_;

        // 3. Measurement Updates
        updateWithAccelerometer(accel);

        if (use_mag_ && has_mag_data_) {
            Eigen::Vector3d mag(latest_mag_data_.magnetic_field.x,
                              latest_mag_data_.magnetic_field.y,
                              latest_mag_data_.magnetic_field.z);
            updateWithMagnetometer(mag);
        }
    }

    void processComplementary(const sensor_msgs::Imu::ConstPtr& msg) {
        double gx = msg->angular_velocity.x;
        double gy = msg->angular_velocity.y;
        double gz = msg->angular_velocity.z;

        double ax = msg->linear_acceleration.x;
        double ay = msg->linear_acceleration.y;
        double az = msg->linear_acceleration.z;

        // Get current Euler angles
        double roll, pitch, yaw;
        tf2::Matrix3x3(q_comp_).getRPY(roll, pitch, yaw);

        // Calculate accelerometer-based attitude
        double roll_acc = atan2(ay, az);
        double pitch_acc = atan2(-ax, sqrt(ay*ay + az*az));

        // Update angles using gyroscope data
        roll += gx * dt_;
        pitch += gy * dt_;
        yaw += gz * dt_;

        // Complementary filter fusion
        roll = alpha_ * roll + (1.0 - alpha_) * roll_acc;
        pitch = alpha_ * pitch + (1.0 - alpha_) * pitch_acc;

        // Update quaternion
        q_comp_.setRPY(roll, pitch, yaw);
        q_comp_.normalize();
    }

    void updateWithAccelerometer(const Eigen::Vector3d& accel) {
        // Normalize acceleration
        Eigen::Vector3d accel_norm = accel.normalized();

        // Predict gravity direction
        Eigen::Vector3d gravity_pred = quatToRotMat(state_.orientation) *
                                     Eigen::Vector3d(0, 0, 1);

        // Calculate innovation
        Eigen::Vector3d innovation = accel_norm - gravity_pred;

        // Get measurement Jacobian
        Eigen::MatrixXd H = getAccelMeasurementJacobian();

        // Calculate Kalman gain
        Eigen::MatrixXd S = H * P_ * H.transpose() + R_accel_;
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

        // Update state and covariance
        Eigen::VectorXd delta_x = K * innovation;
        updateState(delta_x);
        P_ = (Eigen::MatrixXd::Identity(7, 7) - K * H) * P_;
    }

    void updateWithMagnetometer(const Eigen::Vector3d& mag) {
        // Normalize magnetic field
        Eigen::Vector3d mag_norm = mag.normalized();

        // Predict magnetic field direction
        Eigen::Vector3d mag_pred = quatToRotMat(state_.orientation) * mag_reference_;

        // Calculate innovation
        Eigen::Vector3d innovation = mag_norm - mag_pred;

        // Get measurement Jacobian
        Eigen::MatrixXd H = getMagMeasurementJacobian();

        // Calculate Kalman gain
        Eigen::MatrixXd S = H * P_ * H.transpose() + R_mag_;
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

        // Update state and covariance
        Eigen::VectorXd delta_x = K * innovation;
        updateState(delta_x);
        P_ = (Eigen::MatrixXd::Identity(7, 7) - K * H) * P_;
    }

    void publishFilteredData() {
        sensor_msgs::Imu filtered_msg;
        filtered_msg.header.stamp = ros::Time::now();
        filtered_msg.header.frame_id = fixed_frame_;

        // Set orientation data based on filter type
        if (filter_type_ == "EKF") {
            filtered_msg.orientation.w = state_.orientation(0);
            filtered_msg.orientation.x = state_.orientation(1);
            filtered_msg.orientation.y = state_.orientation(2);
            filtered_msg.orientation.z = state_.orientation(3);

            // Set covariance
            Eigen::Matrix3d orientation_cov = P_.block<3,3>(0,0);
            for (int i = 0; i < 9; i++) {
                filtered_msg.orientation_covariance[i] = orientation_cov(i/3, i%3);
            }
        } else {
            filtered_msg.orientation.w = q_comp_.w();
            filtered_msg.orientation.x = q_comp_.x();
            filtered_msg.orientation.y = q_comp_.y();
            filtered_msg.orientation.z = q_comp_.z();

            // Set fixed covariance for complementary filter
            std::fill(filtered_msg.orientation_covariance.begin(),
                     filtered_msg.orientation_covariance.end(),
                     0.01);
        }

        // Copy angular velocity and linear acceleration from input
        // You might want to add additional processing here

        filtered_imu_pub_.publish(filtered_msg);
    }

    void magCallback(const sensor_msgs::MagneticField::ConstPtr& msg) {
        if (!use_mag_) return;

        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_mag_data_ = *msg;
        has_mag_data_ = true;
    }

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

        stat.add("Filter type", filter_type_);
        stat.add("Using magnetometer", use_mag_);
        stat.add("Data age (s)", data_age);
        stat.add("dt (s)", dt_);

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

    // Utility functions
    Eigen::Matrix3d quatToRotMat(const Eigen::Vector4d& q) {
        double q0 = q(0), q1 = q(1), q2 = q(2), q3 = q(3);
        Eigen::Matrix3d R;

        R << 1-2*(q2*q2+q3*q3),   2*(q1*q2-q0*q3),   2*(q1*q3+q0*q2),
             2*(q1*q2+q0*q3), 1-2*(q1*q1+q3*q3),   2*(q2*q3-q0*q1),
             2*(q1*q3-q0*q2),   2*(q2*q3+q0*q1), 1-2*(q1*q1+q2*q2);

        return R;
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
        // Update orientation
        Eigen::Vector3d delta_theta = delta_x.segment<3>(0);
        Eigen::Vector4d delta_q;
        delta_q << 1, delta_theta.x()/2, delta_theta.y()/2, delta_theta.z()/2;
        state_.orientation = quaternionMultiply(state_.orientation, delta_q);
        state_.orientation.normalize();

        // Update gyro bias
        state_.gyro_bias += delta_x.segment<3>(4);
    }

private:
    // Node handles and parameters
    ros::NodeHandle& nh_;
    ros::NodeHandle& private_nh_;
    std::string input_topic_;
    std::string output_topic_;
    std::string fixed_frame_;
    int subscriber_queue_size_;
    int num_threads_;

    // Filter parameters
    std::string filter_type_;
    bool use_mag_;
    double publish_freq_;
    double dt_;
    double alpha_;  // Complementary filter parameter
    double max_acceleration_;
    double max_angular_vel_;

    // EKF parameters
    double process_noise_gyro_;
    double process_noise_accel_;
    double process_noise_bias_;
    double measurement_noise_accel_;
    double measurement_noise_mag_;
    double static_threshold_;
    size_t static_samples_;

    // Initial state parameters
    double initial_roll_;
    double initial_pitch_;
    double initial_yaw_;
    Eigen::Vector3d initial_gyro_bias_;

    // Filter state and matrices
    FilterState state_;
    Eigen::MatrixXd P_;    // State covariance
    Eigen::MatrixXd Q_;    // Process noise
    Eigen::Matrix3d R_accel_;  // Accelerometer measurement noise
    Eigen::Matrix3d R_mag_;    // Magnetometer measurement noise
    Eigen::Vector3d mag_reference_;
    tf2::Quaternion q_comp_;  // Complementary filter quaternion

    // Runtime state
    std::mutex data_mutex_;
    bool initialized_;
    bool has_mag_data_;
    ros::Time last_imu_time_;
    sensor_msgs::MagneticField latest_mag_data_;

    // Diagnostics
    diagnostic_updater::Updater diagnostic_updater_;
    std::shared_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> freq_diagnostic_;
    dynamic_reconfigure::Server<amr_imu_filters::ImuFilterConfig> config_server_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "advanced_imu_filter");

    try {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");

        ROS_INFO("Starting IMU filter node...");

        // Create filter instance
        ImuFilter filter(nh, private_nh);

        // Use async spinner with multiple threads
        ros::AsyncSpinner spinner(2);
        spinner.start();

        ROS_INFO("IMU filter node is running.");

        // Wait for shutdown
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
