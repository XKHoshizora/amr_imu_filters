#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <queue>
#include <deque>
#include <string>

class ImuFilter {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber imu_sub_;
    ros::Publisher imu_pub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // 参数
    std::string input_topic_;
    std::string output_topic_;
    std::string base_frame_id_;
    std::string imu_frame_id_;

    // 滑动窗口
    int window_size_;
    std::deque<geometry_msgs::Vector3> acc_window_;
    std::deque<geometry_msgs::Vector3> gyro_window_;

    // 重力加速度和零偏
    double gravity_magnitude_;
    geometry_msgs::Vector3 gyro_bias_;

    // 初始化参数
    bool initialized_;
    int init_count_;
    int init_samples_;

    // IMU数据累积
    geometry_msgs::Vector3 acc_sum_;
    geometry_msgs::Vector3 gyro_sum_;

    // 滤波参数
    double acc_lpf_alpha_;
    double gyro_lpf_alpha_;

    // 上一次的滤波值
    geometry_msgs::Vector3 last_filtered_acc_;
    geometry_msgs::Vector3 last_filtered_gyro_;

    // 协方差矩阵
    double orientation_stddev_;
    double angular_velocity_stddev_;
    double linear_acceleration_stddev_;

public:
    ImuFilter()
        : private_nh_("~")
        , initialized_(false)
        , init_count_(0)
    {
        loadParameters();
        initializeSubscribers();
        initializePublishers();

        // 初始化累积值
        acc_sum_.x = acc_sum_.y = acc_sum_.z = 0.0;
        gyro_sum_.x = gyro_sum_.y = gyro_sum_.z = 0.0;

        ROS_INFO("IMU Filter initialized with:");
        ROS_INFO("Window size: %d", window_size_);
        ROS_INFO("Init samples: %d", init_samples_);
        ROS_INFO("Input topic: %s", input_topic_.c_str());
        ROS_INFO("Output topic: %s", output_topic_.c_str());
    }

private:
    void loadParameters() {
        // 加载ROS参数
        private_nh_.param("window_size", window_size_, 5);
        private_nh_.param("init_samples", init_samples_, 100);
        private_nh_.param("acc_lpf_alpha", acc_lpf_alpha_, 0.2);
        private_nh_.param("gyro_lpf_alpha", gyro_lpf_alpha_, 0.2);
        private_nh_.param("gravity_magnitude", gravity_magnitude_, 9.80665);
        private_nh_.param("input_topic", input_topic_, std::string("/imu"));
        private_nh_.param("output_topic", output_topic_, std::string("/imu_filtered"));
        private_nh_.param("base_frame_id", base_frame_id_, std::string("base_link"));
        private_nh_.param("imu_frame_id", imu_frame_id_, std::string("imu_link"));
        private_nh_.param("orientation_stddev", orientation_stddev_, 0.01);
        private_nh_.param("angular_velocity_stddev", angular_velocity_stddev_, 0.01);
        private_nh_.param("linear_acceleration_stddev", linear_acceleration_stddev_, 0.01);
    }

    void initializeSubscribers() {
        imu_sub_ = nh_.subscribe(input_topic_, 10, &ImuFilter::imuCallback, this);
    }

    void initializePublishers() {
        imu_pub_ = nh_.advertise<sensor_msgs::Imu>(output_topic_, 10);
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        // 数据有效性检查
        if (!validateImuData(*msg)) {
            return;
        }

        if (!initialized_) {
            initializeFilter(msg);
            return;
        }

        sensor_msgs::Imu filtered_msg = *msg;
        filtered_msg.header.frame_id = imu_frame_id_;

        // 应用低通滤波
        geometry_msgs::Vector3 filtered_acc = lowPassFilter(
            msg->linear_acceleration, last_filtered_acc_, acc_lpf_alpha_);
        geometry_msgs::Vector3 filtered_gyro = lowPassFilter(
            msg->angular_velocity, last_filtered_gyro_, gyro_lpf_alpha_);

        // 更新滑动窗口
        updateWindow(filtered_acc, filtered_gyro);

        // 计算平均值
        geometry_msgs::Vector3 avg_acc = calculateAverage(acc_window_);
        geometry_msgs::Vector3 avg_gyro = calculateAverage(gyro_window_);

        // 规范化重力向量
        normalizeAcceleration(avg_acc);

        // 计算姿态四元数
        tf2::Quaternion orientation = calculateOrientation(avg_acc);

        // 填充过滤后的消息
        fillFilteredMessage(filtered_msg, avg_acc, avg_gyro, orientation);

        // 发布TF转换
        publishTransform(orientation, msg->header.stamp);

        // 存储当前值作为下次滤波的基础
        last_filtered_acc_ = filtered_acc;
        last_filtered_gyro_ = filtered_gyro;

        // 发布过滤后的消息
        imu_pub_.publish(filtered_msg);
    }

    bool validateImuData(const sensor_msgs::Imu& msg) {
        // 检查加速度数据是否有效
        if (std::isnan(msg.linear_acceleration.x) ||
            std::isnan(msg.linear_acceleration.y) ||
            std::isnan(msg.linear_acceleration.z)) {
            ROS_WARN_THROTTLE(1, "Received NaN in IMU acceleration data");
            return false;
        }

        // 检查角速度数据是否有效
        if (std::isnan(msg.angular_velocity.x) ||
            std::isnan(msg.angular_velocity.y) ||
            std::isnan(msg.angular_velocity.z)) {
            ROS_WARN_THROTTLE(1, "Received NaN in IMU angular velocity data");
            return false;
        }

        // 检查加速度范围
        double acc_magnitude = sqrt(
            pow(msg.linear_acceleration.x, 2) +
            pow(msg.linear_acceleration.y, 2) +
            pow(msg.linear_acceleration.z, 2));

        if (acc_magnitude < 1.0 || acc_magnitude > 30.0) {
            ROS_WARN_THROTTLE(1, "Acceleration magnitude out of range: %f", acc_magnitude);
            return false;
        }

        return true;
    }

    void initializeFilter(const sensor_msgs::Imu::ConstPtr& msg) {
        if (init_count_ < init_samples_) {
            // 累积IMU数据
            acc_sum_.x += msg->linear_acceleration.x;
            acc_sum_.y += msg->linear_acceleration.y;
            acc_sum_.z += msg->linear_acceleration.z;

            gyro_sum_.x += msg->angular_velocity.x;
            gyro_sum_.y += msg->angular_velocity.y;
            gyro_sum_.z += msg->angular_velocity.z;

            init_count_++;

            if (init_count_ == init_samples_) {
                // 计算初始平均值
                double inv_samples = 1.0 / init_samples_;
                last_filtered_acc_.x = acc_sum_.x * inv_samples;
                last_filtered_acc_.y = acc_sum_.y * inv_samples;
                last_filtered_acc_.z = acc_sum_.z * inv_samples;

                last_filtered_gyro_.x = gyro_sum_.x * inv_samples;
                last_filtered_gyro_.y = gyro_sum_.y * inv_samples;
                last_filtered_gyro_.z = gyro_sum_.z * inv_samples;

                // 计算陀螺仪零偏
                gyro_bias_ = last_filtered_gyro_;

                initialized_ = true;
                ROS_INFO("IMU Filter initialized with %d samples", init_samples_);
            }
        }
    }

    geometry_msgs::Vector3 lowPassFilter(
        const geometry_msgs::Vector3& current,
        const geometry_msgs::Vector3& last,
        double alpha) {
        geometry_msgs::Vector3 filtered;
        filtered.x = alpha * current.x + (1.0 - alpha) * last.x;
        filtered.y = alpha * current.y + (1.0 - alpha) * last.y;
        filtered.z = alpha * current.z + (1.0 - alpha) * last.z;
        return filtered;
    }

    void updateWindow(
        const geometry_msgs::Vector3& acc,
        const geometry_msgs::Vector3& gyro) {
        acc_window_.push_back(acc);
        gyro_window_.push_back(gyro);

        while (acc_window_.size() > window_size_) {
            acc_window_.pop_front();
        }
        while (gyro_window_.size() > window_size_) {
            gyro_window_.pop_front();
        }
    }

    geometry_msgs::Vector3 calculateAverage(
        const std::deque<geometry_msgs::Vector3>& window) {
        geometry_msgs::Vector3 avg;
        avg.x = avg.y = avg.z = 0.0;

        if (window.empty()) {
            return avg;
        }

        for (const auto& v : window) {
            avg.x += v.x;
            avg.y += v.y;
            avg.z += v.z;
        }

        double inv_size = 1.0 / window.size();
        avg.x *= inv_size;
        avg.y *= inv_size;
        avg.z *= inv_size;

        return avg;
    }

    void normalizeAcceleration(geometry_msgs::Vector3& acc) {
        double acc_magnitude = sqrt(
            acc.x * acc.x +
            acc.y * acc.y +
            acc.z * acc.z);

        if (acc_magnitude > 0.1) {
            double scale = gravity_magnitude_ / acc_magnitude;
            acc.x *= scale;
            acc.y *= scale;
            acc.z *= scale;
        }
    }

    tf2::Quaternion calculateOrientation(const geometry_msgs::Vector3& acc) {
        tf2::Vector3 gravity(acc.x, acc.y, acc.z);
        tf2::Vector3 z_axis(0, 0, 1);

        // 计算旋转轴和角度
        tf2::Vector3 rotation_axis = z_axis.cross(gravity.normalized());
        double rotation_angle = acos(z_axis.dot(gravity.normalized()));

        // 创建姿态四元数
        tf2::Quaternion orientation;
        if (rotation_axis.length() > 0.001) {
            orientation.setRotation(rotation_axis.normalized(), rotation_angle);
        } else {
            orientation.setRPY(0, 0, 0);
        }

        return orientation;
    }

    void fillFilteredMessage(
        sensor_msgs::Imu& msg,
        const geometry_msgs::Vector3& acc,
        const geometry_msgs::Vector3& gyro,
        const tf2::Quaternion& orientation) {

        // 填充加速度和角速度
        msg.linear_acceleration = acc;
        msg.angular_velocity = gyro;

        // 填充方向四元数
        msg.orientation.x = orientation.x();
        msg.orientation.y = orientation.y();
        msg.orientation.z = orientation.z();
        msg.orientation.w = orientation.w();

        // 设置协方差
        for (int i = 0; i < 9; i++) {
            msg.orientation_covariance[i] = 0.0;
            msg.angular_velocity_covariance[i] = 0.0;
            msg.linear_acceleration_covariance[i] = 0.0;
        }
        msg.orientation_covariance[0] =
        msg.orientation_covariance[4] =
        msg.orientation_covariance[8] = orientation_stddev_;

        msg.angular_velocity_covariance[0] =
        msg.angular_velocity_covariance[4] =
        msg.angular_velocity_covariance[8] = angular_velocity_stddev_;

        msg.linear_acceleration_covariance[0] =
        msg.linear_acceleration_covariance[4] =
        msg.linear_acceleration_covariance[8] = linear_acceleration_stddev_;
    }

    void publishTransform(
        const tf2::Quaternion& orientation,
        const ros::Time& timestamp) {
        geometry_msgs::TransformStamped transform;

        transform.header.stamp = timestamp;
        transform.header.frame_id = base_frame_id_;
        transform.child_frame_id = imu_frame_id_;

        transform.transform.translation.x = 0.0;
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 0.0;

        transform.transform.rotation.x = orientation.x();
        transform.transform.rotation.y = orientation.y();
        transform.transform.rotation.z = orientation.z();
        transform.transform.rotation.w = orientation.w();

        tf_broadcaster_.sendTransform(transform);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_filter");
    ImuFilter imu_filter;
    ros::spin();
    return 0;
}