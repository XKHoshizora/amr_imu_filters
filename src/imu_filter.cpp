#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Vector3.h>
#include <queue>
#include <deque>

class ImuFilter {
private:
    ros::NodeHandle nh_;
    ros::Subscriber imu_sub_;
    ros::Publisher imu_pub_;

    // 滑动窗口大小
    static const int WINDOW_SIZE = 10;
    std::deque<geometry_msgs::Vector3> acc_window_;
    std::deque<geometry_msgs::Vector3> gyro_window_;

    // 重力加速度大小 (m/s^2)
    const double GRAVITY_MAGNITUDE = 9.80665;

    // 初始化状态
    bool initialized_ = false;
    int init_count_ = 0;
    const int INIT_SAMPLES = 100;  // 初始化所需的样本数

    // IMU数据累积
    geometry_msgs::Vector3 acc_sum_;
    geometry_msgs::Vector3 gyro_sum_;

    // 滤波参数
    double acc_lpf_alpha_ = 0.1;   // 加速度低通滤波系数
    double gyro_lpf_alpha_ = 0.1;  // 角速度低通滤波系数

    // 上一次的滤波值
    geometry_msgs::Vector3 last_filtered_acc_;
    geometry_msgs::Vector3 last_filtered_gyro_;

public:
    ImuFilter() {
        // 订阅原始IMU话题
        imu_sub_ = nh_.subscribe("imu", 10, &ImuFilter::imuCallback, this);
        // 发布过滤后的IMU话题
        imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu_filtered", 10);

        // 初始化累积值
        acc_sum_.x = acc_sum_.y = acc_sum_.z = 0.0;
        gyro_sum_.x = gyro_sum_.y = gyro_sum_.z = 0.0;

        ROS_INFO("IMU Filter node initialized");
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        if (!initialized_) {
            initializeFilter(msg);
            return;
        }

        sensor_msgs::Imu filtered_msg = *msg;

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
        double acc_magnitude = sqrt(avg_acc.x * avg_acc.x +
                                  avg_acc.y * avg_acc.y +
                                  avg_acc.z * avg_acc.z);
        if (acc_magnitude > 0.1) {  // 防止除零
            avg_acc.x *= GRAVITY_MAGNITUDE / acc_magnitude;
            avg_acc.y *= GRAVITY_MAGNITUDE / acc_magnitude;
            avg_acc.z *= GRAVITY_MAGNITUDE / acc_magnitude;
        }

        // 计算姿态四元数
        tf2::Vector3 gravity(avg_acc.x, avg_acc.y, avg_acc.z);
        tf2::Vector3 z_axis(0, 0, 1);

        // 计算旋转轴和角度
        tf2::Vector3 rotation_axis = z_axis.cross(gravity.normalized());
        double rotation_angle = acos(z_axis.dot(gravity.normalized()));

        // 创建姿态四元数
        tf2::Quaternion orientation;
        if (rotation_axis.length() > 0.001) {  // 防止旋转轴过小
            orientation.setRotation(rotation_axis.normalized(), rotation_angle);
        } else {
            orientation.setRPY(0, 0, 0);
        }

        // 填充过滤后的消息
        filtered_msg.linear_acceleration = avg_acc;
        filtered_msg.angular_velocity = avg_gyro;
        filtered_msg.orientation.x = orientation.x();
        filtered_msg.orientation.y = orientation.y();
        filtered_msg.orientation.z = orientation.z();
        filtered_msg.orientation.w = orientation.w();

        // 设置协方差
        for (int i = 0; i < 9; i++) {
            filtered_msg.orientation_covariance[i] = 0.01;
            filtered_msg.angular_velocity_covariance[i] = 0.01;
            filtered_msg.linear_acceleration_covariance[i] = 0.01;
        }

        // 存储当前值作为下次滤波的基础
        last_filtered_acc_ = filtered_acc;
        last_filtered_gyro_ = filtered_gyro;

        // 发布过滤后的消息
        imu_pub_.publish(filtered_msg);
    }

private:
    void initializeFilter(const sensor_msgs::Imu::ConstPtr& msg) {
        if (init_count_ < INIT_SAMPLES) {
            // 累积IMU数据
            acc_sum_.x += msg->linear_acceleration.x;
            acc_sum_.y += msg->linear_acceleration.y;
            acc_sum_.z += msg->linear_acceleration.z;

            gyro_sum_.x += msg->angular_velocity.x;
            gyro_sum_.y += msg->angular_velocity.y;
            gyro_sum_.z += msg->angular_velocity.z;

            init_count_++;

            if (init_count_ == INIT_SAMPLES) {
                // 计算初始平均值
                double inv_samples = 1.0 / INIT_SAMPLES;
                last_filtered_acc_.x = acc_sum_.x * inv_samples;
                last_filtered_acc_.y = acc_sum_.y * inv_samples;
                last_filtered_acc_.z = acc_sum_.z * inv_samples;

                last_filtered_gyro_.x = gyro_sum_.x * inv_samples;
                last_filtered_gyro_.y = gyro_sum_.y * inv_samples;
                last_filtered_gyro_.z = gyro_sum_.z * inv_samples;

                initialized_ = true;
                ROS_INFO("IMU Filter initialized with %d samples", INIT_SAMPLES);
            }
        }
    }

    geometry_msgs::Vector3 lowPassFilter(const geometry_msgs::Vector3& current,
                                       const geometry_msgs::Vector3& last,
                                       double alpha) {
        geometry_msgs::Vector3 filtered;
        filtered.x = alpha * current.x + (1.0 - alpha) * last.x;
        filtered.y = alpha * current.y + (1.0 - alpha) * last.y;
        filtered.z = alpha * current.z + (1.0 - alpha) * last.z;
        return filtered;
    }

    void updateWindow(const geometry_msgs::Vector3& acc,
                     const geometry_msgs::Vector3& gyro) {
        acc_window_.push_back(acc);
        gyro_window_.push_back(gyro);

        if (acc_window_.size() > WINDOW_SIZE) {
            acc_window_.pop_front();
            gyro_window_.pop_front();
        }
    }

    geometry_msgs::Vector3 calculateAverage(const std::deque<geometry_msgs::Vector3>& window) {
        geometry_msgs::Vector3 avg;
        avg.x = avg.y = avg.z = 0.0;

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
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_filter");
    ImuFilter imu_filter;
    ros::spin();
    return 0;
}