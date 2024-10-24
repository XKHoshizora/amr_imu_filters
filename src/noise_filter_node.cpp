#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "amr_imu_filters/imu_filter_core.hpp"

class NoiseFilterNode {
   public:
    NoiseFilterNode() : nh_("~") {
        // 获取ROS参数
        double process_noise, measurement_noise;
        int window_size;
        std::string input_topic, output_topic;

        // 获取滤波器参数
        nh_.param("process_noise", process_noise, 0.01);
        nh_.param("measurement_noise", measurement_noise, 0.1);
        nh_.param("window_size", window_size, 10);

        // 获取话题名称，如果没有设置则使用默认值
        nh_.param<std::string>("input_topic", input_topic, "/imu/data_raw");
        nh_.param<std::string>("output_topic", output_topic,
                               "/imu/data_filtered");

        // 配置过滤器
        filter_.setMovingAverageWindowSize(window_size);
        filter_.setKalmanNoiseParams(process_noise, measurement_noise);

        // 设置发布者和订阅者
        sub_ =
            nh_.subscribe(input_topic, 1, &NoiseFilterNode::imuCallback, this);
        pub_ = nh_.advertise<sensor_msgs::Imu>(output_topic, 1);

        ROS_INFO_STREAM("Noise filter initialized:");
        ROS_INFO_STREAM("Subscribing to: " << input_topic);
        ROS_INFO_STREAM("Publishing to: " << output_topic);
    }

   private:
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        sensor_msgs::Imu filtered_msg = filter_.filterNoise(*msg);
        filtered_msg.header = msg->header;
        pub_.publish(filtered_msg);
    }

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    amr_imu_filters::IMUFilterCore filter_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "noise_filter_node");
    NoiseFilterNode node;
    ros::spin();
    return 0;
}