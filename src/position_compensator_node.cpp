#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_listener.h>

#include "amr_imu_filters/imu_filter_core.hpp"

class PositionCompensatorNode {
   public:
    PositionCompensatorNode() : nh_("~") {
        // 获取ROS参数
        double offset_x, offset_y, offset_z;
        std::string input_topic, output_topic;

        // 获取位置偏移参数
        nh_.param("offset_x", offset_x, 0.0);
        nh_.param("offset_y", offset_y, 0.0);
        nh_.param("offset_z", offset_z, 0.0);

        // 获取话题名称，如果没有设置则使用默认值
        nh_.param<std::string>("input_topic", input_topic,
                               "/imu/data_filtered");
        nh_.param<std::string>("output_topic", output_topic,
                               "/imu/data_compensated");

        // 设置IMU偏移量
        Eigen::Vector3d offset(offset_x, offset_y, offset_z);
        compensator_.setOffsetVector(offset);

        // 设置发布者和订阅者
        sub_ = nh_.subscribe(input_topic, 1,
                             &PositionCompensatorNode::imuCallback, this);
        pub_ = nh_.advertise<sensor_msgs::Imu>(output_topic, 1);

        ROS_INFO_STREAM("Position compensator initialized:");
        ROS_INFO_STREAM("Subscribing to: " << input_topic);
        ROS_INFO_STREAM("Publishing to: " << output_topic);
        ROS_INFO_STREAM("Offset: [" << offset_x << ", " << offset_y << ", "
                                    << offset_z << "]");
    }

   private:
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        sensor_msgs::Imu compensated_msg =
            compensator_.compensatePosition(*msg);
        compensated_msg.header = msg->header;
        pub_.publish(compensated_msg);
    }

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    amr_imu_filters::IMUFilterCore compensator_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "position_compensator_node");
    PositionCompensatorNode node;
    ros::spin();
    return 0;
}