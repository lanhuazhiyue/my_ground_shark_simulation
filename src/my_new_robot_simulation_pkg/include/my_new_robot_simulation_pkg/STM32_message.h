#ifndef MY_NEW_ROBOT_SIMULATION_PKG_STM32_MESSAGE_H_
#define MY_NEW_ROBOT_SIMULATION_PKG_STM32_MESSAGE_H_

// rosmsg show gazebo_msgs/ModelState

// string model_name
// geometry_msgs/Pose pose
//   geometry_msgs/Point position
//     float64 x
//     float64 y
//     float64 z
//   geometry_msgs/Quaternion orientation
//     float64 x
//     float64 y
//     float64 z
//     float64 w
// geometry_msgs/Twist twist
//   geometry_msgs/Vector3 linear
//     float64 x
//     float64 y
//     float64 z
//   geometry_msgs/Vector3 angular
//     float64 x
//     float64 y
//     float64 z
// string reference_frame

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std_msgs;

namespace STM32_message_namespace
{
    class STM32_message{
    public:
        
        STM32_message();
        ~STM32_message();
        
        void init(ros::NodeHandle &nh); //初始化函数
        void publishOdom();
        
    private:
        ros::NodeHandle nh_;
        ros::Subscriber sub_;
        ros::Publisher pub_;
        std::string Model_name_="ground_shark"; //Gazebo生成的模型状态所使用的模型名称
        nav_msgs::Odometry odom_; //发布消息
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        tf2_ros::TransformBroadcaster tf_broadcaster_;
        tf2_ros::StaticTransformBroadcaster static_broadcaster_;
        geometry_msgs::TransformStamped tf_world_to_base;
        geometry_msgs::TransformStamped tf_odom_to_base;
        geometry_msgs::TwistStamped twist_body_;
        double x_position_init_;
        double y_position_init_;

        void model_states_callback(const gazebo_msgs::ModelStatesConstPtr &model_state); //回调函数
    };
}


#endif //MY_NEW_ROBOT_SIMULATION_PKG_STM32_MESSAGE_H_