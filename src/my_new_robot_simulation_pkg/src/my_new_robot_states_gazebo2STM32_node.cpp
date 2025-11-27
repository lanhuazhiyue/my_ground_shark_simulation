

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "my_new_robot_simulation_pkg/STM32_message.h"


int main(int argc, char **argv)
{
    //设置编码
    setlocale(LC_ALL,"");

    ros::init(argc, argv, "my_new_robot_states_gazebo2STM32");
    ros::NodeHandle nh;
    ros::Duration(10.0).sleep(); // 停10s等Gazebo加载完成

    STM32_message_namespace::STM32_message STM32_msg;

    STM32_msg.init(nh); // 初始化

    ros::Rate loop_rate(100); // 100 Hz
    while (ros::ok())
    {
        STM32_msg.publishOdom();
        ros::spinOnce(); // 不阻塞主线程，每次循环处理一次回调，回调如果有10个缓存就执行处理10次
        loop_rate.sleep();
    }


    
    return 0;
}



// #include <ros/ros.h>
// #include <tf2_ros/transform_listener.h>
// #include <geometry_msgs/TransformStamped.h>

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "tf_debug_node");
//     ros::NodeHandle nh;

//     tf2_ros::Buffer tf_buffer;
//     tf2_ros::TransformListener tf_listener(tf_buffer);

//     ros::Rate rate(10);
//     while (ros::ok()) {
//         try {
//             geometry_msgs::TransformStamped transform = 
//                 tf_buffer.lookupTransform("Base_Link", "odom", ros::Time(0));
//             ROS_INFO("Transform found!");
//         } catch (tf2::TransformException &ex) {
//             ROS_ERROR("%s", ex.what());
//         }
//         rate.sleep();
//     }
//     return 0;
// }