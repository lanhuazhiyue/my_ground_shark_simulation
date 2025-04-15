#include "my_new_robot_simulation_pkg/STM32_message.h"

namespace STM32_message_namespace
{
    STM32_message::STM32_message() : tf_listener_(tf_buffer_)
    {
        // ROS_INFO("STM32_message::STM32_message()");
    }

    STM32_message::~STM32_message()
    {
        // ROS_INFO("STM32_message::~STM32_message()");
    }
    void STM32_message::init(ros::NodeHandle &nh)
    {
        nh_ = nh;
        // 获取参数
        nh_.param<std::string>("model_name", Model_name_, "ground_shark");
        nh_.param<double>("x_position_init", x_position_init_, 0.0);
        nh_.param<double>("y_position_init", y_position_init_, 0.0);

        // 实例化订阅者
        sub_ = nh_.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 100, &STM32_message::model_states_callback, this);

        // 实例化发布者
        pub_ = nh_.advertise<nav_msgs::Odometry>("/stm32_message/odom", 10);

        odom_.header.stamp = ros::Time::now();

        // 发布 world 坐标系到 odom 坐标系的变换
        geometry_msgs::TransformStamped static_transform;
        static_transform.header.stamp = ros::Time::now();
        static_transform.header.frame_id = "world";
        static_transform.child_frame_id = "odom";
        static_transform.transform.translation.x = x_position_init_;
        static_transform.transform.translation.y = y_position_init_;
        static_transform.transform.translation.z = 0.0;
        static_transform.transform.rotation.x = 0.0;
        static_transform.transform.rotation.y = 0.0;
        static_transform.transform.rotation.z = 0.0;
        static_transform.transform.rotation.w = 1.0;
        static_broadcaster_.sendTransform(static_transform);
        // // 发布 base_footprint 坐标系到 laser_link 坐标系的变换
        // while (true)
        // {
        //     try
        //     {
        //         // 订阅 base_Link 到 lidar_link 的变换
        //         static_transform = tf_buffer_.lookupTransform("Lader_Link", "Base_Link", ros::Time(0)); // ros::Time(0)表示最新可用时间，避免了外推错误
        //         std::cout << "Found transform from Base_Link to Lader_Link" << std::endl;
        //         // 转发变换
        //         static_transform.header.frame_id = "base_footprint";
        //         static_transform.child_frame_id = "laser_link";
        //         static_broadcaster_.sendTransform(static_transform);
        //         std::cout << "Send transform from base_footprint to laser_link" << std::endl;
        //         break;
        //     }
        //     catch (tf2::TransformException &ex)
        //     {
        //         std::cout << "Not found transform from Base_Link to Lader_Link" << std::endl;
        //         std::cout << ex.what() << std::endl;
        //         // ROS_WARN("%s", ex.what());
        //     }
        // }

        // 初始化 odom_
        odom_.pose.pose.position.x = 0;
        odom_.pose.pose.position.y = 0;
    }
    /***********
     * 回调函数
     * 订阅 /gazebo/model_states 话题，获取机器人模型相对于世界坐标系的位置和姿态
     * 转换坐标系并积分更新 nav_msgs::Odometry odom_
     */
    void STM32_message::model_states_callback(const gazebo_msgs::ModelStatesConstPtr &model_state)
    {
        ros::Time current_odom_time = ros::Time::now();
        // 寻找模型名称为 Model_name_ 的模型
        for (int i = 0; i < model_state->name.size(); ++i)
        {
            if (model_state->name[i] == Model_name_)
            {
                // 获取vx,vy,wz,q0,q1,q2,q3
                odom_.child_frame_id = "base_link";
                odom_.header.frame_id = "odom";
                ros::Time last_odom_time = odom_.header.stamp;
                odom_.header.stamp = current_odom_time;
                // world 坐标系下积分得到 position
                odom_.pose.pose.position.x += (model_state->twist[i].linear.x * (current_odom_time - last_odom_time).toSec());
                odom_.pose.pose.position.y += (model_state->twist[i].linear.y * (current_odom_time - last_odom_time).toSec());
                odom_.pose.pose.position.z = 0;
                // 获取姿态
                odom_.pose.pose.orientation = model_state->pose[i].orientation;

                // 更新 odom 到 base_link 的变换
                // 使用估计的 position 和 orientation 作为 odom 到 base_link 的变换
                tf_odom_to_base.header.frame_id = "odom";
                tf_odom_to_base.header.stamp = current_odom_time;
                tf_odom_to_base.child_frame_id = "base_footprint"; // base_link 和 base_footprint 是重合的
                tf_odom_to_base.transform.translation.x = odom_.pose.pose.position.x;
                tf_odom_to_base.transform.translation.y = odom_.pose.pose.position.y;
                tf_odom_to_base.transform.translation.z = odom_.pose.pose.position.z;
                tf_odom_to_base.transform.rotation = odom_.pose.pose.orientation;

                // world 坐标系与 Odom_True 坐标系是重合的，将 world 坐标系下的速度转换到 Base_Link 坐标系下
                // tf2.doTransform不支持 geometry_msgs::TwistStamped ，只能拆成geometry_msgs::Vector3Stamped
                geometry_msgs::Vector3Stamped linear_world, linear_body;
                geometry_msgs::Vector3Stamped angular_world, angular_body;
                linear_world.header.frame_id = "Odom_True";
                linear_world.header.stamp = current_odom_time;
                linear_world.vector = model_state->twist[i].linear;
                angular_world.header.frame_id = "Odom_True";
                angular_world.header.stamp = current_odom_time;
                angular_world.vector = model_state->twist[i].angular;
                try
                {
                    // 获取 Odom_True 到 Base_Link 的变换
                    tf_world_to_base = tf_buffer_.lookupTransform("Base_Link", "Odom_True", ros::Time(0)); // ros::Time(0)表示最新可用时间，避免了外推错误
                }
                catch (tf2::TransformException &ex)
                {
                    std::cout << ex.what() << std::endl;
                    // ROS_WARN("%s", ex.what());
                    return;
                }
                tf2::doTransform(linear_world, linear_body, tf_world_to_base);
                tf2::doTransform(angular_world, angular_body, tf_world_to_base);
                odom_.twist.twist.linear = linear_body.vector;
                odom_.twist.twist.angular = angular_body.vector;
                odom_.twist.twist.linear.z = 0;
                odom_.twist.twist.angular.x = 0;
                odom_.twist.twist.angular.y = 0;

                return;
            }
        }
    }
    void STM32_message::publishOdom()
    {
        // 发布odom消息
        pub_.publish(odom_);
        // 广播 odom 坐标系到 base_link 坐标系的变换
        tf_broadcaster_.sendTransform(tf_odom_to_base);
    }
}