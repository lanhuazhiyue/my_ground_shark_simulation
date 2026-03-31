// 本程序用于将livox_ros_driver2发布的CustomMsg转换为sensor_msgs/PointCloud2
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher pc2_pub;

void customMsgCallback(const livox_ros_driver2::CustomMsg::ConstPtr& msg) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  cloud->header.frame_id = msg->header.frame_id;
  cloud->height = 1;
  cloud->width = msg->points.size();
  cloud->is_dense = true;

  for (const auto& p : msg->points) {
    pcl::PointXYZI pt;
    pt.x = p.x;
    pt.y = p.y;
    pt.z = p.z;
    pt.intensity = p.reflectivity;
    cloud->push_back(pt);
  }

  sensor_msgs::PointCloud2 pc2_msg;
  pcl::toROSMsg(*cloud, pc2_msg);
  pc2_pub.publish(pc2_msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "custom2pc2");
  ros::NodeHandle nh;
  pc2_pub = nh.advertise<sensor_msgs::PointCloud2>("/livox/pointcloud2", 10);
  ros::Subscriber sub = nh.subscribe("/livox/lidar", 10, customMsgCallback);
  ROS_INFO("custom2pc2 node is running...");
  ros::spin();
  return 0;
}
