**重要提示：my_Gazebo_Models.zip压缩包过大超过1G，没有直接放在此github仓库中**
# My_Ground_Shark_Simulation（工作空间）

## 0 Prepare

### 0.1 编译

```shell
sudo apt-get install ros-noetic-turtlebot3-msgs
```

```shell
cd 工作空间
catkin_make
```

### 0.2 设置gazebo使用独显加速打开

* 安装nvidia驱动

```shell
ubuntu-drivers devices
# 选择recomanded的那个版本
sudo apt install nvidia-driver-535
sudo reboot
nvidia-smi
```

* 设置gazebo永久默认使用GPU

```shell
sudo gedit ~/.bashrc 
###
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
###
source ~/.bashrc
```

如果不希望全局使用，可以在launch文件中单独配置

```xml
<launch>
  <env name="__NV_PRIME_RENDER_OFFLOAD" value="1" />
  <env name="__GLX_VENDOR_LIBRARY_NAME" value="nvidia" />
  <!-- 其他内容保持不变 -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find your_pkg)/worlds/your_world.world"/>
  </include>
</launch>
```

### 0.3 设置gazebo的模型加载路径

解压 my_Gazebo_Models 压缩包内的内容到 ~/.gazebo/models/ 路径下，没有models文件夹可以自己新建一个，其默认为隐藏文件夹可以 ctrl+H 显示隐藏。同时删除其中的.git文件夹。

### 0.4 Gazebo 进程崩溃

Gazebo容易出现接口占用的情况，其实是因为后台存在上一次结束的gzserver崩溃但没有终止，此时只需要强制杀死进程即可

```shell
killall -9 gzserver gzclient
```

## 1. ground_shark_description（自定义功能包）

功能：建立能被gazebo识别的机器人模型文件,.urdf文件或.xacro文件

输入：机器人描述；

输出：/RobotModel

详细参考[仿真模型](./src/ground_shark_description/readme.md)

## 2. my_new_robot_simulation_pkg（自定义功能包）

功能：gazebo世界的小车发出传感器测量数据，并等待接收速度指令后执行运动。

输入：/cmd_vel

输出：/stm32_message/odom、 /rplidarA1M8/LaserScan

详细参考[仿真系统](./src/my_new_robot_simulation_pkg/readme.md)

## 3. fastlio2_2d_simulation （自定义功能包）(开发中)

功能： fast-lio2的2D平面版本

输入：/stm32_message/odom、 /rplidarA1M8/LaserScan

输出：/map、 /pose
/lidar/imu
开发中。。。。。。

## 4. ego_simulation_gazebo （自定义功能包）(开发中)

功能： 无人机仿真，fast-lio2输入定位和点云，ego-planner进行建图导航。
开发中。。。。。。

## 5. voxel_PCA_mapping_simulation （自定义功能包）

功能： mid360仿真, 无人车仿真，搭载自带imu的mid360雷达，完成定位和建图。

输入：/vel_cmd 。

中间话题：gazebo生成/lidar/imu话题和/scan（livox_ros_driver2/CustomMsg）话题

输出：/odom、/map

详细参考[Voxel_PCA_Mapping仿真](./src/FAST-LIVO2-PCA/readme.md)

## 6. ground_shark_mid360 （自定义功能包）
与ground_shark_description类似，是soliworks的urdf插件自动生成的一个机器人模型功能包，但作了改动，同时添加了gazebo仿真功能。

详细参考[带有mid360雷达的无人车模型](./src/ground_shark_mid360/readme.md)
