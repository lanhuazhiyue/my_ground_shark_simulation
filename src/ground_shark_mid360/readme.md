# 带有mid360雷达的无人车模型

本工作空间是由solidworks的urdf插件自动生成的

对urdf文件夹进行了以下改动：
    
1. 为了使用麦克纳姆轮的gazebo仿真插件，修改车轮关节为固定关节，建立了新的 *ground_shark_mid360_fixed.urdf* ，为imu_link添加了惯性，在无人车头部新增了一个相机模型,注释了lidar和imu的碰撞属性避免遮挡测量
2. 新增了*mid360_gazebo.xacro*文件，是mid360雷达的gazebo插件，用于仿真雷达数据，依赖于功能包[livox_laser_simulation_Mid360
](https://github.com/lanhuazhiyue/livox_laser_simulation_Mid360.git),使用前请先在 livox_SDK2 的工作空间编译 livox_laser_simulation_Mid360 功能包并确保已添加到系统环境变量
1. 新增了*default_link_gazebo.xacro*文件，是一些基础的宏定义，用于gazebo仿真
2. 新增了*my_ground_robot_mid360.xacro*文件，是将仿真插件和模型文件结合在一起了，以便在gazebo中加载。
3. 新增了*my_ground_robot_mid360_gazebo.launch*文件，用于启动gazebo仿真，并加载模型文件
4. 新增了*custom2pc2.cpp*文件，用于将 livox_ros_driver2 发布的 CustomMsg 转换为 sensor_msgs/PointCloud2 ，以便在rviz中显示
   
## 使用方法
### 测试仿真插件

``` bash
roslaunch livox_laser_simulation mid360_IMU_platform.launch
```

> **注意**：位于 *[Mid360_imu_sim](~/ws_livox/src/Mid360_imu_sim/launch)*文件夹下，此ros工作空间已加入系统环境变量，功能包源码放在了另一个仓库[livox_laser_simulation_Mid360](https://github.com/lanhuazhiyue/livox_laser_simulation_Mid360.git)中。

如果 *~/ws_livox/devel/lib/liblivox_laser_simulation.so* 生成成功，应该能看到下面两个话题
**注意：** 官方仿真库默认生成的点云是sensor_msgs/PointCloud2类型，本仓库生成的是实物的livox_ros_driver2/CustomMsg类型，能够被fast-lio2和fast-livo2直接使用。

```bash
# 打开一个新的终端
rostopic list
# 输出如下 >>>>>
/scan (livox_ros_driver2/CustomMsg)
/livox/imu
```
### 下载编译此功能包

```shell
cd ~/my_ground_shark_simulation/src
git clone https://github.com/lanhuazhiyue/ground_shark_mid360.git
cd ~/my_ground_shark_simulation
catkin_make
```

### 运行加载Mid360的小车

**温馨提示：**先开一个独立的终端运行roscore再运行gazebo启动launch，可以避免需要频繁重启gazebo。

```shell
cd ~/my_ground_shark_simulation
source devel/setup.bash
roslaunch ground_shark_mid360 my_ground_robot_mid360_gazebo.launch
```
成功会有如下话题输出：
/livox/imu
/livox/lidar

### 点云类型转换
由于rviz里面无法显示livox_ros_driver2/CustomMsg类型的话题，所以需要使用custom2pc2.cpp转换成sensor_msgs/PointCloud2类型的话题，然后才能在rviz中显示
```shell
cd ~/my_ground_shark_simulation
source devel/setup.bash
rosrun ground_shark_mid360 custom2pc2
```
在rviz中添加PointCloud2类型的显示，就可以看到点云数据了。
