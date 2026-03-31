# 带有mid360雷达的无人车模型

本工作空间是由solidworks的urdf插件自动生成的
arfwa e 
对urdf文件夹进行了以下改动：
    
1. 为了使用麦克纳姆轮的gazebo仿真插件，修改车轮关节为固定关节，建立了新的 *ground_shark_mid360_fixed.urdf* ，为imu_link添加了惯性，在无人车头部新增了一个相机模型,注释了lidar和imu的碰撞属性避免遮挡测量
2. 新增了*mid360_gazebo.xacro*文件，是mid360雷达的gazebo插件，用于仿真雷达数据，依赖于功能包[livox_laser_simulation_Mid360
](https://github.com/lanhuazhiyue/livox_laser_simulation_Mid360.git),使用前请先在 livox_SDK2 的工作空间编译 livox_laser_simulation_Mid360 功能包并确保已添加到系统环境变量
3. 新增了*default_link_gazebo.xacro*文件，是一些基础的宏定义，用于gazebo仿真
4. 新增了*my_ground_robot_mid360.xacro*文件，是将仿真插件和模型文件结合在一起了，以便在gazebo中加载。
5. 新增了*my_ground_robot_mid360_gazebo.launch*文件，用于启动gazebo仿真，并加载模型文件
6. 新增了*custom2pc2.cpp*文件，用于将livox_ros_driver2发布的CustomMsg转换为sensor_msgs/PointCloud2，以便在rviz中显示
