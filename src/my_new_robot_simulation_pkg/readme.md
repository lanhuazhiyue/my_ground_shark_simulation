# 仿真系统

## 2. my_new_robot_simulation_pkg（自定义功能包）

功能：gazebo世界的小车发出传感器测量数据，并等待接收速度指令后执行运动。

输入：/cmd_vel

输出：/stm32_message/odom、 /rplidarA1M8/LaserScan

### 2.1 Intro

* world文件夹里储存了一个Gazebo的世界环境描述文件**my_house_with_table.world** ;
* 其余R2D2名称的文件是测试时用的，无实际意义

### 2.2 Show Run

#### 2.2.1 在Gazebo中加载世界和环境和小车模型，并在Rviz中订阅显示Gazebo发出的相关话题

```bash
    roslaunch my_new_robot_simulation_pkg my_new_robot_simulation_gazebo.launch
```

1. 首先，launch文件首先会运行**gazebo_ros**功能包（ROS自带）中的empty_world.launch加载**my_new_robot_simulation_pkg**功能包（自定义）中的my_house_with_table.world文件加载一个世界场景；
2. 然后，运行 **gazebo_ros** 功能包（ROS自带）中的spawn_model节点加载 Ground-**Shark_description** 功能包（自定义）中的 *Ground-Shark_description_fixed.xacro* 文件，并将小车模型命名为 ground_shark 放置在指定初始位置上；
3. 其中 *Ground-Shark_description_fixed.xacro* 会调用 *Ground-Shark_description_fixed.gazebo.xacro* ，里面使用了各种插件，ROS发布速度指令到 /cmd_vel 话题，Gazebo通过插件将速度指令转换为目标模型（ *libgazebo_ros_planar_move.so* 的 robotNamespace ）的实际运动，运动的坐标系关系设置为了 Odom_True --> Base_Link ， 位姿消息发布在 /Gazebo/Odom_True 话题上，在源码里，运动插件会直接取世界坐标系下的真值作为里程计输出，里程计里的速度是根据位置差分计算出来的。
4. （如果使用*libgazebo_ros_joint_state_publisher.so* 插件会发布模型的joint状态到 /ground_shark/joint_states 话题；）
5. 接着，运行 **robot_state_publisher** 功能包（ROS自带）中的 robot_state_publisher 节点加载 "robot_description" 参数中的fixed关节发布静态变换，同时订阅 /ground_shark/joint_states 话题（由Gazebo生成）获得动态变换，广播到 /tf 话题；
6. 最后， Rviz 可以订阅相关话题可视化显示出来。

#### 2.2.2 获取Gazebo的指定数据模拟STM32端发过来的数据,相当于Gazebo端就是一个STM32小车， gazebo_ros 功能包就是串口

```bash
    roslaunch my_new_robot_simulation_pkg my_new_robot_simulation_STM32.launch
```

1. 首先引用了 **my_new_robot_simulation_gazebo.launch** ,
2. 然后加载 **my_new_robot_simulation_STM32** 节点，模拟串口收发STM32端数据生成 /odom 话题。
3. STM32端的自定义数据格式为：**float[] = {vx,vy,wz,q0,q1,q2,q3,ax,ay,az,gx,gy,gz}**
4. 生成的/odom 话题数据格式为： nav_msgs/Odometry ，其中默认 pz = vz = wx = wy = 0；  
    vx 和 vy 和 wz 是由 Gazebo 的 world(Odom_True) 世界坐标系位置状态真值微分 (Gazebo内部进行的微分处理) 变换到 base_link 体坐标系下的数值,
    px 和 py 是由 vx 和 vy 积分 (积分初值为0) 得到的数值，以模拟STM32串口收发的数据和ROS端处理STM32消息的自定义功能包；
    q0,q1,q2,q3 是直接取的Gazebo的 world(Odom_True) 世界坐标系位置状态真值，模拟STM32端的姿态估计结果；
5. 从tf变换上，相当于复制了一份 Odom_True --> Base_Link 到 odom --> base_link ，这样做的原因是一棵TF树只能有一个根节点且每个子节点仅能有一个父节点，而 SLAM 端会生成一个 map --> odom 的tf变换，所以需要将真实的 Odom_Ture 和模拟STM32的 odom 分开成两棵树。
