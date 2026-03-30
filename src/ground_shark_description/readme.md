# 仿真模型

## 1 ground_shark_description（自定义功能包）

功能：建立能被gazebo识别的机器人模型文件,.urdf文件或.xacro文件

输入：机器人描述；

输出：/RobotModel

### 1.1 Intro

* mesh文件夹中的STL文件和urdf文件夹中以**Ground-Shark_description**命名的文件是从soliworks中使用sw_urdf_exporter插件自动导出的
* urdf文件夹里的*Ground-Shark_description_fixed.urdf*是将车轮与电机之间的joint修改为fixed锁死之后的模型文件。
    > urdf不能设置变量和宏和文件引用，所以ros引入了xacro文件
* *Ground-Shark_description_fixed.xacro*是在*Ground-Shark_description_fixed.urdf*的基础上额外添加了camera和imu，并调用了*Ground-Shark_description_fixed.gazebo.xacro*，不调.gazebo.xacro文件的话，模型无法与仿真环境交互，只是有一个空架子。
* *Ground-Shark_description_fixed.gazebo.xacro*里面根据link加载了Gazebo插件，使模型能够与仿真环境进行交互
* 将车轮关节固定的原因是假装模型中的圆柱形状的车轮是麦克纳姆轮，能够带动车身在二维地面上自由移动，不存在转向半径。所以使用的仿真插件是*libgazebo_ros_planar_move.so*而非*libgazebo_ros_diff_drive.so*

### 1.2 Show Run

#### 在Rviz中显示模型

```bash
    roslaunch ground-Shark_description display.launch 
```

1. 其中，display.launch文件首先将 *Ground-Shark_description.urdf*加载到了参数服务器中作为 robot_description  
2. 然后， **joint_state_publisher_gui** 功能包（ROS自带）的joint_state_publisher_gui节点会寻找robot_description指向的urdf中可活动的joint关节显示在UI界面中，用户能够拖动滑杆改变joint的状态，并发布当前状态到/joint_states话题中；
3. 接着， **robot_state_publisher**功能包（ROS自带）的robot_state_publisher节点会订阅/joint_states话题，并广播到/tf话题中；
4. 最后， Rviz 会订阅相关话题，可视化显示；

#### 在Gazebo中显示模型

```bash
    roslaunch ground-Shark_description gazebo.launch 
```

1. 其中，gazebo.launch文件使用 **gazebo_ros** （ROS自带）功能包的 empty_world.launch 脚本加载一个空的世界场景
2. 然后，使用spawn_model节点加载Ground-Shark_description.urdf文件，并将模型命名为 Ground-Shark_description 加载到Gazebo中。

#### 在Gazebo中显示交互模型

```bash
    roslaunch ground-Shark_description gazebo_ros.launch 
```

1. 其中，gazebo_ros.launch文件使用 **gazebo_ros** （ROS自带）功能包的 empty_world.launch 脚本加载一个空的世界场景
2. 然后，使用spawn_model节点加载 Ground-Shark_description_fixed.xacro 文件，并将模型命名为 Ground-Shark_description 加载到Gazebo中。
3. 接着， **robot_state_publisher** （ROS自带）功能包的 robot_state_publisher 节点会读取 my_robot_description 参数的urdf文件，将其中的fixed关节发布静态tf变换，同时订阅 /ground_shark/joint_states 话题（由Gazebo生成）获得动态变换，广播到 /tf 话题；
