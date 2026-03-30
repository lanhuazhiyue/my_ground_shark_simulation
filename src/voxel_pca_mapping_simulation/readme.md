
# Voxel_PCA_Mapping仿真

## 5. voxel_PCA_mapping_simulation （自定义功能包）

功能： mid360仿真, 无人车仿真，搭载自带imu的mid360雷达，完成定位和建图。

输入：/vel_cmd 。

中间话题：gazebo生成/lidar/imu话题和/scan（livox_ros_driver2/CustomMsg）话题

输出：/odom、/map

### 5.1 Intro

* ...

### 5.2 Show Run

#### 5.2.1 测试仿真插件

``` bash
roslaunch livox_laser_simulation mid360_IMU_platform.launch
```

> **注意**：位于 ~/ws_livox/src/Mid360_imu_sim/launch文件夹下，此ros工作空间已加入系统环境变量，功能包源码放在了另一个仓库[livox_laser_simulation_Mid360](https://github.com/lanhuazhiyue/livox_laser_simulation_Mid360.git)中。

如果 *~/ws_livox/devel/lib/liblivox_laser_simulation.so* 生成成功，应该能看到下面两个话题

```bash
# 打开一个新的终端
rostopic list
# 输出如下 >>>>>
/scan (livox_ros_driver2/CustomMsg)
/livox/imu
```

#### 5.2.2 运行加载Mid360的小车

``` bash
roslaunch livox_laser_simulation mid360_IMU_platform.launch
```
