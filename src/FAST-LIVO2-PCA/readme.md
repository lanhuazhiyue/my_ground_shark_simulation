# 1. FAST-LIVO2-PCA

## 1.1. [FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry](./README_fast-livo2.md)

按照fastlivo2的教程安装配置其标准环境，如果遇到编译错误可参考以下解决方案：[Sophus编译报错](./Supplementary/sophus报错.png)

关于fastlivo2论文和代码的详细讲解可以参考[论文fastlivo2论文详解]()和[fastlivo2工程详解]()、[fastlivo2代码详解]()。

## 1.2. PCA

- 加入了mid360雷达的配置和启动文件 *./src/FAST-LIVO2-PCA/config/mid360.yaml*。
- 将fast-livo2的地图结构从Voxel+OctoMap改为Voxel+PCA Map，并添加了可视化功能，可以实时显示PCA Map。
- 代码修改逻辑是对于 std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map_; 将体素位置键对应的值从 VoxelOctoTree 节点对象换成新的 VoxelKDTree 节点对象。

### 1.2.1. 仅换用mid360雷达和关闭视觉

```bash
roslaunch fast_livo_pca mapping_mid360.launch
```

### 1.2.2. 可视化观察Voxel+OctoMap
关于 *./src/FAST-LIVO2-PCA/rviz_cfg/fast_livo2.rviz* 对于 *src/FAST-LIVO2-PCA/config/mid360.yaml* 配置文件：
1. 将 dense_map_en 参数设置为 false 即可看见降采样后的每帧点云在 /cloud_registered 话题上,设置Decay后可以看见一段时间积累的效果见图片[降采样点云](./Supplementary/down_sample点云降采样效果图.png)，此图是在静止状态下采集的，图中**红色点**是降采样后的原始点，**绿色点**是成功关联到平面的有效匹配点，可以看出[livox雷达的扫描结构](./Supplementary/livox点云扫描结构.png)决定了0.1m的体素立方体降采样得到的点云是分布在各自的领域内的。构建voxel_map_中plane的数据来源就是这些红色的点云。
2. 将 pub_effect_point_en 参数设置为 true 即可看见成功关联到体素平面的点（用于状态估计的有效点）在 /cloud_effected 话题上。关于从体素降采样的输入点云如何拟合平面的流程可以参考图片[点云去向图](./Supplementary/lidar_point雷达点云去向图.png)。
3. 将 pub_plane_en 参数设置为 true 即可看见以 MarkerArray 形式发布所有体素内部的拟合平面（显示为带颜色的扁平圆柱体）在 /planes 话题上，颜色越红表示不确定性越大。
详细参考图片[publish对象解释](./Supplementary/publish对象.png)
关于输出终端中的变量含义详细参考图片[输出变量解释](./Supplementary/输出变量解释.png)

### 1.2.3. 可视化观察Voxel+PCA Map
配置 pca_mapping_en 参数为 true ,即可屏蔽octomap的使用而改用自建的pcamap 



