/* 
This file is part of FAST-LIVO2-PCA

Developer: Tieniu Wang <szuwtn@gmail.com>

For commercial use, please contact me at <1650011724@qq.com> 

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef VOXEL_TYPES_H
#define VOXEL_TYPES_H

#include "common_lib.h"
#include <Eigen/Dense>


#define VOXELMAP_HASH_P 116101
#define VOXELMAP_MAX_N 10000000000

// ==================== 体素位置与哈希 ====================
class VOXEL_LOCATION
{
public:
  int64_t x, y, z;

  VOXEL_LOCATION(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0) : x(vx), y(vy), z(vz) {}

  bool operator==(const VOXEL_LOCATION &other) const { return (x == other.x && y == other.y && z == other.z); }
};

// Hash value
namespace std
{
template <> struct hash<VOXEL_LOCATION>
{
  int64_t operator()(const VOXEL_LOCATION &s) const
  {
    using std::hash;
    using std::size_t;
    return ((((s.z) * VOXELMAP_HASH_P) % VOXELMAP_MAX_N + (s.y)) * VOXELMAP_HASH_P) % VOXELMAP_MAX_N + (s.x);
  }
};
} // namespace std

// ==================== 体素地图配置 ====================
typedef struct VoxelMapConfig
{
  double max_voxel_size_;
  int max_layer_;
  int max_iterations_;
  std::vector<int> layer_init_num_;
  int max_points_num_;
  double planner_threshold_;
  double beam_err_;
  double dept_err_;
  double sigma_num_;
  bool is_pub_plane_map_;
  bool is_pub_pca_en_;    // 是否发布PCA特征向量

  // config of local map sliding
  double sliding_thresh;
  bool map_sliding_en;
  int half_map_size;
} VoxelMapConfig;

// ==================== 点-面匹配结构 ====================
typedef struct PointToPlane
{
  Eigen::Vector3d point_b_; 
  Eigen::Vector3d point_w_;
  Eigen::Vector3d normal_; // 关联平面的法向量，定义在世界系下
  Eigen::Vector3d center_; // 关联平面的中心点，定义在世界系下
  Eigen::Matrix<double, 6, 6> plane_var_; // 平面的6×6不确定性矩阵
  M3D body_cov_;           // 点的本体测量协方差矩阵
  int layer_;
  double d_;               // 平面方程的常数项
  double eigen_value_;
  bool is_valid_;
  float dis_to_plane_;    // 点到平面的距离残差
} PointToPlane;

// ==================== 体素平面特征 ====================
typedef struct VoxelPlane
{
  Eigen::Vector3d center_;    // 面中心点世界系位置
  Eigen::Vector3d normal_;    // 最小特征值的方向（法向量）
  Eigen::Vector3d y_normal_;  // 中间特征值的方向
  Eigen::Vector3d x_normal_;  // 最大特征值的方向
  Eigen::Matrix3d covariance_;// 点云的协方差矩阵 A = Σ(p_i - q)(p_i - q)^T / N
  Eigen::Matrix<double, 6, 6> plane_var_; // 平面参数 (n, q) 的联合协方差矩阵 Σ_{n,q}，融合了传感器的测量噪声和位姿估计误差 // 决定一个平面需要6维变量（法向量3维方向，中心点3维位置）
  float radius_ = 0; // 最大特征值的开方，// 平面有效半径 (sqrt(λ_max))，定义了点云的实际分布范围 // 可以表征平面残差
  float min_eigen_value_ = 1; // 最小特征值(拟合质量，越小越像平面)
  float mid_eigen_value_ = 1; // 中间特征值
  float max_eigen_value_ = 1; // 最大特征值（主方向上的分布跨度）
  float d_ = 0;               // 平面方程的常数项: normal·x + d = 0 或 Ax + By + Cz + d = 0
  int points_size_ = 0;       // 参与拟合平面的点的个数
  bool is_plane_ = false;
  bool is_init_ = false;
  int id_ = 0;                // 全局唯一的平面ID (用于可视化或追踪)
  bool is_update_ = false;
  VoxelPlane()
  {
    plane_var_ = Eigen::Matrix<double, 6, 6>::Zero();
    covariance_ = Eigen::Matrix3d::Zero();
    center_ = Eigen::Vector3d::Zero();
    normal_ = Eigen::Vector3d::Zero();
  }
} VoxelPlane;

#endif // VOXEL_TYPES_H