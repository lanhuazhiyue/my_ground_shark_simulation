/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "voxel_map.h"

void calcBodyCov(Eigen::Vector3d &pb, const float range_inc, const float degree_inc, Eigen::Matrix3d &cov)
{
  if (pb[2] == 0) pb[2] = 0.0001;
  float range = sqrt(pb[0] * pb[0] + pb[1] * pb[1] + pb[2] * pb[2]); // 点云深度
  float range_var = range_inc * range_inc;
  Eigen::Matrix2d direction_var;
  direction_var << pow(sin(DEG2RAD(degree_inc)), 2), 0, 0, pow(sin(DEG2RAD(degree_inc)), 2);
  Eigen::Vector3d direction(pb); // 点云方向
  direction.normalize();
  Eigen::Matrix3d direction_hat; // 反对称矩阵
  direction_hat << 0, -direction(2), direction(1), direction(2), 0, -direction(0), -direction(1), direction(0), 0;
  //根据角度确定切平面上的一组正交基 N =（N1，N2）
  Eigen::Vector3d base_vector1(1, 1, -(direction(0) + direction(1)) / direction(2));
  base_vector1.normalize();
  Eigen::Vector3d base_vector2 = base_vector1.cross(direction);
  base_vector2.normalize();
  Eigen::Matrix<double, 3, 2> N;
  N << base_vector1(0), base_vector2(0), base_vector1(1), base_vector2(1), base_vector1(2), base_vector2(2);
  Eigen::Matrix<double, 3, 2> A = range * direction_hat * N;
  cov = direction * range_var * direction.transpose() + A * direction_var * A.transpose();
}

void loadVoxelConfig(ros::NodeHandle &nh, VoxelMapConfig &voxel_config)
{
  nh.param<bool>("publish/pub_plane_en", voxel_config.is_pub_plane_map_, false);
  nh.param<bool>("publish/pub_pca_en", voxel_config.is_pub_pca_en_, false);

  nh.param<int>("lio/max_layer", voxel_config.max_layer_, 1);
  nh.param<double>("lio/voxel_size", voxel_config.max_voxel_size_, 0.5);
  nh.param<double>("lio/min_eigen_value", voxel_config.planner_threshold_, 0.01);
  nh.param<double>("lio/sigma_num", voxel_config.sigma_num_, 3);
  nh.param<double>("lio/beam_err", voxel_config.beam_err_, 0.02); // 雷达激光的角度误差，0.02是avia手册给的
  nh.param<double>("lio/dept_err", voxel_config.dept_err_, 0.05); // 雷达激光的深度误差，0.05是avia手册给的  
  nh.param<vector<int>>("lio/layer_init_num", voxel_config.layer_init_num_, vector<int>{5,5,5,5,5});
  nh.param<int>("lio/max_points_num", voxel_config.max_points_num_, 50);
  nh.param<int>("lio/max_iterations", voxel_config.max_iterations_, 5);

  nh.param<bool>("local_map/map_sliding_en", voxel_config.map_sliding_en, false);
  nh.param<int>("local_map/half_map_size", voxel_config.half_map_size, 100);
  nh.param<double>("local_map/sliding_thresh", voxel_config.sliding_thresh, 8);

  ROS_INFO("VOXEL_TREE_NAME: %s", VOXEL_TREE_NAME);
}













void VoxelMapManager::StateEstimation(StatesGroup &state_propagat)
{
  cross_mat_list_.clear();
  cross_mat_list_.reserve(feats_down_size_);
  body_cov_list_.clear(); //存体坐标系下的一堆点
  body_cov_list_.reserve(feats_down_size_);

  // build_residual_time = 0.0;
  // ekf_time = 0.0;
  // double t0 = omp_get_wtime();

  // 与 BuildVoxelMap() 内的点云不确定性计算相比，这里的点云不确定性计算不需要传播到世界坐标系（因为此时暂未更新 state_ ）
  for (size_t i = 0; i < feats_down_body_->size(); i++)
  {
    V3D point_this(feats_down_body_->points[i].x, feats_down_body_->points[i].y, feats_down_body_->points[i].z);
    if (point_this[2] == 0) { point_this[2] = 0.001; }
    M3D var;
    calcBodyCov(point_this, config_setting_.dept_err_, config_setting_.beam_err_, var);
    body_cov_list_.push_back(var);
    point_this = extR_ * point_this + extT_; //从雷达体坐标系转到IMU坐标系
    M3D point_crossmat;
    point_crossmat << SKEW_SYM_MATRX(point_this);
    cross_mat_list_.push_back(point_crossmat);
  }

  vector<pointWithVar>().swap(pv_list_); //存世界坐标系下的一堆点
  pv_list_.resize(feats_down_size_);

  int rematch_num = 0; // 记录满足收敛的次数
  MD(DIM_STATE, DIM_STATE) G, H_T_H, I_STATE; // 待估计状态共19维 = 【6维位姿+3维线速度+6维IMU偏差+3维重力+1维相机曝光时间】
  G.setZero();
  H_T_H.setZero();
  I_STATE.setIdentity();

  bool flg_EKF_inited, flg_EKF_converged, EKF_stop_flg = 0;
  for (int iterCount = 0; iterCount < config_setting_.max_iterations_; iterCount++) //预设的最大迭代次数为5次，迭代的原因是观测函数非线性，目的是找更好的线性化切点state_
  {
    double total_residual = 0.0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr world_lidar(new pcl::PointCloud<pcl::PointXYZI>);
    TransformLidar(state_.rot_end, state_.pos_end, feats_down_body_, world_lidar); // 用每次迭代产生的新 state_ 重新变换到世界系，从而缩小残差
    M3D rot_var = state_.cov.block<3, 3>(0, 0);
    M3D t_var = state_.cov.block<3, 3>(3, 3);
    for (size_t i = 0; i < feats_down_body_->size(); i++)
    {
      pointWithVar &pv = pv_list_[i];
      pv.point_b << feats_down_body_->points[i].x, feats_down_body_->points[i].y, feats_down_body_->points[i].z;
      pv.point_w << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;

      M3D cov = body_cov_list_[i];
      M3D point_crossmat = cross_mat_list_[i];
      //////作者在github的[issue#89]()和VoxelMap的issue#15承认此处是存在bug的，但不能轻易按论文直接修改，参考./Supplementary/var协方差计算公式的Bug.png/////////////////
      cov = state_.rot_end * cov * state_.rot_end.transpose() + (-point_crossmat) * rot_var * (-point_crossmat.transpose()) + t_var;
      ////具体表现为第一项是雷达测量的不确定性传播到世界坐标系——正确，
      ////        第二项是姿态估计的不确定性传播到世界坐标系——错误，少了IMU到世界系的旋转,结果表现为数值偏小
      ////        第三项是位置估计的不确定性传播到世界坐标系——正确////////
      pv.var = cov;
      pv.body_var = body_cov_list_[i];
    }
    // 遍历后 pv_list_ 内是按最新估计 state_ 变换后的点云
    ptpl_list_.clear();

    // double t1 = omp_get_wtime();
    // 对当前的世界系点云进行面匹配，计算残差储存在 ptpl_list_ 中，由于一般不是所有点都能成功匹配到平面， ptpl_list_ 的大小应该略小于 pv_list_ 
    BuildResidualListOMP(pv_list_, ptpl_list_); // 依赖 OpenMP 并行加速

    // build_residual_time += omp_get_wtime() - t1;

    for (int i = 0; i < ptpl_list_.size(); i++)
    {
      total_residual += fabs(ptpl_list_[i].dis_to_plane_);
    }
    effct_feat_num_ = ptpl_list_.size();
    //////////////////////////每次迭代都在终端输出一行信息//////////////////////////////////////////////////////
    cout << "[ LIO ] Raw feature num: " << feats_undistort_->size() << ", downsampled feature num:" << feats_down_size_ 
         << " effective feature num: " << effct_feat_num_ << " average residual: " << total_residual / effct_feat_num_ << endl;

    /*** Computation of Measuremnt Jacobian matrix H and measurents covarience ***/
    MatrixXd Hsub(effct_feat_num_, 6);  // 仅受6维位姿误差影响，视觉更新中会额外加入曝光时间τ，其余偏置误差的更新不会影响测量雅可比矩阵的大小
    MatrixXd Hsub_T_R_inv(6, effct_feat_num_);
    VectorXd R_inv(effct_feat_num_); 
    VectorXd meas_vec(effct_feat_num_);
    meas_vec.setZero();
    for (int i = 0; i < effct_feat_num_; i++)
    {
      auto &ptpl = ptpl_list_[i];
      V3D point_this(ptpl.point_b_);
      point_this = extR_ * point_this + extT_; // 变换到IMU坐标系
      V3D point_body(ptpl.point_b_);
      M3D point_crossmat;
      point_crossmat << SKEW_SYM_MATRX(point_this);

      /*** get the normal vector of closest surface/corner ***/
      ///////计算观测噪音的协方差 R 的逆，仅受测量端不确定性影响，不吸纳位姿估计误差的不确定性/////////
      V3D point_world = state_propagat.rot_end * point_this + state_propagat.pos_end; // 根据当前的先验估计状态变换到世界坐标系，需注意不等于ptpl.point_w_
      Eigen::Matrix<double, 1, 6> J_nq; // 这里与 build_single_residual 函数内的部分计算类似但需要额外做一次，是因为 point_world ≠ pv.point_w ，同时 var ≠ pv.var
      J_nq.block<1, 3>(0, 0) = point_world - ptpl_list_[i].center_;
      J_nq.block<1, 3>(0, 3) = -ptpl_list_[i].normal_;

      M3D var;
      // V3D normal_b = state_.rot_end.inverse() * ptpl_list_[i].normal_;
      // V3D point_b = ptpl_list_[i].point_b_;
      // double cos_theta = fabs(normal_b.dot(point_b) / point_b.norm());
      // ptpl_list_[i].body_cov_ = ptpl_list_[i].body_cov_ * (1.0 / cos_theta) * (1.0 / cos_theta);

      // point_w cov
      // var = state_propagat.rot_end * extR_ * ptpl_list_[i].body_cov_ * (state_propagat.rot_end * extR_).transpose() +
      //       state_propagat.cov.block<3, 3>(3, 3) + (-point_crossmat) * state_propagat.cov.block<3, 3>(0, 0) * (-point_crossmat).transpose();

      // point_w cov (another_version)
      // var = state_propagat.rot_end * extR_ * ptpl_list_[i].body_cov_ * (state_propagat.rot_end * extR_).transpose() +
      //       state_propagat.cov.block<3, 3>(3, 3) - point_crossmat * state_propagat.cov.block<3, 3>(0, 0) * point_crossmat;

      // point_body cov
      var = state_propagat.rot_end * extR_ * ptpl_list_[i].body_cov_ * (state_propagat.rot_end * extR_).transpose();
      ////这里类似前文提到的 issue#89 没有严格遵守论文中的推导公式，仅保留了第一项，舍弃了后两项
      ////但这里的舍弃是完全正确的，因为对于观测噪音的协方差应应认为位姿估计（R,t）的不确定性（ rot_var ， t_var ）=（0，0）
      /////////

      double sigma_l = J_nq * ptpl_list_[i].plane_var_ * J_nq.transpose();

      R_inv(i) = 1.0 / (0.001 + sigma_l + ptpl_list_[i].normal_.transpose() * var * ptpl_list_[i].normal_); // 世界系的观测噪音协方差R的逆,额外引入了个0.001确保稳定
      // R_inv(i) = 1.0 / (sigma_l + ptpl_list_[i].normal_.transpose() * var * ptpl_list_[i].normal_);

      /*** calculate the Measuremnt Jacobian matrix H ***/
      // 关于H的计算公式推导可以参考[传感器融合课件](xxx)，观测雅可比矩阵H在上一次迭代的最优估计状态 state_ 处取
      V3D A(point_crossmat * state_.rot_end.transpose() * ptpl_list_[i].normal_);
      Hsub.row(i) << VEC_FROM_ARRAY(A), ptpl_list_[i].normal_[0], ptpl_list_[i].normal_[1], ptpl_list_[i].normal_[2];
      Hsub_T_R_inv.col(i) << A[0] * R_inv(i), A[1] * R_inv(i), A[2] * R_inv(i), ptpl_list_[i].normal_[0] * R_inv(i),
          ptpl_list_[i].normal_[1] * R_inv(i), ptpl_list_[i].normal_[2] * R_inv(i);
      meas_vec(i) = -ptpl_list_[i].dis_to_plane_; // 残差z，注意前面有个负号“-”
    }
    EKF_stop_flg = false;
    flg_EKF_converged = false;
    /*** Iterative Kalman Filter Update ***/
    MatrixXd K(DIM_STATE, effct_feat_num_);
    // auto &&Hsub_T = Hsub.transpose();
    auto &&HTz = Hsub_T_R_inv * meas_vec;
    // fout_dbg<<"HTz: "<<HTz<<endl;
    H_T_H.block<6, 6>(0, 0) = Hsub_T_R_inv * Hsub;
    // EigenSolver<Matrix<double, 6, 6>> es(H_T_H.block<6,6>(0,0));
    MD(DIM_STATE, DIM_STATE) &&K_1 = (H_T_H.block<DIM_STATE, DIM_STATE>(0, 0) + state_.cov.block<DIM_STATE, DIM_STATE>(0, 0).inverse()).inverse();
    G.block<DIM_STATE, 6>(0, 0) = K_1.block<DIM_STATE, 6>(0, 0) * H_T_H.block<6, 6>(0, 0);  // Gain = Kk*H
    auto vec = state_propagat - state_;  // StatesGroup 内已经重载了新的减法运算
    VD(DIM_STATE)
    solution = K_1.block<DIM_STATE, 6>(0, 0) * HTz + vec.block<DIM_STATE, 1>(0, 0) - G.block<DIM_STATE, 6>(0, 0) * vec.block<6, 1>(0, 0);
    int minRow, minCol;
    state_ += solution; // 状态更新，第κ次迭代得到的最优线性化点
    auto rot_add = solution.block<3, 1>(0, 0); // 姿态增量
    auto t_add = solution.block<3, 1>(3, 0);   // 位置增量
    if ((rot_add.norm() * 57.3 < 0.01) && (t_add.norm() * 100 < 0.015)) { flg_EKF_converged = true; } // 收敛条件：状态增量是否足够小
    // V3D euler_cur = state_.rot_end.eulerAngles(2, 1, 0); // 作者遗留的中间变量，可注释

    /*** Rematch Judgement ***/

    if (flg_EKF_converged || ((rematch_num == 0) && (iterCount == (config_setting_.max_iterations_ - 2)))) { rematch_num++; } // 记录满足收敛条件的迭代次数，同时引入了兜底机制确保没有满足收敛也能停止迭代

    /*** Convergence Judgements and Covariance Update ***/
    if (!EKF_stop_flg && (rematch_num >= 2 || (iterCount == config_setting_.max_iterations_ - 1))) // 存在两次收敛就停止迭代
    {
      /*** Covariance Update ***/
      // _state.cov = (I_STATE - G) * _state.cov;  //注意虽然迭代过程一直在更新 state_ ，但直到收敛后才更新状态协方差 state_.cov 
      state_.cov.block<DIM_STATE, DIM_STATE>(0, 0) =
          (I_STATE.block<DIM_STATE, DIM_STATE>(0, 0) - G.block<DIM_STATE, DIM_STATE>(0, 0)) * state_.cov.block<DIM_STATE, DIM_STATE>(0, 0);
      // total_distance += (_state.pos_end - position_last).norm();
      position_last_ = state_.pos_end;
      // geoQuat_ = tf::createQuaternionMsgFromRollPitchYaw(euler_cur(0), euler_cur(1), euler_cur(2)); // 在发布 odom 之前专门计算了，这里可以与euler_cur同步注释掉

      // VD(DIM_STATE) K_sum  = K.rowwise().sum();
      // VD(DIM_STATE) P_diag = _state.cov.diagonal();
      EKF_stop_flg = true;
    }
    if (EKF_stop_flg) break;
  }

  // double t2 = omp_get_wtime();
  // scan_count++;
  // ekf_time = t2 - t0 - build_residual_time;

  // ave_build_residual_time = ave_build_residual_time * (scan_count - 1) / scan_count + build_residual_time / scan_count;
  // ave_ekf_time = ave_ekf_time * (scan_count - 1) / scan_count + ekf_time / scan_count;

  // cout << "[ Mapping ] ekf_time: " << ekf_time << "s, build_residual_time: " << build_residual_time << "s" << endl;
  // cout << "[ Mapping ] ave_ekf_time: " << ave_ekf_time << "s, ave_build_residual_time: " << ave_build_residual_time << "s" << endl;
}

void VoxelMapManager::TransformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud,
                                     pcl::PointCloud<pcl::PointXYZI>::Ptr &trans_cloud)
{
  pcl::PointCloud<pcl::PointXYZI>().swap(*trans_cloud);
  trans_cloud->reserve(input_cloud->size());
  for (size_t i = 0; i < input_cloud->size(); i++)
  {
    pcl::PointXYZINormal p_c = input_cloud->points[i];
    Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
    p = (rot * (extR_ * p + extT_) + t);
    pcl::PointXYZI pi;
    pi.x = p(0);
    pi.y = p(1);
    pi.z = p(2);
    pi.intensity = p_c.intensity;
    trans_cloud->points.push_back(pi);
  }
}

void VoxelMapManager::BuildVoxelMap()
{
  float voxel_size = config_setting_.max_voxel_size_; // 最大体素0.5m
  float planer_threshold = config_setting_.planner_threshold_; // 最小特征值的最大阈值
  int max_layer = config_setting_.max_layer_;  // 0-2共3层 
  int max_points_num = config_setting_.max_points_num_; // 50个点收敛
  std::vector<int> layer_init_num = config_setting_.layer_init_num_; // 初始化点数量最低阈值默认为[5，5，5，5，5]

  std::vector<pointWithVar> input_points;

  for (size_t i = 0; i < feats_down_world_->size(); i++) // 遍历所有的世界系下的下采样点云
  {
    pointWithVar pv;
    pv.point_w << feats_down_world_->points[i].x, feats_down_world_->points[i].y, feats_down_world_->points[i].z;
    V3D point_this(feats_down_body_->points[i].x, feats_down_body_->points[i].y, feats_down_body_->points[i].z);
    M3D var;
    calcBodyCov(point_this, config_setting_.dept_err_, config_setting_.beam_err_, var); // 基于设定的雷达误差（深度和方向）计算体坐标系点云协方差表示不确定性
    M3D point_crossmat; 
    point_crossmat << SKEW_SYM_MATRX(point_this);
    //////作者在github的issue#89和VoxelMap的issue#15承认此处是存在bug的，但不能轻易按论文直接修改,参考./Supplementary/var协方差计算公式的Bug.png/////////////////
    var = (state_.rot_end * extR_) * var * (state_.rot_end * extR_).transpose() +
          (-point_crossmat) * state_.cov.block<3, 3>(0, 0) * (-point_crossmat).transpose() + state_.cov.block<3, 3>(3, 3);
    ///具体表现为///////
    pv.var = var; // 体坐标系协方差传播到世界坐标系
    input_points.push_back(pv);
    
  }

  uint plsize = input_points.size();
  for (uint i = 0; i < plsize; i++) // 遍历所有点分布到voxel_map_中
  {
    const pointWithVar p_v = input_points[i];
    float loc_xyz[3];
    for (int j = 0; j < 3; j++)
    {
      loc_xyz[j] = p_v.point_w[j] / voxel_size;
      if (loc_xyz[j] < 0) { loc_xyz[j] -= 1.0; }
    } // 将点云坐标除以体素大小，得到体素坐标
    VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]); // 获得体素坐标对应的key
    auto iter = voxel_map_.find(position); // 返回该体素对应的八叉树节点索引
    if (iter != voxel_map_.end()) // 已有节点
    {
      voxel_map_[position]->temp_points_.push_back(p_v);  
      voxel_map_[position]->new_points_++;
    }
    else // 在第0层构建新的根节点
    {
      VoxelTree *octo_tree = new VoxelTree(max_layer, 0, layer_init_num[0], max_points_num, planer_threshold);// 设置最大层索引2. 当前层数为 0，点的数量阈值 5，点数上限 50，最小特征值上限0.005
      voxel_map_[position] = octo_tree;
      voxel_map_[position]->quater_length_ = voxel_size / 4; // 为了方便计算子体素中心点的世界位置
      voxel_map_[position]->voxel_center_[0] = (0.5 + position.x) * voxel_size; // 该位置对于体素的中心点世界系坐标
      voxel_map_[position]->voxel_center_[1] = (0.5 + position.y) * voxel_size;
      voxel_map_[position]->voxel_center_[2] = (0.5 + position.z) * voxel_size;
      voxel_map_[position]->temp_points_.push_back(p_v); //即将参与面构建的点
      voxel_map_[position]->new_points_++;  // 暂未参与面构建的新点个数
      voxel_map_[position]->layer_init_num_ = layer_init_num;
    }
  }
  for (auto iter = voxel_map_.begin(); iter != voxel_map_.end(); ++iter) // 遍历所有根节点进行初始化
  {
    iter->second->init_octo_tree(); // iter->first 表示该元素的键（key）iter->second 表示该元素的值（value）
  }
}

V3F VoxelMapManager::RGBFromVoxel(const V3D &input_point)
{
  int64_t loc_xyz[3];
  for (int j = 0; j < 3; j++)
  {
    loc_xyz[j] = floor(input_point[j] / config_setting_.max_voxel_size_);
  }

  VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
  int64_t ind = loc_xyz[0] + loc_xyz[1] + loc_xyz[2];
  uint k((ind + 100000) % 3);
  V3F RGB((k == 0) * 255.0, (k == 1) * 255.0, (k == 2) * 255.0);
  // cout<<"RGB: "<<RGB.transpose()<<endl;
  return RGB;
}

void VoxelMapManager::UpdateVoxelMap(const std::vector<pointWithVar> &input_points)
{
  float voxel_size = config_setting_.max_voxel_size_;
  float planer_threshold = config_setting_.planner_threshold_;
  int max_layer = config_setting_.max_layer_;
  int max_points_num = config_setting_.max_points_num_;
  std::vector<int> layer_init_num = config_setting_.layer_init_num_;
  uint plsize = input_points.size();
  for (uint i = 0; i < plsize; i++) // 遍历所有的新点分布到 voxel_map_ 中
  {
    const pointWithVar p_v = input_points[i];
    float loc_xyz[3];
    for (int j = 0; j < 3; j++)
    {
      loc_xyz[j] = p_v.point_w[j] / voxel_size;
      if (loc_xyz[j] < 0) { loc_xyz[j] -= 1.0; }
    }
    VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
    auto iter = voxel_map_.find(position);
    if (iter != voxel_map_.end()) { voxel_map_[position]->UpdateOctoTree(p_v); } // 已有对应体素
    else // 构建新体素
    {
      VoxelTree *octo_tree = new VoxelTree(max_layer, 0, layer_init_num[0], max_points_num, planer_threshold);
      voxel_map_[position] = octo_tree;
      voxel_map_[position]->layer_init_num_ = layer_init_num;
      voxel_map_[position]->quater_length_ = voxel_size / 4;
      voxel_map_[position]->voxel_center_[0] = (0.5 + position.x) * voxel_size;
      voxel_map_[position]->voxel_center_[1] = (0.5 + position.y) * voxel_size;
      voxel_map_[position]->voxel_center_[2] = (0.5 + position.z) * voxel_size;
      voxel_map_[position]->UpdateOctoTree(p_v);
    }
  }
}

void VoxelMapManager::BuildResidualListOMP(std::vector<pointWithVar> &pv_list, std::vector<PointToPlane> &ptpl_list)
{
  int max_layer = config_setting_.max_layer_;
  double voxel_size = config_setting_.max_voxel_size_;
  double sigma_num = config_setting_.sigma_num_; //3σ原则，用于判断点是否在平面内
  std::mutex mylock;
  ptpl_list.clear();
  std::vector<PointToPlane> all_ptpl_list(pv_list.size());
  std::vector<bool> useful_ptpl(pv_list.size());
  std::vector<size_t> index(pv_list.size()); // 额外包一个索引列表，是为了满足openMP的循环并行化检索需要使用int而非size_t的要求
  for (size_t i = 0; i < index.size(); ++i)
  {
    index[i] = i;
    useful_ptpl[i] = false;
  }

  #ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);   // 最大4线程并行
    #pragma omp parallel for
  #endif
  for (int i = 0; i < index.size(); i++)
  {
    pointWithVar &pv = pv_list[i];
    float loc_xyz[3];
    for (int j = 0; j < 3; j++)
    {
      loc_xyz[j] = pv.point_w[j] / voxel_size;
      if (loc_xyz[j] < 0) { loc_xyz[j] -= 1.0; }
    }
    VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
    auto iter = voxel_map_.find(position); //获取根节点索引
    if (iter != voxel_map_.end()) //判断根节点是否存在，如果根节点不存在那面就一定不存在，表示这个点无需计算残差
    {
      VoxelTree *current_octo = iter->second;
      PointToPlane single_ptpl;
      bool is_sucess = false;
      double prob = 0;
      build_single_residual(pv, current_octo, 0, is_sucess, prob, single_ptpl); // 计算该点到体素内平面的带有协方差的绝对距离，认为该点匹配到平面的概率应满足3σ原则
      if (!is_sucess) // 所有父子节点都没成功匹配，需要找相邻体素
      {
        VOXEL_LOCATION near_position = position;
        if (loc_xyz[0] > (current_octo->voxel_center_[0] + current_octo->quater_length_)) { near_position.x = near_position.x + 1; }
        else if (loc_xyz[0] < (current_octo->voxel_center_[0] - current_octo->quater_length_)) { near_position.x = near_position.x - 1; }
        if (loc_xyz[1] > (current_octo->voxel_center_[1] + current_octo->quater_length_)) { near_position.y = near_position.y + 1; }
        else if (loc_xyz[1] < (current_octo->voxel_center_[1] - current_octo->quater_length_)) { near_position.y = near_position.y - 1; }
        if (loc_xyz[2] > (current_octo->voxel_center_[2] + current_octo->quater_length_)) { near_position.z = near_position.z + 1; }
        else if (loc_xyz[2] < (current_octo->voxel_center_[2] - current_octo->quater_length_)) { near_position.z = near_position.z - 1; }
        auto iter_near = voxel_map_.find(near_position); // 获取离当前点最近的那个相邻体素的索引
        if (iter_near != voxel_map_.end()) { build_single_residual(pv, iter_near->second, 0, is_sucess, prob, single_ptpl); }
      }
      if (is_sucess)
      {
        mylock.lock();
        useful_ptpl[i] = true;
        all_ptpl_list[i] = single_ptpl;
        mylock.unlock();
      }
      else // 相邻体素也没匹配成功，表示该点对计算残差无贡献
      {
        mylock.lock();
        useful_ptpl[i] = false;
        mylock.unlock();
      }
    }
  }
  for (size_t i = 0; i < useful_ptpl.size(); i++)
  {
    if (useful_ptpl[i]) { ptpl_list.push_back(all_ptpl_list[i]); } // 仅保留有效匹配
  }
}

void VoxelMapManager::build_single_residual(pointWithVar &pv, const VoxelTree *current_octo, const int current_layer, bool &is_sucess,
                                            double &prob, PointToPlane &single_ptpl)
{
  int max_layer = config_setting_.max_layer_;
  double sigma_num = config_setting_.sigma_num_; // 3σ原则，用于判断点是否在平面内

  double radius_k = 3;
  Eigen::Vector3d p_w = pv.point_w;
  if (current_octo->plane_ptr_->is_plane_)
  {
    VoxelPlane &plane = *current_octo->plane_ptr_;
    Eigen::Vector3d p_world_to_center = p_w - plane.center_;
    float dis_to_plane = fabs(plane.normal_(0) * p_w(0) + plane.normal_(1) * p_w(1) + plane.normal_(2) * p_w(2) + plane.d_); // 点到平面的绝对距离
    float dis_to_center = (plane.center_(0) - p_w(0)) * (plane.center_(0) - p_w(0)) + (plane.center_(1) - p_w(1)) * (plane.center_(1) - p_w(1)) +
                          (plane.center_(2) - p_w(2)) * (plane.center_(2) - p_w(2)); //点到平面中心的模长平方
    float range_dis = sqrt(dis_to_center - dis_to_plane * dis_to_plane); //点到平面中心点向量在面上投影的模长

    if (range_dis <= radius_k * plane.radius_) // 判断是否在3倍有效半径内
    {
      Eigen::Matrix<double, 1, 6> J_nq;
      J_nq.block<1, 3>(0, 0) = p_w - plane.center_;
      J_nq.block<1, 3>(0, 3) = -plane.normal_;
      double sigma_l = J_nq * plane.plane_var_ * J_nq.transpose(); // 基于平面不确定性
      sigma_l += plane.normal_.transpose() * pv.var * plane.normal_; // 加上点的不确定性
      if (dis_to_plane < sigma_num * sqrt(sigma_l)) // 判断是否满足3σ原则
      {
        is_sucess = true; // 该点成功匹配到平面
        double this_prob = 1.0 / (sqrt(sigma_l)) * exp(-0.5 * dis_to_plane * dis_to_plane / sigma_l); // 根据期望和协方差计算这个匹配的可信概率
        if (this_prob > prob) // 仅保留最大概率的那个匹配记录到 single_ptpl 
        {
          prob = this_prob;
          pv.normal = plane.normal_;
          single_ptpl.body_cov_ = pv.body_var;
          single_ptpl.point_b_ = pv.point_b;
          single_ptpl.point_w_ = pv.point_w;
          single_ptpl.plane_var_ = plane.plane_var_;
          single_ptpl.normal_ = plane.normal_;
          single_ptpl.center_ = plane.center_;
          single_ptpl.d_ = plane.d_;
          single_ptpl.layer_ = current_layer;
          single_ptpl.dis_to_plane_ = plane.normal_(0) * p_w(0) + plane.normal_(1) * p_w(1) + plane.normal_(2) * p_w(2) + plane.d_;
        }
        return;
      }
      else
      {
        // is_sucess = false;
        return;
      }
    }
    else
    {
      // is_sucess = false;
      return;
    }
  }
  else // 根节点不成面就向子节点递归查询
  {
    if (current_layer < max_layer)
    {
      for (size_t leafnum = 0; leafnum < 8; leafnum++) // 八个子节点内只要成功匹配到面就会存在一个可信概率，仅保留最大概率的那个匹配
      {
        if (current_octo->leaves_[leafnum] != nullptr)
        {

          VoxelTree *leaf_octo = static_cast<VoxelTree*>(current_octo->leaves_[leafnum]);
          build_single_residual(pv, leaf_octo, current_layer + 1, is_sucess, prob, single_ptpl);
        }
      }
      return;
    }
    else { return; }
  }
}

void VoxelMapManager::pubVoxelMap()
{
  int id_counter = 0;
  double max_trace = 0.25;
  double pow_num = 0.2;
  ros::Rate loop(500);
  float use_alpha = 0.8;
  visualization_msgs::MarkerArray voxel_plane;
  voxel_plane.markers.reserve(1000000);
  std::vector<VoxelPlane> pub_plane_list;
  for (auto iter = voxel_map_.begin(); iter != voxel_map_.end(); iter++)
  {
    GetUpdatePlane(iter->second, config_setting_.max_layer_, pub_plane_list);
  }
  for (size_t i = 0; i < pub_plane_list.size(); i++)
  {
    V3D plane_cov = pub_plane_list[i].plane_var_.block<3, 3>(0, 0).diagonal();
    double trace = plane_cov.sum();
    if (trace >= max_trace) { trace = max_trace; }
    trace = trace * (1.0 / max_trace);
    trace = pow(trace, pow_num);
    uint8_t r, g, b;
    mapJet(trace, 0, 1, r, g, b);
    Eigen::Vector3d plane_rgb(r / 256.0, g / 256.0, b / 256.0);
    double alpha;
    if (pub_plane_list[i].is_plane_) 
    {
      alpha = use_alpha; 
      id_counter++;
      pub_plane_list[i].id_ = id_counter;
      pubSinglePlane(voxel_plane, "plane", pub_plane_list[i], alpha, plane_rgb);
    }
    else { alpha = 0; }
    
  }
  voxel_map_pub_.publish(voxel_plane);
  // plane_pub.publish(voxel_plane);

  // ====== 新增：调试可视化体素边界与点云 ======
  visualization_msgs::MarkerArray bound_markers;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr debug_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  int marker_id = 0;

  for (auto& kv : voxel_map_) {
      collectDebugInfo(kv.second, bound_markers, debug_cloud, marker_id);
  }

  // 发布体素边界
  if (pub_voxel_bounds_) {
     pub_voxel_bounds_.publish(bound_markers);
  }

  // 发布体素内点云
  if (pub_voxel_points_ && !debug_cloud->empty()) {
      sensor_msgs::PointCloud2 cloud_msg;
      pcl::toROSMsg(*debug_cloud, cloud_msg);
      cloud_msg.header.frame_id = "camera_init";
      cloud_msg.header.stamp = ros::Time::now();
      pub_voxel_points_.publish(cloud_msg);
  }
  
  // loop.sleep();
}

void VoxelMapManager::GetUpdatePlane(const VoxelTree *current_octo, const int pub_max_voxel_layer, std::vector<VoxelPlane> &plane_list)
{
  if (current_octo->layer_ > pub_max_voxel_layer) { return; }
  if (current_octo->plane_ptr_->is_update_) { plane_list.push_back(*current_octo->plane_ptr_); }
  if (current_octo->layer_ < current_octo->max_layer_)
  {
    if (!current_octo->plane_ptr_->is_plane_)
    {
      for (size_t i = 0; i < 8; i++)
      {
        if (current_octo->leaves_[i] != nullptr) { GetUpdatePlane(static_cast<const VoxelTree*>(current_octo->leaves_[i]), pub_max_voxel_layer, plane_list); }
      }
    }
  }
  return;
}

void VoxelMapManager::pubSinglePlane(visualization_msgs::MarkerArray &plane_pub, const std::string plane_ns, const VoxelPlane &single_plane,
                                     const float alpha, const Eigen::Vector3d rgb)
{
  visualization_msgs::Marker plane;
  plane.header.frame_id = "camera_init";
  plane.header.stamp = ros::Time();
  plane.ns = plane_ns;
  plane.id = single_plane.id_;
  plane.type = visualization_msgs::Marker::CYLINDER;
  plane.action = visualization_msgs::Marker::ADD;
  plane.pose.position.x = single_plane.center_[0];
  plane.pose.position.y = single_plane.center_[1];
  plane.pose.position.z = single_plane.center_[2];
  geometry_msgs::Quaternion q;
  CalcVectQuation(single_plane.x_normal_, single_plane.y_normal_, single_plane.normal_, q);
  plane.pose.orientation = q;
  plane.scale.x = 3 * sqrt(single_plane.max_eigen_value_); // 根据平面特征值的大小调整平面在rviz中的显示大小，特征值越大表示平面越不确定，显示越小
  plane.scale.y = 3 * sqrt(single_plane.mid_eigen_value_);
  plane.scale.z = 2 * sqrt(single_plane.min_eigen_value_);
  plane.color.a = alpha;
  plane.color.r = rgb(0);
  plane.color.g = rgb(1);
  plane.color.b = rgb(2);
  plane.lifetime = ros::Duration();
  plane_pub.markers.push_back(plane);
}

// 新增递归函数，用于收集调试信息，包括边界框和点云数据
void VoxelMapManager::collectDebugInfo(const VoxelOctoTree* node,
                                       visualization_msgs::MarkerArray& bound_markers,
                                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                       int& marker_id) const
{
    if (!node) return;

    // 获取平面法向量（若存在）
    Eigen::Vector3d normal(0,0,1);
    bool has_plane = node->plane_ptr_->is_plane_;
    if (has_plane) {
        normal = node->plane_ptr_->normal_;
        normal.normalize();
    }

    // 计算法向量与理想墙面方向的偏差（假设理想墙面法向量为水平方向，即点乘 Z 轴应接近 0）
    // 我们用法向量与 Z 轴（垂直方向）的夹角来衡量：夹角接近 90° 说明是垂直墙面，接近 0° 说明是水平面。
    double dot_z = std::abs(normal.z());  // normal 已归一化
    // 夹角余弦值：dot_z 小表示垂直墙（正常），dot_z 大表示水平面或倾斜严重
    // 我们将 dot_z > 0.3 视为“异常”（即法向量过于垂直或倾斜）

    // 颜色映射：异常 → 红色，正常 → 绿色
    uint8_t r, g, b;
    if (has_plane && dot_z > 0.3) {  // 可根据实际情况调整阈值
        r = 255; g = 50; b = 50;    // 红色：异常
    } else if (has_plane) {
        r = 50; g = 255; b = 50;    // 绿色：正常垂直面
    } else {
        r = 200; g = 200; b = 0;    // 黄色：尚未形成平面
    }

    // 将该体素的所有临时点加入点云，并着色
    for (const auto& pv : node->temp_points_) {
        pcl::PointXYZRGB pt;
        pt.x = pv.point_w.x();
        pt.y = pv.point_w.y();
        pt.z = pv.point_w.z();
        pt.r = r; pt.g = g; pt.b = b;
        cloud->push_back(pt);
    }

    // 绘制体素边界（颜色与点云一致，但透明度降低）
    if (node->octo_state_ == 0 || node->plane_ptr_->is_plane_ || node->layer_ >= node->max_layer_) {
        // ...（边界绘制代码不变，颜色使用上面的 r,g,b）
        // 注意：边界颜色也应使用相同的 r,g,b，alpha 设为 0.3
    }

    // 递归子节点
    if (!node->plane_ptr_->is_plane_ && node->layer_ < node->max_layer_) {
        for (int i=0; i<8; ++i) {
            if (node->leaves_[i]) {
                collectDebugInfo(node->leaves_[i], bound_markers, cloud, marker_id);
            }
        }
    }
}

void VoxelMapManager::CalcVectQuation(const Eigen::Vector3d &x_vec, const Eigen::Vector3d &y_vec, const Eigen::Vector3d &z_vec,
                                      geometry_msgs::Quaternion &q)
{
  // Eigen::Matrix3d rot;
  // rot << x_vec(0), x_vec(1), x_vec(2), y_vec(0), y_vec(1), y_vec(2), z_vec(0), z_vec(1), z_vec(2);
  // Eigen::Matrix3d rotation = rot.transpose();
  // Eigen::Quaterniond eq(rotation);
  // eq.normalize();   // 确保单位四元数
  // q.w = eq.w();
  // q.x = eq.x();
  // q.y = eq.y();
  // q.z = eq.z();

  // 源代码的三个轴向量并不完全正交（x和y轴平面向量没有随着z轴法向量同步更新），因此改为仅使用法向量计算四元数
  Eigen::Quaterniond eq = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), z_vec);
  eq.normalize();
  q.w = eq.w();
  q.x = eq.x();
  q.y = eq.y();
  q.z = eq.z();
}

void VoxelMapManager::mapJet(double v, double vmin, double vmax, uint8_t &r, uint8_t &g, uint8_t &b)
{
  r = 255;
  g = 255;
  b = 255;

  if (v < vmin) { v = vmin; }

  if (v > vmax) { v = vmax; }

  double dr, dg, db;

  if (v < 0.1242)
  {
    db = 0.504 + ((1. - 0.504) / 0.1242) * v;
    dg = dr = 0.;
  }
  else if (v < 0.3747)
  {
    db = 1.;
    dr = 0.;
    dg = (v - 0.1242) * (1. / (0.3747 - 0.1242));
  }
  else if (v < 0.6253)
  {
    db = (0.6253 - v) * (1. / (0.6253 - 0.3747));
    dg = 1.;
    dr = (v - 0.3747) * (1. / (0.6253 - 0.3747));
  }
  else if (v < 0.8758)
  {
    db = 0.;
    dr = 1.;
    dg = (0.8758 - v) * (1. / (0.8758 - 0.6253));
  }
  else
  {
    db = 0.;
    dg = 0.;
    dr = 1. - (v - 0.8758) * ((1. - 0.504) / (1. - 0.8758));
  }

  r = (uint8_t)(255 * dr);
  g = (uint8_t)(255 * dg);
  b = (uint8_t)(255 * db);
}

void VoxelMapManager::mapSliding()
{
  if((position_last_ - last_slide_position).norm() < config_setting_.sliding_thresh)
  {
    std::cout<<RED<<"[DEBUG]: Last sliding length "<<(position_last_ - last_slide_position).norm()<<RESET<<"\n";
    return;
  }

  //get global id now
  last_slide_position = position_last_;
  double t_sliding_start = omp_get_wtime();
  float loc_xyz[3];
  for (int j = 0; j < 3; j++)
  {
    loc_xyz[j] = position_last_[j] / config_setting_.max_voxel_size_;
    if (loc_xyz[j] < 0) { loc_xyz[j] -= 1.0; }
  }
  // VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);//discrete global
  clearMemOutOfMap((int64_t)loc_xyz[0] + config_setting_.half_map_size, (int64_t)loc_xyz[0] - config_setting_.half_map_size,
                    (int64_t)loc_xyz[1] + config_setting_.half_map_size, (int64_t)loc_xyz[1] - config_setting_.half_map_size,
                    (int64_t)loc_xyz[2] + config_setting_.half_map_size, (int64_t)loc_xyz[2] - config_setting_.half_map_size);
  double t_sliding_end = omp_get_wtime();
  std::cout<<RED<<"[DEBUG]: Map sliding using "<<t_sliding_end - t_sliding_start<<" secs"<<RESET<<"\n";
  return;
}

void VoxelMapManager::clearMemOutOfMap(const int& x_max,const int& x_min,const int& y_max,const int& y_min,const int& z_max,const int& z_min )
{
  int delete_voxel_cout = 0;
  // double delete_time = 0;
  // double last_delete_time = 0;
  for (auto it = voxel_map_.begin(); it != voxel_map_.end(); )
  {
    const VOXEL_LOCATION& loc = it->first;
    bool should_remove = loc.x > x_max || loc.x < x_min || loc.y > y_max || loc.y < y_min || loc.z > z_max || loc.z < z_min;
    if (should_remove){
      // last_delete_time = omp_get_wtime();
      delete it->second;
      it = voxel_map_.erase(it);
      // delete_time += omp_get_wtime() - last_delete_time;
      delete_voxel_cout++;
    } else {
      ++it;
    }
  }
  std::cout<<RED<<"[DEBUG]: Delete "<<delete_voxel_cout<<" root voxels"<<RESET<<"\n";
  // std::cout<<RED<<"[DEBUG]: Delete "<<delete_voxel_cout<<" voxels using "<<delete_time<<" s"<<RESET<<"\n";
}