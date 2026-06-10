#ifndef VOXEL_PCATREE_H
#define VOXEL_PCATREE_H

#include "voxel_OctoTree.h"

class VoxelPcaTree : public VoxelOctoTree
{
public:
    // 继承构造函数
    using VoxelOctoTree::VoxelOctoTree;
    ~VoxelPcaTree();

    // 重写这些函数以实现 PCA 切分逻辑
    void init_octo_tree();
    void cut_octo_tree();
    VoxelPcaTree* find_correspond(Eigen::Vector3d pw);
    VoxelPcaTree* Insert(const pointWithVar &pv);
    void UpdateOctoTree(const pointWithVar &pv);

private:
    // PCA 切分用到的额外成员
    Eigen::Vector3d split_plane_normal_ = Eigen::Vector3d::UnitX();
    double split_plane_offset_ = 0.0;
    bool has_split_plane_ = false;
};

#endif