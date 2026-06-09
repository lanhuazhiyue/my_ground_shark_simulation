#include "voxel_PcaTree.h"

VoxelPcaTree::~VoxelPcaTree() {}

void VoxelPcaTree::cut_octo_tree() {
    // 暂用基类实现
    VoxelOctoTree::cut_octo_tree();
}

VoxelOctoTree* VoxelPcaTree::find_correspond(Eigen::Vector3d pw) {
    return VoxelOctoTree::find_correspond(pw);
}

VoxelOctoTree* VoxelPcaTree::Insert(const pointWithVar &pv) {
    return VoxelOctoTree::Insert(pv);
}

void VoxelPcaTree::UpdateOctoTree(const pointWithVar &pv) {
    VoxelOctoTree::UpdateOctoTree(pv);
}