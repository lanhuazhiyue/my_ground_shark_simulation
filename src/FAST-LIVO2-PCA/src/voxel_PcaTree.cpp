#include "voxel_PcaTree.h"

VoxelPcaTree::~VoxelPcaTree() {}

// 核心：在创建子节点的地方都用 new VoxelPcaTree，其余逻辑与基类完全一致
void VoxelPcaTree::cut_octo_tree()
{
    // 完全复制基类的 cut_octo_tree，但将所有 new VoxelOctoTree 替换为 new VoxelPcaTree
    // 同时由于 leaves_ 类型为 VoxelOctoTree*，向上转型是隐式的
    if (layer_ >= max_layer_)
    {
        octo_state_ = 0;
        return;
    }

    for (size_t i = 0; i < temp_points_.size(); i++)
    {
        int xyz[3] = {0, 0, 0};
        if (temp_points_[i].point_w[0] > voxel_center_[0]) { xyz[0] = 1; }
        if (temp_points_[i].point_w[1] > voxel_center_[1]) { xyz[1] = 1; }
        if (temp_points_[i].point_w[2] > voxel_center_[2]) { xyz[2] = 1; }
        int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
        if (leaves_[leafnum] == nullptr)
        {
            // 唯一修改：用 VoxelPcaTree 替代 VoxelOctoTree
            leaves_[leafnum] = new VoxelPcaTree(max_layer_, layer_ + 1,
                                                layer_init_num_[layer_ + 1],
                                                max_points_num_, planer_threshold_);
            leaves_[leafnum]->layer_init_num_ = layer_init_num_;
            leaves_[leafnum]->voxel_center_[0] = voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
            leaves_[leafnum]->voxel_center_[1] = voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
            leaves_[leafnum]->voxel_center_[2] = voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
            leaves_[leafnum]->quater_length_ = quater_length_ / 2;
        }
        leaves_[leafnum]->temp_points_.push_back(temp_points_[i]);
        leaves_[leafnum]->new_points_++;
    }

    for (uint i = 0; i < 8; i++)
    {
        if (leaves_[i] != nullptr)
        {
            if (leaves_[i]->temp_points_.size() > (size_t)leaves_[i]->points_size_threshold_)
            {
                init_plane(leaves_[i]->temp_points_, leaves_[i]->plane_ptr_);
                if (leaves_[i]->plane_ptr_->is_plane_)
                {
                    leaves_[i]->octo_state_ = 0;
                    if (leaves_[i]->temp_points_.size() > (size_t)leaves_[i]->max_points_num_)
                    {
                        leaves_[i]->update_enable_ = false;
                        std::vector<pointWithVar>().swap(leaves_[i]->temp_points_);
                        leaves_[i]->new_points_ = 0;
                    }
                }
                else
                {
                    leaves_[i]->octo_state_ = 1;
                    leaves_[i]->cut_octo_tree(); // 递归，会调用 VoxelPcaTree::cut_octo_tree()
                }
                leaves_[i]->init_octo_ = true;
                leaves_[i]->new_points_ = 0;
            }
        }
    }
}

// init_octo_tree 同样用派生类的 cut_octo_tree
void VoxelPcaTree::init_octo_tree()
{
    if (temp_points_.size() > (size_t)points_size_threshold_)
    {
        init_plane(temp_points_, plane_ptr_);
        if (plane_ptr_->is_plane_)
        {
            octo_state_ = 0;
            if (temp_points_.size() > (size_t)max_points_num_)
            {
                update_enable_ = false;
                std::vector<pointWithVar>().swap(temp_points_);
                new_points_ = 0;
            }
        }
        else
        {
            octo_state_ = 1;
            cut_octo_tree();   // 调用 VoxelPcaTree::cut_octo_tree
        }
        init_octo_ = true;
        new_points_ = 0;
    }
}

// find_correspond：内部调用基类，然后安全转换（因为我们知道子节点是 VoxelPcaTree）
VoxelPcaTree* VoxelPcaTree::find_correspond(Eigen::Vector3d pw)
{
    // 直接调用基类的 find_correspond，它返回 VoxelOctoTree*
    VoxelOctoTree* node = VoxelOctoTree::find_correspond(pw);
    // 基类可能返回 this（此时 this 是 VoxelPcaTree*），或子节点（也是 VoxelPcaTree*），所以 static_cast 安全
    return static_cast<VoxelPcaTree*>(node);
}

// Insert 类似处理
VoxelPcaTree* VoxelPcaTree::Insert(const pointWithVar &pv)
{
    VoxelOctoTree* node = VoxelOctoTree::Insert(pv);
    return static_cast<VoxelPcaTree*>(node);
}

// UpdateOctoTree：创建子节点时用 VoxelPcaTree，其余调用基类
void VoxelPcaTree::UpdateOctoTree(const pointWithVar &pv)
{
    // 沿用基类逻辑，但在需要新建子节点时，改为 new VoxelPcaTree
    // 为了避免大量复制代码，我们仍然调用基类版本，但在基类中创建子节点的部分（cut_octo_tree）已经被我们重写，
    // 所以当基类 UpdateOctoTree 内部调用 cut_octo_tree 时会自动用 VoxelPcaTree 版本。
    // 因此直接调用基类 UpdateOctoTree 即可，但需要确保基类版本不会创建 VoxelOctoTree。
    // 仔细分析基类 UpdateOctoTree：在非平面且需要向下传递时，如果子节点不存在，会 new VoxelOctoTree 并设置 voxel_center_ 等。
    // 为了安全，我们重写整个 UpdateOctoTree，完全复制基类代码，仅将 new VoxelOctoTree 改为 new VoxelPcaTree。

    // 为了简洁，我们可以采用另一种方式：调用基类版本，因为基类版本内部创建子节点是通过 cut_octo_tree 或直接 new，但 cut_octo_tree 已被重写，
    // 而直接 new 的地方只有一处：在 layer_ < max_layer_ 且 leaves_[leafnum] == nullptr 时，
    // 基类会 new VoxelOctoTree。我们需要接管这部分。因此这里提供一个重写的简化版本：
    if (!init_octo_)
    {
        new_points_++;
        temp_points_.push_back(pv);
        if (temp_points_.size() > (size_t)points_size_threshold_) { init_octo_tree(); }
    }
    else
    {
        if (plane_ptr_->is_plane_)
        {
            if (update_enable_)
            {
                new_points_++;
                temp_points_.push_back(pv);
                if (new_points_ > update_size_threshold_)
                {
                    init_plane(temp_points_, plane_ptr_);
                    new_points_ = 0;
                }
                if (temp_points_.size() >= (size_t)max_points_num_)
                {
                    update_enable_ = false;
                    std::vector<pointWithVar>().swap(temp_points_);
                    new_points_ = 0;
                }
            }
        }
        else
        {
            if (layer_ < max_layer_)
            {
                int xyz[3] = {0, 0, 0};
                if (pv.point_w[0] > voxel_center_[0]) { xyz[0] = 1; }
                if (pv.point_w[1] > voxel_center_[1]) { xyz[1] = 1; }
                if (pv.point_w[2] > voxel_center_[2]) { xyz[2] = 1; }
                int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
                if (leaves_[leafnum] != nullptr)
                {
                    // 递归调用派生类版本
                    static_cast<VoxelPcaTree*>(leaves_[leafnum])->UpdateOctoTree(pv);
                }
                else
                {
                    // 创建 VoxelPcaTree 子节点
                    leaves_[leafnum] = new VoxelPcaTree(max_layer_, layer_ + 1,
                                                        layer_init_num_[layer_ + 1],
                                                        max_points_num_, planer_threshold_);
                    leaves_[leafnum]->layer_init_num_ = layer_init_num_;
                    leaves_[leafnum]->voxel_center_[0] = voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
                    leaves_[leafnum]->voxel_center_[1] = voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
                    leaves_[leafnum]->voxel_center_[2] = voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
                    leaves_[leafnum]->quater_length_ = quater_length_ / 2;
                    leaves_[leafnum]->UpdateOctoTree(pv);
                }
            }
            else
            {
                // 最大层，直接处理（与基类相同）
                VoxelOctoTree::UpdateOctoTree(pv);
            }
        }
    }
}