#ifndef VOXEL_TREE_ALIAS_H
#define VOXEL_TREE_ALIAS_H

#ifdef PCA_MAPPING
  #include "voxel_PcaTree.h"
  using VoxelTree = VoxelPcaTree;
  #define VOXEL_TREE_NAME "VoxelPcaTree"
#else
  #include "voxel_OctoTree.h"
  using VoxelTree = VoxelOctoTree;
  #define VOXEL_TREE_NAME "VoxelOctoTree"
#endif

#endif