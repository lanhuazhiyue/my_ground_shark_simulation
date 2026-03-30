# 带有mid360雷达的无人车模型

本工作空间是由solidworks的urdf插件自动生成的

对urdf文件夹进行了以下改动：
    
1. 为了使用麦克纳姆轮的gazebo仿真插件，修改车轮关节为固定关节，建立了新的 *ground_shark_mid360_fixed.urdf*
2. 新增了*mid360_gazebo.xacro*文件，是mid360雷达的gazebo插件，用于仿真雷达数据
3. 新增了*my_ground_robot_mid360.xacro*文件，是将仿真插件和模型文件结合在一起了，以便在gazebo中加载。
