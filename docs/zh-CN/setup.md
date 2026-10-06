# 安装与依赖状态

[返回中文首页](../../README.zh-CN.md)

## 已知环境

- Unity 编辑器：`6000.0.75f1`，来自源项目的 `ProjectVersion.txt`。
- Windows 上运行 Unity，WSL 发行版为 `Ubuntu-20.04`。
- ROS Noetic。
- 内嵌 ROS-TCP-Connector 包元数据版本：`0.7.0-preview`，保留了本地修改。
- 深度分支依赖见 `ros/fire_perception/requirements-depth.txt`；完整、精确的工作版本锁定仍需完善。

## 打开 Unity 项目

使用匹配版本的编辑器打开 `unity/`。主要场景包括 `SampleScene`、`FireRescue_Simplified`（Original 配置）及 `FireRescue_Easy`。现有构建设置最初选择 `SampleScene`，应显式打开所需场景，并保留 `.meta` 文件。

不要同时运行连接到同一 ROS 端点和话题的原项目与发布副本，避免两个仿真实例互相干扰。

## 外部依赖

运行还需要 FAR Planner、ROS-TCP-Endpoint，以及自主探索开发环境中的局部规划、地形处理等包；它们没有作为完整源码树包含在本候选版本中。必须核对所用提交、本地补丁及许可证，才能承诺完整安装复现。

推荐入口是 `Tools/ROS/start_release_nav.sh`。通过 `FIRE_ROS_SETUP` 和 `FIRE_CMU_SETUP` 指定已经构建的工作空间；`FIRE_RUN_DIR` 指定新的运行目录。`--check` 是检查模式，去掉后才启动系统。工作空间需要包含自定义 Unity 启动文件，不能假设未经修改的上游包已具备该文件。

旧的 `Tools/ROS/start_fire_nav.sh` 假设存在 `/root/catkin_ws` 和 `/root/autonomous_exploration_development_environment`；`install_shortcuts.sh` 也包含用户相关位置。它们不应被当成通用一键安装器直接执行。

深度定位模块默认模型为 `depth-anything/Depth-Anything-V2-Small-hf`，可通过配置覆盖。仓库不包含模型权重。使用前需确认实际模型路径、配置及许可证。

## 验证范围

现有记录报告在当前电脑上完成 Unity 导入、独立构建、通信及一次完整任务；之后又完成了单独的视频演示。演示使用了新的启动入口。英文安装说明和入口说明已同步这些状态。

这些进展不等于全新机器安装验证。仍需完成外部依赖交付、路径与端点说明、精确版本记录及跨机器安装测试。证据见 [完整运行验证](../full_validation_complete.txt)及[录制演示记录](../recorded_demo.txt)。
