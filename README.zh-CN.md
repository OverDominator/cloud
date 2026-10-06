# Unity–ROS 消防搜救仿真项目

[English](README.md) | [简体中文](README.zh-CN.md)

本项目是一个可配置的室内机器人搜救软件在环原型，基于 Unity 和 ROS，将自主探索、模拟受困者线索、FAR 路径规划、机器人运动执行和救援确认连接成完整任务流程。当前实现还提供键盘驾驶和人工驾驶轨迹记录功能。

本文件为经项目所有者评审的中文版说明。英文版见 [README.md](README.md)。中英文说明已按现有验证记录同步；文档更新不表示完成了新的运行验证。

当前项目是**私有发布候选版本**，不是已经验证可独立安装的完整发行版。仓库为 [OverDominator/cloud](https://github.com/OverDominator/cloud)。外部 ROS 依赖、跨机器部署和再分发条件仍需完善，详见[发布检查清单](docs/zh-CN/release_checklist.md)。本地依赖快照和完整验证证据目录未包含在仓库中。

## 演示视频

[下载 Easy 场景完整演示视频](media/fire-rescue-demo-v0.1.0.mp4)。仓库保持私有期间，需要仓库访问权限。

视频同时展示同一轮运行的 Unity 外部观察视角、机器人 RGB 图像和 ROS/RViz 规划视图。视频保留完整录制过程和原始播放速度，添加了说明文字及片尾结果，没有配音或音轨。外部观察视角不是机器人感知输入；RGB 图像也不代表物理热成像。

该轮完成了全部四名模拟受困者的救援。任务记录器记录的任务用时为 **284.684 秒**，行驶距离为 **179.979 米**，碰撞事件记录为零。这是单独录制的演示，**不计入主实验200轮**。详见[演示证据说明](docs/recorded_demo.txt)与[该轮结果](results/demo_20261005/trials.csv)。它证明一次成功运行，不代表每次运行都能成功，也不是全新电脑安装复现的证明。

## 验证状态

2026年10月5日，独立验证副本在 Easy 场景完成了一轮完整自主任务，全部四名模拟受困者获救，任务耗时约 **205.6 秒**，记录行驶距离约 **131.6 米**。该轮使用独立构建的本地 ROS 依赖及单独日志，是发布验证，不是主实验的新增轮次，也不是上述视频中的那一轮。详见[验证证据摘要](docs/full_validation_complete.txt)。

已在现有 Windows/WSL 电脑上检查 Unity 6000.0.75f1 项目导入、ROS Noetic 构建、启动文件解析、双向通信和 Depth Anything V2 离线加载。**全新机器安装、跨轮次重复可靠性以及与历史依赖完全一致，尚未验证。**

当前速度指令适配器已排除主动循墙、倒退脱困和沿历史轨迹回退程序，仍保留安全检查以及规划器可能输出的有界负速度指令。因此，“移除主动倒退恢复”不等于“任何情况下都不允许负速度”。补充测试场景及其可执行辅助程序已从此候选版本中移除，补充实验 CSV 仅作为历史结果保留。

## 在已准备好的环境中启动

先构建发布包和经过修改的外部依赖，然后指定工作空间环境文件，检查包是否能被找到：

```bash
FIRE_ROS_SETUP=/path/to/ros_workspace/devel/setup.bash \
FIRE_CMU_SETUP=/path/to/cmu_workspace/devel/setup.bash \
FIRE_RUN_DIR=/path/to/new_run_directory \
bash Tools/ROS/start_release_nav.sh --check
```

所提供的工作空间必须包含 FAR、自定义 Unity 集成和 ROS-TCP-Endpoint。`--check` 只检查包发现及相关启动文件，不下载或构建依赖，也不启动 ROS 进程。准备运行时去掉 `--check`。Unity 和可视化窗口需要另行启动。

已录制演示在配置好的 Windows/WSL 验证环境使用了该入口，但这不代表全新机器安装成功，也不代表精确复现历史实验环境。旧快捷启动脚本仍含特定机器路径。详见[安装与依赖说明](docs/zh-CN/setup.md)。

Depth Anything V2 需要另行准备模型文件。权重缺失时，现有定位模块会继续使用仅 LiDAR 的定位方式；若要测试包含深度分支的流程，应确认模型就绪状态。模型说明见 [perception_dependency_check.txt](docs/perception_dependency_check.txt)。

## 仓库包含的内容

- `unity/`：Unity 项目资源、场景配置，以及本地使用的 ROS-TCP-Connector 包。
- `ros/fire_perception/`：FAR 工作流程中的感知、任务管理、安全检查及记录代码。
- `Tools/`：检查与录制辅助工具；部分路径仍与原电脑相关。
- `results/main_200/`：四个原始批次的结果摘要和部分历史代码快照。
- `results/supplementary_30/`：单独报告的选定局部障碍实验结果。
- `results/demo_20261005/`：单独录制演示的结果摘要。
- `third_party/`：保留的连接器许可证及声明。

请结合[操作说明](docs/zh-CN/operation.md)、[实验范围](docs/zh-CN/experiments.md)和[中文评审注意事项](docs/zh-CN/review_notes.md)阅读。

## 实验结果与适用范围

每轮主实验的时间限制为600秒。Original 场景100轮有效终止实验中完成61轮，Easy 场景100轮中完成92轮。选定补充批次包括 End Wall、Right Angle 和 Left Angle，每种配置各有10轮成功的单受困者任务。补充实验与主实验不同，不能合并统计。

这些结果评价的是指定仿真假设下的自主运行，不是人工驾驶效果或专业消防员训练效果。当前开发代码不被表述为200轮实验配置的精确副本；历史快照是部分溯源证据，不是完整、可执行的历史复现包。

机器人位姿与火源位置使用仿真信息，视觉目标线索经过简化。本项目未验证物理雷达、热辐射传感、独立 SLAM 定位精度或真实火场部署表现。FAR Planner 和 Depth Anything V2 是采用的已有方法，并非本项目提出的新算法。

## 许可证状态

发布候选版本尚未指定覆盖整个项目的许可证。ROS 包元数据声明 MIT，但公开发布前仍需核实所有权及可再分发范围。第三方权利归相应权利人所有，详见[第三方声明中文版](docs/zh-CN/third_party_notices.md)。
