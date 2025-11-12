# ROS 安装指南和示例项目

## 项目概述

本项目提供了完整的ROS（Robot Operating System）安装指南和示例代码，基于参考项目 [ROS-based-USV-project](https://github.com/matreshka15/ROS-based-USV-project)。

ROS是一个开源的机器人操作系统框架，提供了硬件抽象、设备驱动、库函数、可视化工具等功能，广泛应用于机器人开发领域。

## 目录结构

```
ROS_Installation_Guide/
├── README.md                      # 项目说明
├── ROS_Installation_Guide.md      # 详细安装指南
├── install_ros.sh                 # 一键安装脚本
├── test_ros.sh                    # 安装测试脚本
├── examples/                      # 示例代码
│   ├── talker.py                  # 发布者节点示例
│   └── listener.py                # 订阅者节点示例
├── config/                        # 配置文件
│   └── example_config.yaml        # 配置文件示例
└── launch/                        # 启动文件
    └── example.launch             # 启动文件示例
```

## 快速开始

### 1. 一键安装ROS

```bash
# 下载安装脚本

# 添加执行权限
chmod +x install_ros.sh

# 运行安装脚本
./install_ros.sh
```

### 2. 验证安装

```bash
# 下载测试脚本

# 添加执行权限
chmod +x test_ros.sh

# 运行测试脚本
./test_ros.sh
```

### 3. 运行示例代码

```bash
# 启动ROS核心
roscore

# 在新终端中运行发布者
python3 examples/talker.py

# 在另一个新终端中运行订阅者
python3 examples/listener.py
```

## 详细安装步骤

请参考 [ROS_Installation_Guide.md](ROS_Installation_Guide.md) 获取详细的安装说明，包括：

- 系统要求和版本选择
- 手动安装步骤
- 常见问题解决
- 环境配置
- 开发工具安装

## 使用说明

### 安装脚本说明

**install_ros.sh** - 一键安装脚本功能：
- 系统版本检查
- 软件源备份和更新
- 自动添加ROS源和密钥
- 安装ROS Noetic完整版
- 配置环境变量
- 安装开发工具
- 初始化rosdep（使用国内优化版rosdepc）
- 创建工作空间
- 安装额外工具

### 测试脚本说明

**test_ros.sh** - 安装测试脚本功能：
- 检查ROS环境变量
- 验证roscore启动
- 检查rosdep配置
- 验证工作空间
- 测试turtlesim功能

### 示例代码说明

**examples/talker.py** - 发布者节点：
- 以10Hz频率发布字符串消息
- 包含时间戳和计数器
- 完整的错误处理和日志记录

**examples/listener.py** - 订阅者节点：
- 订阅chatter话题
- 计算消息延迟和统计信息
- 详细的日志输出

### 配置文件说明

**config/example_config.yaml** - 配置文件示例：
- 节点配置参数
- 传感器配置
- 控制器参数
- 导航配置
- 硬件配置
- 安全配置

### 启动文件说明

**launch/example.launch** - 启动文件示例：
- 启动参数配置
- 多节点启动
- 参数设置和加载
- 条件执行
- 命名空间管理
- 依赖关系处理

## 常用命令

### ROS基础命令

```bash
# 启动ROS核心
roscore

# 运行节点
rosrun package_name node_name

# 启动launch文件
roslaunch package_name launch_file.launch

# 查看节点信息
rosnode list
rosnode info node_name

# 查看话题信息
rostopic list
rostopic echo topic_name
rostopic hz topic_name

# 查看服务信息
rosservice list
rosservice call service_name

# 查看参数信息
rosparam list
rosparam get param_name
rosparam set param_name value
```

### 工作空间命令

```bash
# 创建工作空间
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make

# 设置环境变量
source devel/setup.bash

# 创建功能包
cd src
catkin_create_pkg package_name dependencies

# 编译工作空间
cd ~/catkin_ws
catkin_make

# 清理工作空间
catkin_make clean
```

### 调试命令

```bash
# 查看ROS日志
rosclean check
rosclean purge

# 录制和回放数据
rosbag record -a
rosbag play bag_file.bag

# 动态重配置
rosrun rqt_reconfigure rqt_reconfigure

# 可视化工具
rviz
rqt
rqt_graph
```

## 学习资源

### 官方文档

- [ROS官方网站](http://www.ros.org/)
- [ROS Noetic文档](http://wiki.ros.org/noetic)
- [ROS教程](http://wiki.ros.org/ROS/Tutorials)

### 国内资源

- [鱼香ROS](https://fishros.com/)
- [ROS中文社区](http://www.rosclub.cn/)
- [ROS中文教程](http://wiki.ros.org/cn)

### 书籍推荐

- 《ROS机器人开发实践》
- 《A Gentle Introduction to ROS》
- 《ROS By Example》

## 常见问题

### 1. 网络连接问题

**问题**: 无法连接到ROS服务器或下载速度慢

**解决方案**:
- 使用国内镜像源
- 检查网络连接
- 使用代理服务器
- 安装rosdepc替代rosdep

### 2. 权限问题

**问题**: 执行脚本时出现权限错误

**解决方案**:
```bash
chmod +x script_name.sh
sudo ./script_name.sh
```

### 3. 依赖问题

**问题**: 安装过程中出现依赖错误

**解决方案**:
```bash
sudo apt --fix-broken install
sudo apt autoremove
sudo apt autoclean
```

### 4. 环境变量问题

**问题**: 找不到ROS命令

**解决方案**:
```bash
source /opt/ros/noetic/setup.bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```

## 贡献指南

欢迎贡献代码、文档或提出问题。请遵循以下步骤：

1. Fork本项目
2. 创建特性分支
3. 提交更改
4. 推送分支
5. 创建Pull Request

## 许可证

本项目采用MIT许可证，详情请参考 [LICENSE](LICENSE) 文件。

## 联系信息

如果您有任何问题或建议，请通过以下方式联系：

- 参考项目: https://github.com/matreshka15/ROS-based-USV-project

---

**最后更新**: 2025年11月12日
**版本**: 1.0