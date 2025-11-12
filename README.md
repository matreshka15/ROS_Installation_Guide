# ROS 安装指南

## 目录

1. [项目概述](#项目概述)
2. [系统要求](#系统要求)
3. [ROS 版本选择](#ros版本选择)
4. [安装步骤](#安装步骤)
5. [自动化安装脚本](#自动化安装脚本)
6. [测试验证](#测试验证)
7. [常见问题解决](#常见问题解决)
8. [示例代码](#示例代码)
9. [参考资料](#参考资料)

## 项目概述

本项目提供了完整的 ROS（Robot Operating System）安装指南，基于参考项目 [ROS-based-USV-project](https://github.com/matreshka15/ROS-based-USV-project)。ROS 是一个开源的机器人操作系统框架，提供了硬件抽象、设备驱动、库函数、可视化工具等功能，广泛应用于机器人开发领域。

## 系统要求

### 支持的操作系统

| ROS 版本            | Ubuntu 版本        | 支持状态    |
| ----------------- | ---------------- | ------- |
| Noetic Ninjemys   | Ubuntu 20.04 LTS | ✅ 推荐    |
| Melodic Morenia   | Ubuntu 18.04 LTS | ✅ 支持    |
| Kinetic Kame      | Ubuntu 16.04 LTS | ❌ 已停止支持 |
| ROS 2 Iron Irwini | Ubuntu 22.04 LTS | ✅ 支持    |

### 硬件要求

* **处理器**: 64 位处理器（x86_64 或 ARM64）
* **内存**: 至少 4GB RAM（推荐 8GB 以上）
* **存储空间**: 至少 10GB 可用空间
* **网络**: 稳定的互联网连接

## ROS 版本选择

### 推荐版本: ROS Noetic Ninjemys

**适用系统**: Ubuntu 20.04 LTS
**支持状态**: 长期支持，持续维护
**特点**:

* ROS 1 系列的最新稳定版本
* 广泛的硬件和软件支持
* 学习资源丰富
* 从 ROS 1 迁移到 ROS 2 的成本较低

### 其他版本选择

* **ROS Melodic**: 适用于 Ubuntu 18.04，适合需要兼容性的项目
* **ROS 2 Iron**: 适用于 Ubuntu 22.04，适合新项目开发，支持更多现代特性

## 安装步骤

### 1. 系统准备

#### 1.1 配置软件源
打开 "软件和更新" 设置，确保以下选项被勾选：
- main
- universe
- restricted
- multiverse

#### 1.2 备份并更新软件源列表

```bash
# 备份原始软件源
sudo cp /etc/apt/sources.list /etc/apt/sources.list.backup

# 更新软件包索引
sudo apt update
sudo apt upgrade -y
```

### 2. 添加 ROS 软件源

#### 2.1 官方源（推荐有良好网络连接时使用）

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

#### 2.2 国内镜像源（推荐国内用户使用）

**清华大学源**

```bash
sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
```

**上海交通大学源**

```bash
sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.sjtug.sjtu.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
```

**中国科学技术大学源**

```bash
sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
```

### 3. 添加密钥

```bash
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

如果上述命令失败，可以尝试：

```bash
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

### 4. 更新软件包索引

```bash
sudo apt update
```

### 5. 安装 ROS

#### 5.1 安装完整版（推荐）
包含 ROS、rqt、rviz、机器人通用库、2D/3D 模拟器等

```bash
sudo apt install ros-noetic-desktop-full
```

#### 5.2 安装桌面版
包含 ROS 基础功能、机器人通用函数库、rqt 工具箱和 rviz 可视化工具

```bash
sudo apt install ros-noetic-desktop
```

#### 5.3 安装基础版
仅包含核心功能包、构建工具和通信机制，适合嵌入式系统

```bash
sudo apt install ros-noetic-ros-base
```

#### 5.4 安装特定功能包

```bash
sudo apt install ros-noetic-PACKAGE_NAME
```

### 6. 设置环境变量

#### 6.1 临时设置（当前终端有效）

```bash
source /opt/ros/noetic/setup.bash
```

#### 6.2 永久设置（推荐）

**Bash 用户**

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Zsh 用户**

```bash
echo "source /opt/ros/noetic/setup.zsh" >> ~/.zshrc
source ~/.zshrc
```

### 7. 安装开发工具

```bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

### 8. 初始化 rosdep

#### 8.1 标准方法

```bash
sudo rosdep init
rosdep update
```

#### 8.2 国内优化方法（推荐）
由于网络原因，国内用户推荐使用 rosdepc（中国版本）

```bash
# 安装pip3
sudo apt install python3-pip

# 安装rosdepc
sudo pip3 install rosdepc

# 初始化rosdepc
sudo rosdepc init
rosdepc update
```

## 自动化安装脚本

### 一键安装脚本

我们提供了自动化安装脚本，可以简化安装过程：

```bash
#!/bin/bash

# ROS Noetic 一键安装脚本
echo "开始安装ROS Noetic..."

# 更新系统
sudo apt update && sudo apt upgrade -y

# 安装依赖
sudo apt install -y curl gnupg2 lsb-release

# 添加ROS源（清华源）
sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'

# 添加密钥
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# 更新软件包
sudo apt update

# 安装ROS Noetic完整版
sudo apt install -y ros-noetic-desktop-full

# 设置环境变量
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 安装开发工具
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# 安装并初始化rosdepc
sudo apt install -y python3-pip
sudo pip3 install rosdepc
sudo rosdepc init
rosdepc update

echo "ROS Noetic安装完成！"
echo "请重新打开终端或执行: source ~/.bashrc"
```

### 使用方法

1. 保存上述脚本为 `install_ros.sh`
2. 添加执行权限：`chmod +x install_ros.sh`
3. 运行脚本：`./install_ros.sh`

### 测试验证脚本

我们还提供了测试验证脚本，用于验证安装是否成功：

```bash
#!/bin/bash

# ROS 安装测试脚本
echo "开始测试ROS安装..."

# 检查ROS环境变量
if [ -f "/opt/ros/noetic/setup.bash" ]; then
    echo "✓ ROS环境变量已设置"
else
    echo "✗ ROS环境变量未设置"
    exit 1
fi

# 检查ROS命令
if command -v roscore &> /dev/null; then
    echo "✓ roscore命令可用"
else
    echo "✗ roscore命令不可用"
    exit 1
fi

if command -v rosrun &> /dev/null; then
    echo "✓ rosrun命令可用"
else
    echo "✗ rosrun命令不可用"
    exit 1
fi

if command -v roslaunch &> /dev/null; then
    echo "✓ roslaunch命令可用"
else
    echo "✗ roslaunch命令不可用"
    exit 1
fi

# 检查rosdep配置
if command -v rosdep &> /dev/null; then
    echo "✓ rosdep命令可用"
else
    echo "✗ rosdep命令不可用"
    exit 1
fi

# 启动ROS核心进行测试
echo "正在启动roscore进行测试..."
roscore &
ROSCORE_PID=$!
sleep 3

if ps -p $ROSCORE_PID > /dev/null; then
    echo "✓ roscore启动成功"
    kill $ROSCORE_PID
else
    echo "✗ roscore启动失败"
    exit 1
fi

# 测试turtlesim
echo "正在测试turtlesim..."
if command -v rosrun &> /dev/null && rosrun turtlesim turtlesim_node --help > /dev/null 2>&1; then
    echo "✓ turtlesim功能正常"
else
    echo "⚠ turtlesim可能未安装或功能异常"
fi

echo "ROS安装测试完成！"
```

使用方法：
1. 保存上述脚本为 `test_ros.sh`
2. 添加执行权限：`chmod +x test_ros.sh`
3. 运行脚本：`./test_ros.sh`

## 测试验证

### 验证 ROS 安装

打开新的终端，执行：

```bash
roscore
```

如果显示类似以下信息，说明 ROS 核心服务启动成功：

```
... logging to /home/user/.ros/log/xxxx-xxxx-xxxx-xxxx/roslaunch-xxxx-xxxx.log

Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://localhost:11311/
ros_comm version 1.15.14

SUMMARY
========
PARAMETERS
 * /rosdistro: noetic
 * /rosversion: 1.15.14

NODES

auto-starting new master
process[master]: started with pid [xxxx]
ROS_MASTER_URI=http://localhost:11311/

setting /run_id to xxxx-xxxx-xxxx-xxxx
process[rosout-1]: started with pid [xxxx]
started core service [/rosout]
```

### 运行小海龟示例

1. 保持 roscore 运行，打开新终端：

```bash
rosrun turtlesim turtlesim_node
```

2. 打开第三个终端，运行键盘控制节点：

```bash
rosrun turtlesim turtle_teleop_key
```

3. 在键盘控制终端中，使用方向键控制小海龟移动

### 运行 RViz 可视化工具

```bash
rviz
```

## 常见问题解决

### 1. 网络连接问题

**问题**: 无法连接到 ROS 服务器或下载速度慢

**解决方案**:
* 使用国内镜像源
* 检查网络连接
* 使用代理服务器
* 安装rosdepc替代rosdep

### 2. 密钥添加失败

**问题**: `apt-key` 命令执行失败

**解决方案**:

```bash
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

### 3. rosdep 初始化失败

**问题**: `sudo rosdep init` 失败

**解决方案**:
* 使用 rosdepc 替代
* 检查网络连接
* 手动创建 sources.list.d 文件

### 4. 软件包依赖问题

**问题**: 安装过程中出现依赖错误

**解决方案**:

```bash
sudo apt --fix-broken install
sudo apt autoremove
sudo apt autoclean
```

### 5. 权限问题

**问题**: 执行脚本时出现权限错误

**解决方案**:

```bash
chmod +x script_name.sh
sudo ./script_name.sh
```

### 6. 环境变量问题

**问题**: 找不到ROS命令

**解决方案**:

```bash
source /opt/ros/noetic/setup.bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```

## 示例代码

### 1. 简单的 ROS 节点示例

#### 发布者节点 (examples/talker.py)

```python
#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def talker():
    # 初始化节点
    rospy.init_node('talker', anonymous=True)

    # 创建发布者，发布到chatter话题
    pub = rospy.Publisher('chatter', String, queue_size=10)

    # 设置循环频率
    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        # 创建消息内容
        hello_str = "hello world %s" % rospy.get_time()

        # 发布消息
        pub.publish(hello_str)

        # 打印日志信息
        rospy.loginfo(hello_str)

        # 按照设定频率休眠
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```

#### 订阅者节点 (examples/listener.py)

```python
#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def callback(data):
    # 回调函数，处理接收到的消息
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def listener():
    # 初始化节点
    rospy.init_node('listener', anonymous=True)

    # 创建订阅者，订阅chatter话题
    rospy.Subscriber("chatter", String, callback)

    # 保持节点运行
    rospy.spin()

if __name__ == '__main__':
    listener()
```

### 2. 创建 ROS 功能包

```bash
# 创建工作空间
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make

# 设置环境变量
source devel/setup.bash

# 创建功能包
cd src
catkin_create_pkg beginner_tutorials std_msgs rospy roscpp

# 编译功能包
cd ~/catkin_ws/
catkin_make
```

### 3. 运行示例节点

```bash
# 启动ROS核心
roscore

# 在新终端中运行发布者
rosrun beginner_tutorials talker.py

# 在另一个新终端中运行订阅者
rosrun beginner_tutorials listener.py
```

## 项目结构

```
ROS_Installation_Guide/
├── README.md                      # 项目说明 (本文件)
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
