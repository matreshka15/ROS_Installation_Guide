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



* **处理器**: 64 位处理器（x86\_64 或 ARM64）

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

main

universe

restricted

multiverse

#### 1.2 备份并更新软件源列表



```
\# 备份原始软件源

sudo cp /etc/apt/sources.list /etc/apt/sources.list.backup

\# 更新软件包索引

sudo apt update

sudo apt upgrade -y
```

### 2. 添加 ROS 软件源

#### 2.1 官方源（推荐有良好网络连接时使用）



```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu \$(lsb\_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

#### 2.2 国内镜像源（推荐国内用户使用）

**清华大学源**



```
sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ \`lsb\_release -cs\` main" > /etc/apt/sources.list.d/ros-latest.list'
```

**上海交通大学源**



```
sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.sjtug.sjtu.edu.cn/ros/ubuntu/ \`lsb\_release -cs\` main" > /etc/apt/sources.list.d/ros-latest.list'
```

**中国科学技术大学源**



```
sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ \`lsb\_release -cs\` main" > /etc/apt/sources.list.d/ros-latest.list'
```

### 3. 添加密钥



```
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

如果上述命令失败，可以尝试：



```
sudo apt install curl

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

### 4. 更新软件包索引



```
sudo apt update
```

### 5. 安装 ROS

#### 5.1 安装完整版（推荐）

包含 ROS、rqt、rviz、机器人通用库、2D/3D 模拟器等



```
sudo apt install ros-noetic-desktop-full
```

#### 5.2 安装桌面版

包含 ROS 基础功能、机器人通用函数库、rqt 工具箱和 rviz 可视化工具



```
sudo apt install ros-noetic-desktop
```

#### 5.3 安装基础版

仅包含核心功能包、构建工具和通信机制，适合嵌入式系统



```
sudo apt install ros-noetic-ros-base
```

#### 5.4 安装特定功能包



```
sudo apt install ros-noetic-PACKAGE\_NAME
```

### 6. 设置环境变量

#### 6.1 临时设置（当前终端有效）



```
source /opt/ros/noetic/setup.bash
```

#### 6.2 永久设置（推荐）

**Bash 用户**



```
echo "source /opt/ros/noetic/setup.bash" >> \~/.bashrc

source \~/.bashrc
```

**Zsh 用户**



```
echo "source /opt/ros/noetic/setup.zsh" >> \~/.zshrc

source \~/.zshrc
```

### 7. 安装开发工具



```
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

### 8. 初始化 rosdep

#### 8.1 标准方法



```
sudo rosdep init

rosdep update
```

#### 8.2 国内优化方法（推荐）

由于网络原因，国内用户推荐使用 rosdepc（China 版本）



```
\# 安装pip3

sudo apt install python3-pip

\# 安装rosdepc

sudo pip3 install rosdepc

\# 初始化rosdepc

sudo rosdepc init

rosdepc update
```

## 自动化安装脚本

### 一键安装脚本

我们提供了自动化安装脚本，可以简化安装过程：



```
\#!/bin/bash

\# ROS Noetic 一键安装脚本

echo "开始安装ROS Noetic..."

\# 更新系统

sudo apt update && sudo apt upgrade -y

\# 安装依赖

sudo apt install -y curl gnupg2 lsb-release

\# 添加ROS源（清华源）

sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ \`lsb\_release -cs\` main" > /etc/apt/sources.list.d/ros-latest.list'

\# 添加密钥

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

\# 更新软件包

sudo apt update

\# 安装ROS Noetic完整版

sudo apt install -y ros-noetic-desktop-full

\# 设置环境变量

echo "source /opt/ros/noetic/setup.bash" >> \~/.bashrc

source \~/.bashrc

\# 安装开发工具

sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

\# 安装并初始化rosdepc

sudo apt install -y python3-pip

sudo pip3 install rosdepc

sudo rosdepc init

rosdepc update

echo "ROS Noetic安装完成！"

echo "请重新打开终端或执行: source \~/.bashrc"
```

### 使用方法



1. 保存上述脚本为 `install_ros.sh`

2. 添加执行权限：`chmod +x install_ros.sh`

3. 运行脚本：`./install_ros.sh`

## 测试验证

### 验证 ROS 安装

打开新的终端，执行：



```
roscore
```

如果显示类似以下信息，说明 ROS 核心服务启动成功：



```
... logging to /home/user/.ros/log/xxxx-xxxx-xxxx-xxxx/roslaunch-xxxx-xxxx.log

Checking log directory for disk usage. This may take a while.

Press Ctrl-C to interrupt

Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://localhost:11311/

ros\_comm version 1.15.14

SUMMARY

\========

PARAMETERS

&#x20;\* /rosdistro: noetic

&#x20;\* /rosversion: 1.15.14

NODES

auto-starting new master

process\[master]: started with pid \[xxxx]

ROS\_MASTER\_URI=http://localhost:11311/

setting /run\_id to xxxx-xxxx-xxxx-xxxx

process\[rosout-1]: started with pid \[xxxx]

started core service \[/rosout]
```

### 运行小海龟示例



1. 保持 roscore 运行，打开新终端：



```
rosrun turtlesim turtlesim\_node
```



1. 打开第三个终端，运行键盘控制节点：



```
rosrun turtlesim turtle\_teleop\_key
```



1. 在键盘控制终端中，使用方向键控制小海龟移动

### 运行 RViz 可视化工具



```
rviz
```

## 常见问题解决

### 1. 网络连接问题

**问题**: 无法连接到 ROS 服务器或下载速度慢

**解决方案**:



* 使用国内镜像源

* 检查网络连接

* 使用代理服务器

### 2. 密钥添加失败

**问题**: `apt-key` 命令执行失败

**解决方案**:



```
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



```
sudo apt --fix-broken install

sudo apt autoremove

sudo apt autoclean
```

## 示例代码

### 1. 简单的 ROS 节点示例

#### 发布者节点 (talker.py)



```
\#!/usr/bin/env python3

import rospy

from std\_msgs.msg import String

def talker():

&#x20;   \# 初始化节点

&#x20;   rospy.init\_node('talker', anonymous=True)

&#x20;  &#x20;

&#x20;   \# 创建发布者，发布到chatter话题

&#x20;   pub = rospy.Publisher('chatter', String, queue\_size=10)

&#x20;  &#x20;

&#x20;   \# 设置循环频率

&#x20;   rate = rospy.Rate(10)  # 10Hz

&#x20;  &#x20;

&#x20;   while not rospy.is\_shutdown():

&#x20;       \# 创建消息内容

&#x20;       hello\_str = "hello world %s" % rospy.get\_time()

&#x20;      &#x20;

&#x20;       \# 发布消息

&#x20;       pub.publish(hello\_str)

&#x20;      &#x20;

&#x20;       \# 打印日志信息

&#x20;       rospy.loginfo(hello\_str)

&#x20;      &#x20;

&#x20;       \# 按照设定频率休眠

&#x20;       rate.sleep()

if \_\_name\_\_ == '\_\_main\_\_':

&#x20;   try:

&#x20;       talker()

&#x20;   except rospy.ROSInterruptException:

&#x20;       pass
```

#### 订阅者节点 (listener.py)



```
\#!/usr/bin/env python3

import rospy

from std\_msgs.msg import String

def callback(data):

&#x20;   \# 回调函数，处理接收到的消息

&#x20;   rospy.loginfo(rospy.get\_caller\_id() + "I heard %s", data.data)

def listener():

&#x20;   \# 初始化节点

&#x20;   rospy.init\_node('listener', anonymous=True)

&#x20;  &#x20;

&#x20;   \# 创建订阅者，订阅chatter话题

&#x20;   rospy.Subscriber("chatter", String, callback)

&#x20;  &#x20;

&#x20;   \# 保持节点运行

&#x20;   rospy.spin()

if \_\_name\_\_ == '\_\_main\_\_':

&#x20;   listener()
```

### 2. 创建 ROS 功能包



```
\# 创建工作空间

mkdir -p \~/catkin\_ws/src

cd \~/catkin\_ws/

catkin\_make

\# 设置环境变量

source devel/setup.bash

\# 创建功能包

cd src

catkin\_create\_pkg beginner\_tutorials std\_msgs rospy roscpp

\# 编译功能包

cd \~/catkin\_ws/

catkin\_make
```

### 3. 运行示例节点



```
\# 启动ROS核心

roscore

\# 在新终端中运行发布者

rosrun beginner\_tutorials talker.py

\# 在另一个新终端中运行订阅者

rosrun beginner\_tutorials listener.py
```

## 参考资料

### 官方文档



* [ROS 官方网站](http://www.ros.org/)

* [ROS Noetic 官方文档](http://wiki.ros.org/noetic)

* [ROS 安装指南](http://wiki.ros.org/ROS/Installation)

### 国内资源



* [鱼香 ROS](https://fishros.com/)

* [ROS 中文社区](http://www.rosclub.cn/)

### 学习资源



* [ROS 入门教程](http://wiki.ros.org/ROS/Tutorials)

* [ROS By Example](https://github.com/pirobot/ros-by-example)

* [A Gentle Introduction to ROS](https://www.cse.sc.edu/~jokane/agitr/)

### 相关项目



* [参考项目: ROS-based-USV-project](https://github.com/matreshka15/ROS-based-USV-project)

* [ROS 官方示例项目](https://github.com/ros)



***

**注意**: 本指南基于 Ubuntu 20.04 和 ROS Noetic 编写。对于其他版本的 Ubuntu 和 ROS，请参考相应的官方文档进行调整。

**维护者**: ROS 开发团队

**最后更新**: 2025 年 11 月 12 日

> （注：文档部分内容可能由 AI 生成）