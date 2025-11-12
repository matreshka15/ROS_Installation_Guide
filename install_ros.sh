#!/bin/bash
# ROS Noetic 一键安装脚本
# 适用于 Ubuntu 20.04 LTS
# 参考: https://github.com/matreshka15/ROS-based-USV-project

set -e  # 遇到错误立即退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 函数定义
print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查系统版本
check_system() {
    print_info "检查系统版本..."
    
    if [ ! -f /etc/lsb-release ]; then
        print_error "不支持的操作系统，仅支持Ubuntu系统"
        exit 1
    fi
    
    source /etc/lsb-release
    
    if [ "$DISTRIB_ID" != "Ubuntu" ]; then
        print_error "仅支持Ubuntu系统，当前系统: $DISTRIB_ID"
        exit 1
    fi
    
    if [ "$DISTRIB_RELEASE" != "20.04" ]; then
        print_warning "推荐使用Ubuntu 20.04 LTS，当前版本: $DISTRIB_RELEASE"
        read -p "是否继续安装？(y/n): " choice
        if [ "$choice" != "y" ] && [ "$choice" != "Y" ]; then
            exit 0
        fi
    fi
    
    print_info "系统版本检查通过"
}

# 备份软件源
backup_sources() {
    print_info "备份软件源列表..."
    if [ ! -f /etc/apt/sources.list.backup ]; then
        sudo cp /etc/apt/sources.list /etc/apt/sources.list.backup
        print_info "软件源已备份到 /etc/apt/sources.list.backup"
    else
        print_warning "软件源备份已存在，跳过备份"
    fi
}

# 更新系统
update_system() {
    print_info "更新系统软件包..."
    sudo apt update
    sudo apt upgrade -y
    print_info "系统更新完成"
}

# 安装依赖
install_dependencies() {
    print_info "安装必要依赖..."
    sudo apt install -y \
        curl \
        gnupg2 \
        lsb-release \
        software-properties-common \
        apt-transport-https \
        ca-certificates
    
    print_info "依赖安装完成"
}

# 添加ROS源
add_ros_sources() {
    print_info "添加ROS软件源..."
    
    # 添加清华ROS源
    sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
    
    # 添加ROS密钥
    print_info "添加ROS密钥..."
    if ! sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654; then
        print_warning "密钥添加失败，尝试备用方法..."
        curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    fi
    
    print_info "ROS源添加完成"
}

# 安装ROS
install_ros() {
    print_info "安装ROS Noetic..."
    
    # 更新软件包索引
    sudo apt update
    
    # 安装ROS Noetic完整版
    sudo apt install -y ros-noetic-desktop-full
    
    print_info "ROS Noetic安装完成"
}

# 配置环境变量
configure_environment() {
    print_info "配置环境变量..."
    
    # 检查是否已经配置
    if ! grep -q "source /opt/ros/noetic/setup.bash" ~/.bashrc; then
        echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
        print_info "环境变量已添加到 ~/.bashrc"
    else
        print_warning "环境变量已存在，跳过配置"
    fi
    
    # 立即生效
    source /opt/ros/noetic/setup.bash
    
    print_info "环境变量配置完成"
}

# 安装开发工具
install_dev_tools() {
    print_info "安装ROS开发工具..."
    
    sudo apt install -y \
        python3-rosdep \
        python3-rosinstall \
        python3-rosinstall-generator \
        python3-wstool \
        build-essential \
        python3-catkin-tools \
        python3-osrf-pycommon
    
    print_info "开发工具安装完成"
}

# 初始化rosdep
initialize_rosdep() {
    print_info "初始化rosdep..."
    
    # 安装pip3
    if ! command -v pip3 &> /dev/null; then
        sudo apt install -y python3-pip
    fi
    
    # 安装rosdepc（国内优化版）
    if ! command -v rosdepc &> /dev/null; then
        print_info "安装rosdepc（国内优化版）..."
        sudo pip3 install rosdepc
    fi
    
    # 初始化rosdepc
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        sudo rosdepc init
    else
        print_warning "rosdepc已初始化，跳过"
    fi
    
    # 更新rosdepc
    rosdepc update
    
    print_info "rosdep初始化完成"
}

# 创建工作空间
create_workspace() {
    print_info "创建ROS工作空间..."
    
    if [ ! -d ~/catkin_ws ]; then
        mkdir -p ~/catkin_ws/src
        cd ~/catkin_ws
        catkin_make
        
        # 配置工作空间环境变量
        if ! grep -q "source ~/catkin_ws/devel/setup.bash" ~/.bashrc; then
            echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
        fi
        
        print_info "工作空间创建完成: ~/catkin_ws"
    else
        print_warning "工作空间已存在，跳过创建"
    fi
}

# 安装额外工具
install_additional_tools() {
    print_info "安装额外工具..."
    
    # 安装常用的ROS工具
    sudo apt install -y \
        ros-noetic-rqt \
        ros-noetic-rqt-gui \
        ros-noetic-rqt-common-plugins \
        ros-noetic-rviz \
        ros-noetic-turtlesim \
        ros-noetic-xacro \
        ros-noetic-urdf \
        ros-noetic-joint-state-publisher \
        ros-noetic-robot-state-publisher
    
    print_info "额外工具安装完成"
}

# 安装完成提示
installation_complete() {
    echo -e "\n${GREEN}=====================================${NC}"
    echo -e "${GREEN}ROS Noetic 安装完成！${NC}"
    echo -e "${GREEN}=====================================${NC}\n"
    
    echo -e "${YELLOW}使用说明:${NC}"
    echo "1. 重新打开终端或执行: source ~/.bashrc"
    echo "2. 启动ROS核心: roscore"
    echo "3. 运行小海龟示例: rosrun turtlesim turtlesim_node"
    echo "4. 运行键盘控制: rosrun turtlesim turtle_teleop_key"
    echo "5. 运行RViz: rviz"
    
    echo -e "\n${YELLOW}工作空间位置:${NC} ~/catkin_ws"
    echo -e "${YELLOW}ROS版本:${NC} Noetic Ninjemys"
    echo -e "${YELLOW}支持系统:${NC} Ubuntu 20.04 LTS"
    
    echo -e "\n${GREEN}祝您ROS开发愉快！${NC}"
}

# 主函数
main() {
    echo -e "${GREEN}=====================================${NC}"
    echo -e "${GREEN}ROS Noetic 一键安装脚本${NC}"
    echo -e "${GREEN}参考: https://github.com/matreshka15/ROS-based-USV-project${NC}"
    echo -e "${GREEN}=====================================${NC}\n"
    
    # 检查是否以root用户运行
    if [ "$(id -u)" -eq 0 ]; then
        print_error "请勿以root用户运行此脚本，请使用普通用户并确保有sudo权限"
        exit 1
    fi
    
    # 检查sudo权限
    if ! sudo -v &> /dev/null; then
        print_error "当前用户没有sudo权限，请联系系统管理员"
        exit 1
    fi
    
    # 执行安装步骤
    check_system
    backup_sources
    update_system
    install_dependencies
    add_ros_sources
    install_ros
    configure_environment
    install_dev_tools
    initialize_rosdep
    create_workspace
    install_additional_tools
    
    installation_complete
}

# 运行主函数
main "$@"