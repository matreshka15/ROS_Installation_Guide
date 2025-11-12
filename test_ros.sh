#!/bin/bash
# ROS安装测试脚本
# 验证ROS是否正确安装并能正常运行

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

# 检查ROS环境
check_ros_environment() {
    print_info "检查ROS环境..."
    
    # 检查ROS_ROOT
    if [ -z "$ROS_ROOT" ]; then
        print_error "ROS环境变量未设置"
        print_warning "请确保已执行: source /opt/ros/noetic/setup.bash"
        exit 1
    fi
    
    # 检查ROS版本
    if ! command -v roscore &> /dev/null; then
        print_error "roscore命令未找到，ROS可能未正确安装"
        exit 1
    fi
    
    print_success "ROS环境检查通过"
    print_info "ROS版本: $(rosversion -d)"
}

# 检查rosdep
check_rosdep() {
    print_info "检查rosdep..."
    
    if command -v rosdepc &> /dev/null; then
        print_info "找到rosdepc (国内优化版)"
    elif command -v rosdep &> /dev/null; then
        print_info "找到rosdep (标准版)"
    else
        print_warning "rosdep未找到，某些功能可能受限"
    fi
    
    print_success "rosdep检查完成"
}

# 检查工作空间
check_workspace() {
    print_info "检查ROS工作空间..."
    
    if [ -d ~/catkin_ws ]; then
        print_info "找到工作空间: ~/catkin_ws"
        
        # 检查是否已经编译
        if [ -d ~/catkin_ws/devel ]; then
            print_info "工作空间已编译"
        else
            print_warning "工作空间尚未编译，建议执行: cd ~/catkin_ws && catkin_make"
        fi
    else
        print_warning "未找到工作空间，建议创建: mkdir -p ~/catkin_ws/src"
    fi
    
    print_success "工作空间检查完成"
}

# 测试roscore
test_roscore() {
    print_info "测试roscore启动..."
    
    # 检查是否已有roscore在运行
    if pgrep -x "roscore" > /dev/null; then
        print_warning "检测到roscore已在运行，跳过测试"
        return
    fi
    
    # 启动roscore并在后台运行
    roscore &
    ROSCORE_PID=$!
    
    # 等待roscore启动
    print_info "等待roscore启动..."
    sleep 10
    
    # 检查roscore是否仍在运行
    if ps -p $ROSCORE_PID > /dev/null; then
        print_success "roscore启动成功"
        
        # 关闭roscore
        kill $ROSCORE_PID
        wait $ROSCORE_PID 2>/dev/null
        print_info "roscore已关闭"
    else
        print_error "roscore启动失败"
        exit 1
    fi
}

# 测试turtlesim
test_turtlesim() {
    print_info "测试turtlesim功能..."
    
    # 检查turtlesim是否安装
    if ! dpkg -l | grep -q ros-noetic-turtlesim; then
        print_error "turtlesim未安装，正在安装..."
        sudo apt install -y ros-noetic-turtlesim
    fi
    
    print_info "turtlesim安装状态正常"
    print_success "turtlesim测试准备完成"
}

# 显示使用说明
show_usage() {
    echo -e "\n${GREEN}=====================================${NC}"
    echo -e "${GREEN}ROS安装测试完成！${NC}"
    echo -e "${GREEN}=====================================${NC}\n"
    
    echo -e "${YELLOW}推荐测试步骤:${NC}"
    echo "1. 启动ROS核心服务:"
    echo "   roscore"
    echo ""
    echo "2. 在新终端中启动小海龟模拟器:"
    echo "   rosrun turtlesim turtlesim_node"
    echo ""
    echo "3. 在另一个新终端中启动键盘控制:"
    echo "   rosrun turtlesim turtle_teleop_key"
    echo ""
    echo "4. 使用方向键控制小海龟移动"
    echo ""
    echo "5. 启动RViz可视化工具:"
    echo "   rviz"
    echo ""
    echo -e "${YELLOW}如果所有命令都能正常执行，说明ROS安装成功！${NC}"
}

# 主函数
main() {
    echo -e "${GREEN}=====================================${NC}"
    echo -e "${GREEN}ROS安装测试脚本${NC}"
    echo -e "${GREEN}=====================================${NC}\n"
    
    # 检查是否以root用户运行
    if [ "$(id -u)" -eq 0 ]; then
        print_error "请勿以root用户运行此脚本"
        exit 1
    fi
    
    # 执行测试步骤
    check_ros_environment
    check_rosdep
    check_workspace
    test_roscore
    test_turtlesim
    
    # 显示使用说明
    show_usage
    
    echo -e "\n${GREEN}所有测试完成！${NC}"
}

# 运行主函数
main "$@"