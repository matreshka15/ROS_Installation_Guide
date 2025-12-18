#!/bin/bash
# ROS 一键安装脚本
# 支持 ROS 1 Noetic 和 ROS 2 Iron
# 功能：换源、安装ROS、配置环境、生成示例项目
# 参考: https://github.com/matreshka15/ROS-based-USV-project

set -e  # 遇到错误立即退出

# 默认配置
ROS_VERSION="ros1"
MIRROR_SOURCE="tsinghua"  # 默认使用清华大学源
generate_example_project="true"  # 默认生成示例项目
install_specific_packages=""  # 安装特定功能包，用空格分隔

# 支持的镜像源列表
MIRROR_SOURCES=("tsinghua" "aliyun" "ustc" "official")

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
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

# 显示帮助信息
show_help() {
    echo "ROS 一键安装脚本使用说明"
    echo ""
    echo "功能：换源、安装ROS、配置环境、生成示例项目"
    echo ""
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "选项:"
    echo "  --ros1                    安装 ROS 1 Noetic (默认)"
    echo "  --ros2                    安装 ROS 2 Iron"
    echo "  --mirror <source>          选择镜像源，支持: tsinghua, aliyun, ustc, official"
    echo "  --no-example              不生成示例项目"
    echo "  --packages <packages>      安装特定功能包，用空格分隔"
    echo "  -h, --help                显示帮助信息"
    echo ""
    echo "示例:"
    echo "  $0 --ros1 --mirror tsinghua           # 使用清华源安装 ROS 1"
    echo "  $0 --ros2 --mirror aliyun            # 使用阿里云源安装 ROS 2"
    echo "  $0 --ros1 --no-example               # 安装 ROS 1 但不生成示例项目"
    echo "  $0 --ros2 --packages turtlesim rviz2  # 安装 ROS 2 并添加特定功能包"
    echo ""
}

# 解析命令行参数
parse_args() {
    while [[ $# -gt 0 ]]; do
        case $1 in
            --ros1)
                ROS_VERSION="ros1"
                shift
                ;;
            --ros2)
                ROS_VERSION="ros2"
                shift
                ;;
            --mirror)
                if [[ $# -gt 1 ]]; then
                    # 检查镜像源是否在支持列表中
                    if [[ " ${MIRROR_SOURCES[@]} " =~ " $2 " ]]; then
                        MIRROR_SOURCE="$2"
                        shift 2
                    else
                        print_error "不支持的镜像源: $2，支持的镜像源: ${MIRROR_SOURCES[*]}"
                        show_help
                        exit 1
                    fi
                else
                    print_error "--mirror 参数缺少镜像源名称"
                    show_help
                    exit 1
                fi
                ;;
            --no-example)
                generate_example_project="false"
                shift
                ;;
            --packages)
                if [[ $# -gt 1 ]]; then
                    install_specific_packages="$2"
                    shift 2
                else
                    print_error "--packages 参数缺少功能包列表"
                    show_help
                    exit 1
                fi
                ;;
            -h|--help)
                show_help
                exit 0
                ;;
            *)
                print_error "未知参数: $1"
                show_help
                exit 1
                ;;
        esac
    done
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
    
    if [ "$ROS_VERSION" = "ros1" ]; then
        # ROS 1 Noetic 推荐 Ubuntu 20.04
        if [ "$DISTRIB_RELEASE" != "20.04" ]; then
            print_warning "推荐使用Ubuntu 20.04 LTS，当前版本: $DISTRIB_RELEASE"
            read -p "是否继续安装？(y/n): " choice
            if [ "$choice" != "y" ] && [ "$choice" != "Y" ]; then
                exit 0
            fi
        fi
    else
        # ROS 2 Iron 推荐 Ubuntu 22.04
        if [ "$DISTRIB_RELEASE" != "22.04" ]; then
            print_warning "推荐使用Ubuntu 22.04 LTS，当前版本: $DISTRIB_RELEASE"
            read -p "是否继续安装？(y/n): " choice
            if [ "$choice" != "y" ] && [ "$choice" != "Y" ]; then
                exit 0
            fi
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

# 获取ROS 1源URL
get_ros1_source_url() {
    local mirror="$1"
    local source_url=""
    
    case $mirror in
        "tsinghua")
            source_url="http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu"
            ;;
        "aliyun")
            source_url="http://mirrors.aliyun.com/ros/ubuntu"
            ;;
        "ustc")
            source_url="http://mirrors.ustc.edu.cn/ros/ubuntu"
            ;;
        "official")
            source_url="http://packages.ros.org/ros/ubuntu"
            ;;
        *)
            source_url="http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu"
            ;;
    esac
    
    echo "$source_url"
}

# 获取ROS 2源URL
get_ros2_source_url() {
    local mirror="$1"
    local source_url=""
    
    case $mirror in
        "tsinghua")
            source_url="http://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu"
            ;;
        "aliyun")
            source_url="http://mirrors.aliyun.com/ros2/ubuntu"
            ;;
        "ustc")
            source_url="http://mirrors.ustc.edu.cn/ros2/ubuntu"
            ;;
        "official")
            source_url="http://packages.ros.org/ros2/ubuntu"
            ;;
        *)
            source_url="http://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu"
            ;;
    esac
    
    echo "$source_url"
}

# 添加ROS源 (ROS 1)
add_ros1_sources() {
    print_info "添加ROS 1软件源..."
    
    # 获取选择的镜像源URL
    local ros1_source_url=$(get_ros1_source_url "$MIRROR_SOURCE")
    print_info "使用镜像源: $MIRROR_SOURCE ($ros1_source_url)"
    
    # 添加ROS 1源
    sudo sh -c '. /etc/lsb-release && echo "deb '$ros1_source_url' `lsb_release -cs` main" > /etc/apt/sources.list.d/ros1-latest.list'
    
    # 添加ROS密钥
    print_info "添加ROS密钥..."
    if ! sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654; then
        print_warning "密钥添加失败，尝试备用方法..."
        curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    fi
    
    print_info "ROS 1源添加完成"
}

# 添加ROS源 (ROS 2)
add_ros2_sources() {
    print_info "添加ROS 2软件源..."
    
    # 获取选择的镜像源URL
    local ros2_source_url=$(get_ros2_source_url "$MIRROR_SOURCE")
    print_info "使用镜像源: $MIRROR_SOURCE ($ros2_source_url)"
    
    # 添加ROS 2源
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] $ros2_source_url $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    print_info "ROS 2源添加完成"
}

# 安装ROS 1
install_ros1() {
    print_info "安装ROS 1 Noetic..."
    
    # 更新软件包索引
    sudo apt update
    
    # 安装ROS Noetic完整版
    sudo apt install -y ros-noetic-desktop-full
    
    # 安装特定功能包（如果有指定）
    if [ -n "$install_specific_packages" ]; then
        print_info "安装指定的ROS 1功能包: $install_specific_packages"
        sudo apt install -y $install_specific_packages
    fi
    
    print_info "ROS 1 Noetic安装完成"
}

# 安装ROS 2
install_ros2() {
    print_info "安装ROS 2 Iron..."
    
    # 更新软件包索引
    sudo apt update
    
    # 安装ROS 2 Iron完整版
    sudo apt install -y ros-iron-desktop-full
    
    # 安装特定功能包（如果有指定）
    if [ -n "$install_specific_packages" ]; then
        print_info "安装指定的ROS 2功能包: $install_specific_packages"
        sudo apt install -y $install_specific_packages
    fi
    
    print_info "ROS 2 Iron安装完成"
}

# 生成ROS 1示例项目
generate_ros1_example_project() {
    print_info "生成ROS 1示例项目..."
    
    # 确保工作空间存在
    if [ ! -d ~/catkin_ws/src ]; then
        mkdir -p ~/catkin_ws/src
    fi
    
    cd ~/catkin_ws/src
    
    # 创建一个简单的ROS 1示例功能包
    print_info "创建ROS 1示例功能包..."
    if [ ! -d beginner_tutorials ]; then
        catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
        
        # 创建talker节点
        cat > beginner_tutorials/src/talker.cpp << 'EOF'
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok()){
    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
EOF
        
        # 创建listener节点
        cat > beginner_tutorials/src/listener.cpp << 'EOF'
#include "ros/ros.h"
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg){
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv){
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  ros::spin();
  return 0;
}
EOF
        
        # 更新CMakeLists.txt
        cat >> beginner_tutorials/CMakeLists.txt << 'EOF'
add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
EOF
        
        print_info "ROS 1示例功能包创建完成"
    else
        print_warning "ROS 1示例功能包已存在，跳过创建"
    fi
    
    # 编译功能包
    print_info "编译ROS 1示例功能包..."
    cd ~/catkin_ws
    catkin_make
    
    print_info "ROS 1示例项目生成完成"
}

# 生成ROS 2示例项目
generate_ros2_example_project() {
    print_info "生成ROS 2示例项目..."
    
    # 确保工作空间存在
    if [ ! -d ~/ros2_ws/src ]; then
        mkdir -p ~/ros2_ws/src
    fi
    
    cd ~/ros2_ws/src
    
    # 创建一个简单的ROS 2示例功能包
    print_info "创建ROS 2示例功能包..."
    if [ ! -d beginner_tutorials ]; then
        ros2 pkg create --build-type ament_cmake beginner_tutorials --dependencies rclcpp std_msgs
        
        # 创建talker节点
        cat > beginner_tutorials/src/talker.cpp << 'EOF'
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node{
public:
  MinimalPublisher() : Node("minimal_publisher"), count_(0){
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback(){
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
EOF
        
        # 创建listener节点
        cat > beginner_tutorials/src/listener.cpp << 'EOF'
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node{
public:
  MinimalSubscriber() : Node("minimal_subscriber"){
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const std_msgs::msg::String & msg) const{
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
EOF
        
        # 更新CMakeLists.txt
        cat >> beginner_tutorials/CMakeLists.txt << 'EOF'
add_executable(talker src/talker.cpp)
target_include_directories(talker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(talker PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  talker
  "rclcpp"
  "std_msgs"
)

add_executable(listener src/listener.cpp)
target_include_directories(listener PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(listener PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  listener
  "rclcpp"
  "std_msgs"
)

install(TARGETS talker listener
  DESTINATION lib/${PROJECT_NAME})
EOF
        
        print_info "ROS 2示例功能包创建完成"
    else
        print_warning "ROS 2示例功能包已存在，跳过创建"
    fi
    
    # 编译功能包
    print_info "编译ROS 2示例功能包..."
    cd ~/ros2_ws
    colcon build
    
    print_info "ROS 2示例项目生成完成"
}

# 配置环境变量 (ROS 1)
configure_ros1_environment() {
    print_info "配置ROS 1环境变量..."
    
    # 检查是否已经配置
    if ! grep -q "source /opt/ros/noetic/setup.bash" ~/.bashrc; then
        echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
        print_info "ROS 1环境变量已添加到 ~/.bashrc"
    else
        print_warning "ROS 1环境变量已存在，跳过配置"
    fi
    
    # 立即生效
    source /opt/ros/noetic/setup.bash
    
    print_info "ROS 1环境变量配置完成"
}

# 配置环境变量 (ROS 2)
configure_ros2_environment() {
    print_info "配置ROS 2环境变量..."
    
    # 检查是否已经配置
    if ! grep -q "source /opt/ros/iron/setup.bash" ~/.bashrc; then
        echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc
        print_info "ROS 2环境变量已添加到 ~/.bashrc"
    else
        print_warning "ROS 2环境变量已存在，跳过配置"
    fi
    
    # 立即生效
    source /opt/ros/iron/setup.bash
    
    print_info "ROS 2环境变量配置完成"
}

# 安装ROS 1开发工具
install_ros1_dev_tools() {
    print_info "安装ROS 1开发工具..."
    
    sudo apt install -y \
        python3-rosdep \
        python3-rosinstall \
        python3-rosinstall-generator \
        python3-wstool \
        build-essential \
        python3-catkin-tools \
        python3-osrf-pycommon
    
    print_info "ROS 1开发工具安装完成"
}

# 安装ROS 2开发工具
install_ros2_dev_tools() {
    print_info "安装ROS 2开发工具..."
    
    sudo apt install -y \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        build-essential \
        python3-osrf-pycommon
    
    print_info "ROS 2开发工具安装完成"
}

# 初始化rosdep
initialize_rosdep() {
    print_info "初始化rosdep..."
    
    # 安装pip3
    if ! command -v pip3 &> /dev/null; then
        print_info "安装pip3..."
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

# 创建ROS 1工作空间
create_ros1_workspace() {
    print_info "创建ROS 1工作空间..."
    
    if [ ! -d ~/catkin_ws ]; then
        mkdir -p ~/catkin_ws/src
        cd ~/catkin_ws
        catkin_make
        
        # 配置工作空间环境变量
        if ! grep -q "source ~/catkin_ws/devel/setup.bash" ~/.bashrc; then
            echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
        fi
        
        print_info "ROS 1工作空间创建完成: ~/catkin_ws"
    else
        print_warning "ROS 1工作空间已存在，跳过创建"
    fi
}

# 创建ROS 2工作空间
create_ros2_workspace() {
    print_info "创建ROS 2工作空间..."
    
    if [ ! -d ~/ros2_ws ]; then
        mkdir -p ~/ros2_ws/src
        cd ~/ros2_ws
        colcon build
        
        # 配置工作空间环境变量
        if ! grep -q "source ~/ros2_ws/install/setup.bash" ~/.bashrc; then
            echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
        fi
        
        print_info "ROS 2工作空间创建完成: ~/ros2_ws"
    else
        print_warning "ROS 2工作空间已存在，跳过创建"
    fi
}

# 安装ROS 1额外工具
install_ros1_additional_tools() {
    print_info "安装ROS 1额外工具..."
    
    # 安装常用的ROS 1工具
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
    
    print_info "ROS 1额外工具安装完成"
}

# 安装ROS 2额外工具
install_ros2_additional_tools() {
    print_info "安装ROS 2额外工具..."
    
    # 安装常用的ROS 2工具
    sudo apt install -y \
        ros-iron-rqt \
        ros-iron-rqt-gui \
        ros-iron-rqt-common-plugins \
        ros-iron-rviz2 \
        ros-iron-turtlesim \
        ros-iron-xacro \
        ros-iron-urdf \
        ros-iron-joint-state-publisher \
        ros-iron-robot-state-publisher \
        ros-iron-demo-nodes-cpp \
        ros-iron-demo-nodes-py
    
    print_info "ROS 2额外工具安装完成"
}

# 安装完成提示 (ROS 1)
ros1_installation_complete() {
    echo -e "\n${GREEN}=====================================${NC}"
    echo -e "${GREEN}ROS 1 Noetic 安装完成！${NC}"
    echo -e "${GREEN}=====================================${NC}\n"
    
    echo -e "${YELLOW}使用说明:${NC}"
    echo "1. 重新打开终端或执行: source ~/.bashrc"
    echo "2. 启动ROS核心: roscore"
    echo "3. 运行小海龟示例: rosrun turtlesim turtlesim_node"
    echo "4. 运行键盘控制: rosrun turtlesim turtle_teleop_key"
    echo "5. 运行RViz: rviz"
    
    echo -e "\n${YELLOW}工作空间位置:${NC} ~/catkin_ws"
    echo -e "${YELLOW}ROS版本:${NC} Noetic Ninjemys (ROS 1)"
    echo -e "${YELLOW}支持系统:${NC} Ubuntu 20.04 LTS"
    
    echo -e "\n${GREEN}祝您ROS开发愉快！${NC}"
}

# 安装完成提示 (ROS 2)
ros2_installation_complete() {
    echo -e "\n${GREEN}=====================================${NC}"
    echo -e "${GREEN}ROS 2 Iron 安装完成！${NC}"
    echo -e "${GREEN}=====================================${NC}\n"
    
    echo -e "${YELLOW}使用说明:${NC}"
    echo "1. 重新打开终端或执行: source ~/.bashrc"
    echo "2. 运行小海龟示例: ros2 run turtlesim turtlesim_node"
    echo "3. 运行键盘控制: ros2 run turtlesim turtle_teleop_key"
    echo "4. 运行RViz: rviz2"
    echo "5. 测试节点通信: ros2 run demo_nodes_cpp talker & ros2 run demo_nodes_py listener"
    
    echo -e "\n${YELLOW}工作空间位置:${NC} ~/ros2_ws"
    echo -e "${YELLOW}ROS版本:${NC} Iron Irwini (ROS 2)"
    echo -e "${YELLOW}支持系统:${NC} Ubuntu 22.04 LTS"
    
    echo -e "\n${GREEN}祝您ROS开发愉快！${NC}"
}

# 安装ROS 1主函数
install_ros1() {
    print_info "开始安装ROS 1 Noetic..."
    
    # 执行安装步骤
    check_system
    backup_sources
    update_system
    install_dependencies
    add_ros1_sources
    install_ros1
    configure_ros1_environment
    install_ros1_dev_tools
    initialize_rosdep
    create_ros1_workspace
    install_ros1_additional_tools
    
    # 生成示例项目（如果启用）
    if [ "$generate_example_project" = "true" ]; then
        generate_ros1_example_project
    fi
    
    ros1_installation_complete
}

# 安装ROS 2主函数
install_ros2() {
    print_info "开始安装ROS 2 Iron..."
    
    # 执行安装步骤
    check_system
    backup_sources
    update_system
    install_dependencies
    add_ros2_sources
    install_ros2
    configure_ros2_environment
    install_ros2_dev_tools
    initialize_rosdep
    create_ros2_workspace
    install_ros2_additional_tools
    
    # 生成示例项目（如果启用）
    if [ "$generate_example_project" = "true" ]; then
        generate_ros2_example_project
    fi
    
    ros2_installation_complete
}

# 主函数
main() {
    echo -e "${GREEN}=====================================${NC}"
    echo -e "${GREEN}ROS 一键安装脚本${NC}"
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
    
    # 解析命令行参数
    parse_args "$@"
    
    # 根据选择的ROS版本执行安装
    if [ "$ROS_VERSION" = "ros1" ]; then
        install_ros1
    elif [ "$ROS_VERSION" = "ros2" ]; then
        install_ros2
    else
        print_error "无效的ROS版本选择: $ROS_VERSION"
        show_help
        exit 1
    fi
}

# 运行主函数
main "$@"