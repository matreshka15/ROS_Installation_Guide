// OS and ROS compatibility data
const rosData = {
    osVersions: {
        "ubuntu-18.04": {
            name: "Ubuntu 18.04 LTS (Bionic Beaver)",
            supportedRosVersions: ["melodic"]
        },
        "ubuntu-20.04": {
            name: "Ubuntu 20.04 LTS (Focal Fossa)",
            supportedRosVersions: ["noetic"]
        },
        "ubuntu-22.04": {
            name: "Ubuntu 22.04 LTS (Jammy Jellyfish)",
            supportedRosVersions: ["iron"]
        }
    },
    
    rosVersions: {
        "melodic": {
            name: "ROS Melodic Morenia",
            fullName: "Melodic Morenia",
            supportStatus: "Supported",
            isRos2: false
        },
        "noetic": {
            name: "ROS Noetic Ninjemys",
            fullName: "Noetic Ninjemys",
            supportStatus: "Recommended",
            isRos2: false
        },
        "iron": {
            name: "ROS 2 Iron Irwini",
            fullName: "Iron Irwini",
            supportStatus: "Supported",
            isRos2: true
        }
    },
    
    // Installation steps for each ROS version
    installationSteps: {
        "melodic": [
            {
                title: "System Preparation",
                description: "Prepare your Ubuntu system for ROS installation.",
                content: `
                    <ol>
                        <li>Open "Software and Updates" settings</li>
                        <li>Ensure the following options are enabled:</li>
                        <ul>
                            <li>main</li>
                            <li>universe</li>
                            <li>restricted</li>
                            <li>multiverse</li>
                        </ul>
                    </ol>
                `,
                code: `# Update system packages
sudo apt update
sudo apt upgrade -y`,
                troubleshooting: [
                    {
                        question: "Package update fails with network error",
                        answer: "Check your internet connection and try again. If you're in China, consider using a domestic mirror."
                    }
                ]
            },
            {
                title: "Add ROS Software Sources",
                description: "Add the official ROS repositories to your system.",
                content: `
                    <p>Add the ROS Melodic repository to your system:</p>
                `,
                code: `# Add ROS Melodic repository
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`,
                troubleshooting: [
                    {
                        question: "Repository addition fails",
                        answer: "Try using a domestic mirror instead, such as Tsinghua University's mirror."
                    }
                ]
            },
            {
                title: "Add ROS Keys",
                description: "Add the ROS public keys to authenticate packages.",
                content: `
                    <p>Add the ROS keys to your system:</p>
                `,
                code: `# Add ROS keys
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Alternative method if above fails
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -`,
                troubleshooting: [
                    {
                        question: "Key addition fails",
                        answer: "Check your network connection and ensure you have curl installed."
                    }
                ]
            },
            {
                title: "Update Package Index",
                description: "Update your package index to include ROS packages.",
                content: `
                    <p>Update your package index to include the newly added ROS packages:</p>
                `,
                code: `# Update package index
sudo apt update`,
                troubleshooting: []
            },
            {
                title: "Install ROS Melodic",
                description: "Install the ROS Melodic distribution.",
                content: `
                    <p>Choose one of the following installation options:</p>
                    <ul>
                        <li><strong>Desktop-Full:</strong> ROS, rqt, rviz, robot-generic libraries, 2D/3D simulators, navigation and 2D/3D perception</li>
                        <li><strong>Desktop:</strong> ROS, rqt, rviz, and robot-generic libraries</li>
                        <li><strong>ROS-Base:</strong> (Bare Bones) ROS packaging, build, and communication libraries. No GUI tools.</li>
                    </ul>
                `,
                code: `# Install Desktop-Full (Recommended)
sudo apt install ros-melodic-desktop-full

# Install Desktop version
# sudo apt install ros-melodic-desktop

# Install ROS-Base version
# sudo apt install ros-melodic-ros-base`,
                troubleshooting: [
                    {
                        question: "Package installation fails with dependency errors",
                        answer: "Try running 'sudo apt --fix-broken install' to resolve dependency issues."
                    }
                ]
            },
            {
                title: "Set Up Environment Variables",
                description: "Configure your environment to use ROS commands.",
                content: `
                    <p>Set up your environment variables to use ROS commands:</p>
                `,
                code: `# Set environment variables temporarily (current terminal)
source /opt/ros/melodic/setup.bash

# Set environment variables permanently (recommended)
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# For zsh users
echo "source /opt/ros/melodic/setup.zsh" >> ~/.zshrc
source ~/.zshrc`,
                troubleshooting: [
                    {
                        question: "ROS commands not found after installation",
                        answer: "Make sure you've sourced the setup.bash file or restarted your terminal."
                    }
                ]
            },
            {
                title: "Install Development Tools",
                description: "Install additional tools for ROS development.",
                content: `
                    <p>Install the necessary development tools and dependencies:</p>
                `,
                code: `# Install development tools
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential`,
                troubleshooting: []
            },
            {
                title: "Initialize rosdep",
                description: "Initialize rosdep to manage ROS dependencies.",
                content: `
                    <p>Initialize and update rosdep:</p>
                `,
                code: `# Initialize rosdep
sudo rosdep init
rosdep update

# For Chinese users (recommended alternative)
sudo apt install python-pip
sudo pip install rosdepc
sudo rosdepc init
rosdepc update`,
                troubleshooting: [
                    {
                        question: "rosdep init fails with network error",
                        answer: "Use rosdepc instead, which is optimized for Chinese users."
                    }
                ]
            }
        ],
        "noetic": [
            {
                title: "System Preparation",
                description: "Prepare your Ubuntu system for ROS installation.",
                content: `
                    <ol>
                        <li>Open "Software and Updates" settings</li>
                        <li>Ensure the following options are enabled:</li>
                        <ul>
                            <li>main</li>
                            <li>universe</li>
                            <li>restricted</li>
                            <li>multiverse</li>
                        </ul>
                    </ol>
                `,
                code: `# Update system packages
sudo apt update
sudo apt upgrade -y`,
                troubleshooting: [
                    {
                        question: "Package update fails with network error",
                        answer: "Check your internet connection and try again. If you're in China, consider using a domestic mirror."
                    }
                ]
            },
            {
                title: "Add ROS Software Sources",
                description: "Add the official ROS repositories to your system.",
                content: `
                    <p>Add the ROS Noetic repository to your system:</p>
                `,
                code: `# Add ROS Noetic repository
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`,
                troubleshooting: [
                    {
                        question: "Repository addition fails",
                        answer: "Try using a domestic mirror instead, such as Tsinghua University's mirror."
                    }
                ]
            },
            {
                title: "Add ROS Keys",
                description: "Add the ROS public keys to authenticate packages.",
                content: `
                    <p>Add the ROS keys to your system:</p>
                `,
                code: `# Add ROS keys
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Alternative method if above fails
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -`,
                troubleshooting: [
                    {
                        question: "Key addition fails",
                        answer: "Check your network connection and ensure you have curl installed."
                    }
                ]
            },
            {
                title: "Update Package Index",
                description: "Update your package index to include ROS packages.",
                content: `
                    <p>Update your package index to include the newly added ROS packages:</p>
                `,
                code: `# Update package index
sudo apt update`,
                troubleshooting: []
            },
            {
                title: "Install ROS Noetic",
                description: "Install the ROS Noetic distribution.",
                content: `
                    <p>Choose one of the following installation options:</p>
                    <ul>
                        <li><strong>Desktop-Full:</strong> ROS, rqt, rviz, robot-generic libraries, 2D/3D simulators, navigation and 2D/3D perception</li>
                        <li><strong>Desktop:</strong> ROS, rqt, rviz, and robot-generic libraries</li>
                        <li><strong>ROS-Base:</strong> (Bare Bones) ROS packaging, build, and communication libraries. No GUI tools.</li>
                    </ul>
                `,
                code: `# Install Desktop-Full (Recommended)
sudo apt install ros-noetic-desktop-full

# Install Desktop version
# sudo apt install ros-noetic-desktop

# Install ROS-Base version
# sudo apt install ros-noetic-ros-base`,
                troubleshooting: [
                    {
                        question: "Package installation fails with dependency errors",
                        answer: "Try running 'sudo apt --fix-broken install' to resolve dependency issues."
                    }
                ]
            },
            {
                title: "Set Up Environment Variables",
                description: "Configure your environment to use ROS commands.",
                content: `
                    <p>Set up your environment variables to use ROS commands:</p>
                `,
                code: `# Set environment variables temporarily (current terminal)
source /opt/ros/noetic/setup.bash

# Set environment variables permanently (recommended)
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# For zsh users
echo "source /opt/ros/noetic/setup.zsh" >> ~/.zshrc
source ~/.zshrc`,
                troubleshooting: [
                    {
                        question: "ROS commands not found after installation",
                        answer: "Make sure you've sourced the setup.bash file or restarted your terminal."
                    }
                ]
            },
            {
                title: "Install Development Tools",
                description: "Install additional tools for ROS development.",
                content: `
                    <p>Install the necessary development tools and dependencies:</p>
                `,
                code: `# Install development tools
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential`,
                troubleshooting: []
            },
            {
                title: "Initialize rosdep",
                description: "Initialize rosdep to manage ROS dependencies.",
                content: `
                    <p>Initialize and update rosdep:</p>
                `,
                code: `# Initialize rosdep
sudo rosdep init
rosdep update

# For Chinese users (recommended alternative)
sudo apt install python3-pip
sudo pip3 install rosdepc
sudo rosdepc init
rosdepc update`,
                troubleshooting: [
                    {
                        question: "rosdep init fails with network error",
                        answer: "Use rosdepc instead, which is optimized for Chinese users."
                    }
                ]
            }
        ],
        "iron": [
            {
                title: "System Preparation",
                description: "Prepare your Ubuntu system for ROS 2 installation.",
                content: `
                    <ol>
                        <li>Open "Software and Updates" settings</li>
                        <li>Ensure the following options are enabled:</li>
                        <ul>
                            <li>main</li>
                            <li>universe</li>
                            <li>restricted</li>
                            <li>multiverse</li>
                        </ul>
                    </ol>
                `,
                code: `# Update system packages
sudo apt update
sudo apt upgrade -y`,
                troubleshooting: [
                    {
                        question: "Package update fails with network error",
                        answer: "Check your internet connection and try again. If you're in China, consider using a domestic mirror."
                    }
                ]
            },
            {
                title: "Add ROS 2 Software Sources",
                description: "Add the official ROS 2 repositories to your system.",
                content: `
                    <p>Add the ROS 2 Iron repository to your system:</p>
                `,
                code: `# Add ROS 2 GPG key
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null`,
                troubleshooting: [
                    {
                        question: "Repository addition fails",
                        answer: "Try using a domestic mirror instead, such as Tsinghua University's mirror."
                    }
                ]
            },
            {
                title: "Update Package Index",
                description: "Update your package index to include ROS 2 packages.",
                content: `
                    <p>Update your package index to include the newly added ROS 2 packages:</p>
                `,
                code: `# Update package index
sudo apt update`,
                troubleshooting: []
            },
            {
                title: "Install ROS 2 Iron",
                description: "Install the ROS 2 Iron distribution.",
                content: `
                    <p>Choose one of the following installation options:</p>
                    <ul>
                        <li><strong>Desktop:</strong> ROS 2, rqt, rviz, demos, tutorials</li>
                        <li><strong>ROS-Base:</strong> (Bare Bones) ROS 2 packaging, build, and communication libraries. No GUI tools.</li>
                    </ul>
                `,
                code: `# Install Desktop version (Recommended)
sudo apt install ros-iron-desktop

# Install ROS-Base version
# sudo apt install ros-iron-ros-base`,
                troubleshooting: [
                    {
                        question: "Package installation fails with dependency errors",
                        answer: "Try running 'sudo apt --fix-broken install' to resolve dependency issues."
                    }
                ]
            },
            {
                title: "Set Up Environment Variables",
                description: "Configure your environment to use ROS 2 commands.",
                content: `
                    <p>Set up your environment variables to use ROS 2 commands:</p>
                `,
                code: `# Set environment variables temporarily (current terminal)
source /opt/ros/iron/setup.bash

# Set environment variables permanently (recommended)
echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc
source ~/.bashrc

# For zsh users
echo "source /opt/ros/iron/setup.zsh" >> ~/.zshrc
source ~/.zshrc`,
                troubleshooting: [
                    {
                        question: "ROS 2 commands not found after installation",
                        answer: "Make sure you've sourced the setup.bash file or restarted your terminal."
                    }
                ]
            },
            {
                title: "Install Development Tools",
                description: "Install additional tools for ROS 2 development.",
                content: `
                    <p>Install the necessary development tools and dependencies:</p>
                `,
                code: `# Install development tools and ROS tools
sudo apt install ros-dev-tools`,
                troubleshooting: []
            },
            {
                title: "Install rosdep",
                description: "Install rosdep to manage ROS 2 dependencies.",
                content: `
                    <p>Install and initialize rosdep:</p>
                `,
                code: `# Install rosdep
sudo apt install python3-rosdep

# Initialize rosdep
sudo rosdep init
rosdep update

# For Chinese users (recommended alternative)
sudo apt install python3-pip
sudo pip3 install rosdepc
sudo rosdepc init
rosdepc update`,
                troubleshooting: [
                    {
                        question: "rosdep init fails with network error",
                        answer: "Use rosdepc instead, which is optimized for Chinese users."
                    }
                ]
            },
            {
                title: "Test ROS 2 Installation",
                description: "Verify your ROS 2 installation with a simple test.",
                content: `
                    <p>Test your ROS 2 installation by running a simple publisher and subscriber:</p>
                `,
                code: `# In one terminal, run the talker node
source /opt/ros/iron/setup.bash
ros2 run demo_nodes_cpp talker

# In another terminal, run the listener node
source /opt/ros/iron/setup.bash
ros2 run demo_nodes_cpp listener`,
                troubleshooting: [
                    {
                        question: "Nodes can't communicate with each other",
                        answer: "Make sure you've sourced the setup.bash file in both terminals."
                    }
                ]
            }
        ]
    },
    
    // Verification steps for each ROS version
    verificationSteps: {
        "melodic": [
            {
                title: "Check ROS Environment",
                description: "Verify that ROS environment variables are set correctly.",
                code: `# Check ROS environment variables
echo $ROS_DISTRO
echo $ROS_ROOT`
            },
            {
                title: "Start roscore",
                description: "Test if roscore starts successfully.",
                code: `# Start roscore
roscore`
            },
            {
                title: "Test turtlesim",
                description: "Test the turtlesim package.",
                code: `# In one terminal, run roscore
roscore

# In another terminal, run turtlesim
rosrun turtlesim turtlesim_node

# In a third terminal, run the turtle teleop
rosrun turtlesim turtle_teleop_key`
            }
        ],
        "noetic": [
            {
                title: "Check ROS Environment",
                description: "Verify that ROS environment variables are set correctly.",
                code: `# Check ROS environment variables
echo $ROS_DISTRO
echo $ROS_ROOT`
            },
            {
                title: "Start roscore",
                description: "Test if roscore starts successfully.",
                code: `# Start roscore
roscore`
            },
            {
                title: "Test turtlesim",
                description: "Test the turtlesim package.",
                code: `# In one terminal, run roscore
roscore

# In another terminal, run turtlesim
rosrun turtlesim turtlesim_node

# In a third terminal, run the turtle teleop
rosrun turtlesim turtle_teleop_key`
            }
        ],
        "iron": [
            {
                title: "Check ROS 2 Environment",
                description: "Verify that ROS 2 environment variables are set correctly.",
                code: `# Check ROS 2 environment variables
echo $ROS_DISTRO
echo $ROS_ROOT`
            },
            {
                title: "Test talker/listener",
                description: "Test the demo nodes.",
                code: `# In one terminal, run the talker
ros2 run demo_nodes_cpp talker

# In another terminal, run the listener
ros2 run demo_nodes_cpp listener`
            },
            {
                title: "Test turtlesim",
                description: "Test the turtlesim package.",
                code: `# In one terminal, run turtlesim
ros2 run turtlesim turtlesim_node

# In another terminal, run the turtle teleop
ros2 run turtlesim turtle_teleop_key`
            }
        ]
    }
};

// Export data for use in other files
if (typeof module !== 'undefined' && module.exports) {
    module.exports = rosData;
}