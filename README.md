# 🤖 ROS 安装指南

<div align="center">

[![GitHub stars](https://img.shields.io/github/stars/matreshka15/ROS_Installation_Guide?style=social)](https://github.com/matreshka15/ROS_Installation_Guide/stargazers)
[![GitHub forks](https://img.shields.io/github/forks/matreshka15/ROS_Installation_Guide?style=social)](https://github.com/matreshka15/ROS_Installation_Guide/network/members)
[![GitHub issues](https://img.shields.io/github/issues/matreshka15/ROS_Installation_Guide)](https://github.com/matreshka15/ROS_Installation_Guide/issues)


</div>

## 🚀 快速开始

欢迎使用 **ROS 安装指南**！这是一个专为机器人开发者设计的一站式 ROS 安装解决方案，支持 ROS 1 和 ROS 2 版本，提供了简洁、高效的安装流程和详细的使用指南。

## 📖 详细指南

为了提供更好的阅读体验，我们将完整的安装指南托管在 GitHub Pages 上：

<div align="center">

[✨ **访问完整安装指南**](https://matreshka15.github.io/ROS_Installation_Guide/)

</div>

## ✨ 核心功能

<div align="center">

| 功能 | 描述 |
|------|------|
| 🎯 **多版本支持** | 同时支持 ROS 1 和 ROS 2，灵活选择适合项目的版本 |
| 🚀 **一键安装** | 提供功能强大的自动化脚本，简化复杂的安装流程 |
| 🌐 **多镜像源** | 支持清华、阿里云、中科大等镜像源，加速下载 |
| 📚 **示例项目** | 自动生成可运行的示例项目，快速上手 ROS 开发 |
| ✅ **测试验证** | 提供详细的安装验证步骤，确保安装成功 |
| ❓ **问题排查** | 收集了常见问题解决方案，快速解决安装难题 |

</div>

## 📁 项目结构

```
ROS_Installation_Guide/
├── 📄 README.md              # 项目说明文档
├── 🚀 install_ros.sh         # 一键安装脚本
├── ✅ test_ros.sh            # 安装测试脚本
├── 🌐 index.html             # GitHub Pages 主页
├── 🎨 css/                   # 样式文件
└── ⚙️ js/                    # JavaScript 功能文件
```

## 📦 快速使用

### 1. 克隆项目

```bash
git clone https://github.com/matreshka15/ROS_Installation_Guide.git
cd ROS_Installation_Guide
```

### 2. 运行安装脚本

```bash
# 安装 ROS 1 Noetic（默认使用清华源，生成示例项目）
./install_ros.sh --ros1

# 安装 ROS 2 Iron（默认使用清华源，生成示例项目）
./install_ros.sh --ros2

# 选择镜像源安装
./install_ros.sh --ros1 --mirror aliyun
./install_ros.sh --ros2 --mirror ustc
```

### 3. 验证安装

```bash
./test_ros.sh
```

## 🎯 支持的版本

| ROS 版本 | Ubuntu 版本 | 支持状态 |
|----------|-------------|----------|
| ROS 1 Noetic | Ubuntu 20.04 LTS | ✅ 推荐 |
| ROS 1 Melodic | Ubuntu 18.04 LTS | ✅ 支持 |
| ROS 2 Iron | Ubuntu 22.04 LTS | ✅ 推荐 |
| ROS 2 Humble | Ubuntu 22.04 LTS | ✅ 支持 |

## 📚 学习资源

<div align="center">

| 资源类型 | 链接 |
|----------|------|
| 📖 **ROS 官方文档** | [ROS Wiki](http://wiki.ros.org/) |
| 🎓 **中文教程** | [鱼香ROS](https://fishros.com/) |
| 👥 **中文社区** | [ROS 中文社区](http://www.rosclub.cn/) |
| 📘 **书籍推荐** | 《ROS机器人开发实践》 |

</div>

## ⭐ 支持本项目

如果这个项目对您有帮助，请不要忘记给我们一个 ⭐ Star！这是对我们最大的支持和鼓励，也能让更多人看到这个项目。

<div align="center">

[⭐ 点击这里 Star 支持我们](https://github.com/matreshka15/ROS_Installation_Guide/)

</div>

## 🤝 贡献指南

欢迎提交 Issue 或 Pull Request 来帮助改进这个项目！我们非常感谢社区的每一份贡献。

### 贡献流程

1. Fork 本项目
2. 创建特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送分支 (`git push origin feature/AmazingFeature`)
5. 打开 Pull Request

## 📄 许可证

本项目采用 MIT 许可证 - 查看 [LICENSE](LICENSE) 文件了解详情。

## 📞 联系方式

- 项目主页：[ROS_Installation_Guide](https://github.com/matreshka15/ROS_Installation_Guide/)
- 报告问题：[Issues](https://github.com/matreshka15/ROS_Installation_Guide/issues)
- 参考项目：[ROS-based-USV-project](https://github.com/matreshka15/ROS-based-USV-project)

---

<div align="center">

**感谢您的支持和使用！** 🎉

[✨ 访问完整安装指南](https://matreshka15.github.io/ROS_Installation_Guide/) | [⭐ Star 支持我们](https://github.com/matreshka15/ROS_Installation_Guide/)

</div>