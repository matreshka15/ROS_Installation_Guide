/**
 * ROS Installation Guide - Interactive Features
 * 手把手教程式的交互增强功能
 */

// ==================== 初始化 ====================
document.addEventListener('DOMContentLoaded', function() {
    initializeBackToTop();
    initializeSmoothScroll();
    initializeNavHighlight();
    initializeCodeCopy();
    initializeVersionSelector();
    highlightCode();
});

// ==================== 返回顶部按钮 ====================
function initializeBackToTop() {
    const backBtn = document.getElementById('backToTop');
    if (!backBtn) return;

    window.addEventListener('scroll', () => {
        if (window.scrollY > 300) {
            backBtn.classList.add('show');
        } else {
            backBtn.classList.remove('show');
        }
    });

    backBtn.addEventListener('click', () => {
        window.scrollTo({
            top: 0,
            behavior: 'smooth'
        });
    });
}

// ==================== 平滑滚动 ====================
function initializeSmoothScroll() {
    document.querySelectorAll('a[href^="#"]').forEach(anchor => {
        anchor.addEventListener('click', function(e) {
            e.preventDefault();
            const targetId = this.getAttribute('href');
            const target = document.querySelector(targetId);

            if (target) {
                target.scrollIntoView({
                    behavior: 'smooth',
                    block: 'start'
                });
            }
        });
    });
}

// ==================== 导航高亮 ====================
function initializeNavHighlight() {
    const sections = document.querySelectorAll('.section');
    const navLinks = document.querySelectorAll('.nav-link');

    window.addEventListener('scroll', () => {
        let current = '';

        sections.forEach(section => {
            const sectionTop = section.offsetTop;
            const sectionHeight = section.clientHeight;

            if (window.scrollY >= sectionTop - 200) {
                current = section.getAttribute('id');
            }
        });

        navLinks.forEach(link => {
            link.style.borderBottomColor = 'transparent';
            link.style.color = 'var(--text-secondary)';

            if (link.getAttribute('href') === `#${current}`) {
                link.style.color = 'var(--primary)';
                link.style.borderBottomColor = 'var(--primary)';
            }
        });
    });
}

// ==================== 代码复制功能 ====================
function initializeCodeCopy() {
    document.querySelectorAll('pre').forEach(pre => {
        // 创建复制按钮
        const copyBtn = document.createElement('button');
        copyBtn.className = 'copy-btn';
        copyBtn.textContent = '📋 复制';
        copyBtn.title = '复制代码到剪贴板';

        // 添加点击事件
        copyBtn.addEventListener('click', () => {
            const code = pre.querySelector('code');
            const text = code.textContent;

            // 复制到剪贴板
            navigator.clipboard.writeText(text).then(() => {
                // 显示成功提示
                const originalText = copyBtn.textContent;
                copyBtn.textContent = '✅ 已复制';
                copyBtn.style.background = '#4caf50';

                setTimeout(() => {
                    copyBtn.textContent = originalText;
                    copyBtn.style.background = '#61dafb';
                }, 2000);
            }).catch(err => {
                console.error('复制失败:', err);
                copyBtn.textContent = '❌ 复制失败';
            });
        });

        // 将按钮添加到 pre 中
        pre.style.position = 'relative';
        pre.appendChild(copyBtn);
    });
}

// ==================== 版本选择器 ====================
function initializeVersionSelector() {
    // ROS 兼容性数据
    const compatibility = {
        '18.04': {
            ros: ['melodic'],
            name: 'Ubuntu 18.04 LTS',
            note: '推荐安装 ROS Melodic，这是 ROS 1 系列的稳定版本。'
        },
        '20.04': {
            ros: ['noetic'],
            name: 'Ubuntu 20.04 LTS',
            note: '强烈推荐安装 ROS Noetic，这是 ROS 1 系列的最新版本，长期支持。'
        },
        '22.04': {
            ros: ['iron'],
            name: 'Ubuntu 22.04 LTS',
            note: '推荐安装 ROS 2 Iron，这是 ROS 2 系列的稳定版本。'
        }
    };

    const rosInfo = {
        melodic: {
            name: 'ROS Melodic Morenia',
            type: 'ROS 1',
            desc: '稳定版本，长期支持，适用于 Ubuntu 18.04'
        },
        noetic: {
            name: 'ROS Noetic Ninjemys',
            type: 'ROS 1',
            desc: '最新版本，长期支持，适用于 Ubuntu 20.04（推荐新手使用）'
        },
        iron: {
            name: 'ROS 2 Iron Irwini',
            type: 'ROS 2',
            desc: '现代架构，支持多机器人，适用于 Ubuntu 22.04（推荐新项目）'
        }
    };

    const osSelect = document.getElementById('os-select');
    const rosSelect = document.getElementById('ros-select');
    const recommendation = document.getElementById('recommendation');

    if (!osSelect || !rosSelect) return;

    // 处理 Ubuntu 版本选择变化
    osSelect.addEventListener('change', function() {
        const selectedOS = this.value;

        // 重置 ROS 版本选择
        rosSelect.innerHTML = '<option value="">-- 请选择 ROS 版本 --</option>';

        if (selectedOS && compatibility[selectedOS]) {
            // 启用 ROS 版本选择
            rosSelect.disabled = false;

            // 添加支持的 ROS 版本
            const supportedROS = compatibility[selectedOS].ros;
            supportedROS.forEach(rosVersion => {
                const option = document.createElement('option');
                option.value = rosVersion;
                option.textContent = `${rosInfo[rosVersion].name} (${rosInfo[rosVersion].type})`;
                rosSelect.appendChild(option);
            });

            // 更新推荐信息
            if (recommendation) {
                recommendation.innerHTML = `<strong>📋 ${compatibility[selectedOS].name}</strong><br>${compatibility[selectedOS].note}`;
            }
        } else {
            rosSelect.disabled = true;
            if (recommendation) {
                recommendation.innerHTML = '请选择您的 Ubuntu 版本和想要安装的 ROS 版本，我们会为您提供详细的安装步骤。';
            }
        }

        updateInstallationDisplay();
    });

    // 处理 ROS 版本选择变化
    rosSelect.addEventListener('change', updateInstallationDisplay);

    function updateInstallationDisplay() {
        const selectedROS = rosSelect.value;
        const installSectionTip = document.getElementById('install-select-tip');
        const installNoetic = document.getElementById('install-noetic');
        const installMelodic = document.getElementById('install-melodic');
        const installIron = document.getElementById('install-iron');
        
        // 验证安装部分元素
        const verifySelectTip = document.getElementById('verify-select-tip');
        const verifyROS1 = document.getElementById('verify-ros1');
        const verifyROS2 = document.getElementById('verify-ros2');

        if (installSectionTip) {
            installSectionTip.style.display = selectedROS ? 'none' : 'block';
        }

        // 隐藏所有安装步骤
        if (installNoetic) installNoetic.style.display = 'none';
        if (installMelodic) installMelodic.style.display = 'none';
        if (installIron) installIron.style.display = 'none';

        // 显示选中的安装步骤
        if (selectedROS === 'noetic' && installNoetic) {
            installNoetic.style.display = 'block';
            highlightCode();
        } else if (selectedROS === 'melodic' && installMelodic) {
            installMelodic.style.display = 'block';
            highlightCode();
        } else if (selectedROS === 'iron' && installIron) {
            installIron.style.display = 'block';
            highlightCode();
        }
        
        // 更新验证安装部分
        if (verifySelectTip) {
            verifySelectTip.style.display = selectedROS ? 'none' : 'block';
        }
        
        // 隐藏所有验证步骤
        if (verifyROS1) verifyROS1.style.display = 'none';
        if (verifyROS2) verifyROS2.style.display = 'none';
        
        // 显示对应的验证步骤
        if (selectedROS === 'noetic' || selectedROS === 'melodic') {
            // ROS 1 版本
            if (verifyROS1) verifyROS1.style.display = 'block';
        } else if (selectedROS === 'iron') {
            // ROS 2 版本
            if (verifyROS2) verifyROS2.style.display = 'block';
        }
    }
}

// ==================== 代码高亮 ====================
function highlightCode() {
    if (typeof Prism !== 'undefined') {
        Prism.highlightAll();
    }
}

// ==================== 工具函数 ====================

/**
 * 显示通知消息
 */
function showNotification(message, type = 'info', duration = 3000) {
    const notification = document.createElement('div');
    notification.className = `notification notification-${type}`;
    notification.textContent = message;
    notification.style.cssText = `
        position: fixed;
        top: 20px;
        right: 20px;
        padding: 15px 20px;
        background: ${type === 'success' ? '#4caf50' : '#2196f3'};
        color: white;
        border-radius: 4px;
        box-shadow: 0 4px 12px rgba(0,0,0,0.2);
        z-index: 10000;
        animation: slideIn 0.3s ease;
    `;

    document.body.appendChild(notification);

    setTimeout(() => {
        notification.style.animation = 'slideOut 0.3s ease';
        setTimeout(() => notification.remove(), 300);
    }, duration);
}

/**
 * 检测设备是否支持剪贴板 API
 */
function supportsClipboardAPI() {
    return navigator.clipboard && navigator.clipboard.writeText;
}

/**
 * 获取当前 ROS 版本
 */
function getCurrentROSVersion() {
    const rosSelect = document.getElementById('ros-select');
    return rosSelect ? rosSelect.value : null;
}

/**
 * 滚动到指定元素
 */
function scrollToElement(elementId) {
    const element = document.getElementById(elementId);
    if (element) {
        element.scrollIntoView({ behavior: 'smooth', block: 'start' });
    }
}

// ==================== 导出公共函数 ====================
window.ROSGuide = {
    showNotification,
    supportsClipboardAPI,
    getCurrentROSVersion,
    scrollToElement
};
