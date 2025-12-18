// Main application logic for interactive ROS Installation Guide

// Translation data
const translations = {
    zh: {
        title: "交互式 ROS 安装指南",
        subtitle: "在 Ubuntu 上安装 Robot Operating System 的分步说明",
        selectConfiguration: "选择您的配置",
        linuxDistribution: "Linux 发行版",
        selectUbuntuVersion: "-- 选择 Ubuntu 版本 --",
        rosDistribution: "ROS 发行版",
        selectRosVersion: "-- 选择 ROS 版本 --",
        startInstallation: "开始安装",
        previous: "上一步",
        next: "下一步",
        verificationComplete: "验证完成！",
        installationSuccessful: "ROS 安装成功！",
        successMessage: "您已成功在系统上安装了 ROS。以下是一些开始使用的建议：",
        tryTurtlesim: "尝试运行 turtlesim 示例：<code>rosrun turtlesim turtlesim_node</code>",
        exploreTutorials: "探索 ROS 教程：<a href='http://wiki.ros.org/ROS/Tutorials' target='_blank'>ROS Wiki</a>",
        joinCommunity: "加入 ROS 社区：<a href='https://discourse.ros.org/' target='_blank'>ROS Discourse</a>",
        step: "步骤 {{current}} / {{total}}",
        finish: "完成",
        troubleshootingTips: "故障排除提示",
        copy: "复制",
        copied: "已复制！",
        compatible: "✅ 兼容：{{os}} 与 {{ros}}",
        incompatible: "❌ 不兼容：{{os}} 与 {{ros}}",
        supported: "支持",
        recommended: "推荐"
    },
    en: {
        title: "Interactive ROS Installation Guide",
        subtitle: "Step-by-step instructions for installing Robot Operating System on Ubuntu",
        selectConfiguration: "Select Your Configuration",
        linuxDistribution: "Linux Distribution",
        selectUbuntuVersion: "-- Select Ubuntu Version --",
        rosDistribution: "ROS Distribution",
        selectRosVersion: "-- Select ROS Version --",
        startInstallation: "Start Installation",
        previous: "Previous",
        next: "Next",
        verificationComplete: "Verification Complete!",
        installationSuccessful: "ROS Installation Successful!",
        successMessage: "You've successfully installed ROS on your system. Here are some next steps to get started:",
        tryTurtlesim: "Try running the turtlesim example: <code>rosrun turtlesim turtlesim_node</code>",
        exploreTutorials: "Explore ROS tutorials: <a href='http://wiki.ros.org/ROS/Tutorials' target='_blank'>ROS Wiki</a>",
        joinCommunity: "Join the ROS community: <a href='https://discourse.ros.org/' target='_blank'>ROS Discourse</a>",
        step: "Step {{current}} of {{total}}",
        finish: "Finish",
        troubleshootingTips: "Troubleshooting Tips",
        copy: "Copy",
        copied: "Copied!",
        compatible: "✅ Compatible: {{os}} with {{ros}}",
        incompatible: "❌ Incompatible: {{os}} with {{ros}}",
        supported: "Supported",
        recommended: "Recommended"
    }
};

class ROSInstallationGuide {
    constructor() {
        this.currentOS = null;
        this.currentROS = null;
        this.currentStep = 0;
        this.installationSteps = [];
        this.isInstallationStarted = false;
        this.currentLanguage = 'zh'; // Default to Chinese
        
        this.init();
    }
    
    init() {
        this.bindEvents();
        this.updateLanguage('zh'); // Set default language to Chinese
    }
    
    bindEvents() {
        // Language switcher buttons
        document.getElementById('lang-zh').addEventListener('click', () => {
            this.switchLanguage('zh');
        });
        
        document.getElementById('lang-en').addEventListener('click', () => {
            this.switchLanguage('en');
        });
        
        // OS selection change
        document.getElementById('os-select').addEventListener('change', (e) => {
            this.handleOSChange(e.target.value);
        });
        
        // ROS selection change
        document.getElementById('ros-select').addEventListener('change', (e) => {
            this.handleROSChange(e.target.value);
        });
        
        // Start installation button
        document.getElementById('start-install-btn').addEventListener('click', () => {
            this.startInstallation();
        });
        
        // Step navigation buttons
        document.getElementById('prev-btn').addEventListener('click', () => {
            this.goToPreviousStep();
        });
        
        document.getElementById('next-btn').addEventListener('click', () => {
            this.goToNextStep();
        });
    }
    
    // Switch language
    switchLanguage(language) {
        if (language !== this.currentLanguage) {
            this.currentLanguage = language;
            this.updateLanguage(language);
        }
    }
    
    // Update all text content based on selected language
    updateLanguage(language) {
        // Update language button states
        document.getElementById('lang-zh').classList.toggle('active', language === 'zh');
        document.getElementById('lang-en').classList.toggle('active', language === 'en');
        
        // Update all elements with data-i18n attribute
        const elements = document.querySelectorAll('[data-i18n]');
        elements.forEach(element => {
            const key = element.getAttribute('data-i18n');
            const translation = translations[language][key];
            if (translation) {
                if (element.tagName === 'INPUT' || element.tagName === 'SELECT' || element.tagName === 'TEXTAREA') {
                    element.placeholder = translation;
                } else if (element.tagName === 'OPTION') {
                    element.textContent = translation;
                } else {
                    element.innerHTML = translation;
                }
            }
        });
        
        // Update dynamic content
        if (this.isInstallationStarted) {
            this.updateProgress();
            this.loadStep(this.currentStep);
        }
        
        if (this.currentROS) {
            this.handleROSChange(this.currentROS);
        }
    }
    
    // Get translation for a key with optional placeholders
    t(key, params = {}) {
        let translation = translations[this.currentLanguage][key] || key;
        
        // Replace placeholders
        Object.keys(params).forEach(param => {
            translation = translation.replace(new RegExp(`{{${param}}}`, 'g'), params[param]);
        });
        
        return translation;
    }
    
    // Handle OS selection change
    handleOSChange(osValue) {
        this.currentOS = osValue;
        const rosSelect = document.getElementById('ros-select');
        const compatibilityMsg = document.getElementById('compatibility-message');
        
        // Reset ROS selection
        rosSelect.innerHTML = '<option value="">-- Select ROS Version --</option>';
        rosSelect.disabled = true;
        
        // Clear compatibility message
        compatibilityMsg.className = 'compatibility-message';
        compatibilityMsg.textContent = '';
        
        // Update start button state
        document.getElementById('start-install-btn').disabled = true;
        
        if (osValue) {
            const supportedROSVersions = rosData.osVersions[osValue].supportedRosVersions;
            
            // Populate ROS versions
            supportedROSVersions.forEach(rosVersion => {
                const rosInfo = rosData.rosVersions[rosVersion];
                const option = document.createElement('option');
                option.value = rosVersion;
                option.textContent = `${rosInfo.name} (${rosInfo.supportStatus})`;
                rosSelect.appendChild(option);
            });
            
            rosSelect.disabled = false;
        }
    }
    
    // Handle ROS selection change
    handleROSChange(rosValue) {
        this.currentROS = rosValue;
        const compatibilityMsg = document.getElementById('compatibility-message');
        const startBtn = document.getElementById('start-install-btn');
        
        if (rosValue) {
            const osInfo = rosData.osVersions[this.currentOS];
            const rosInfo = rosData.rosVersions[rosValue];
            
            // Check compatibility (though we only show compatible versions)
            if (osInfo.supportedRosVersions.includes(rosValue)) {
                compatibilityMsg.className = 'compatibility-message success';
                compatibilityMsg.innerHTML = this.t('compatible', { 
                    os: osInfo.name, 
                    ros: rosInfo.name 
                });
                startBtn.disabled = false;
            } else {
                compatibilityMsg.className = 'compatibility-message error';
                compatibilityMsg.innerHTML = this.t('incompatible', { 
                    os: osInfo.name, 
                    ros: rosInfo.name 
                });
                startBtn.disabled = true;
            }
        } else {
            compatibilityMsg.className = 'compatibility-message';
            compatibilityMsg.textContent = '';
            startBtn.disabled = true;
        }
    }
    
    // Start the installation process
    startInstallation() {
        if (!this.currentROS) {
            // Show error message if no ROS version is selected
            const compatibilityMsg = document.getElementById('compatibility-message');
            compatibilityMsg.className = 'compatibility-message error';
            compatibilityMsg.innerHTML = this.t('incompatible', { 
                os: rosData.osVersions[this.currentOS]?.name || 'Unknown OS', 
                ros: 'No ROS version selected' 
            });
            return;
        }
        
        this.isInstallationStarted = true;
        this.currentStep = 0;
        
        // Get installation steps for selected ROS version
        this.installationSteps = rosData.installationSteps[this.currentROS];
        
        // Hide selection section, show installation section
        document.getElementById('selection-section').style.display = 'none';
        document.getElementById('installation-section').style.display = 'block';
        
        // Initialize step indicators
        this.initializeStepIndicators();
        
        // Load first step
        this.loadStep(0);
        
        // Update progress
        this.updateProgress();
        
        // Update navigation buttons with correct translations
        this.updateNavigationButtons();
    }
    
    // Initialize step indicators
    initializeStepIndicators() {
        const container = document.getElementById('step-indicators');
        container.innerHTML = '';
        
        this.installationSteps.forEach((step, index) => {
            const indicator = document.createElement('button');
            indicator.className = `step-indicator ${index === 0 ? 'active' : ''}`;
            indicator.textContent = index + 1;
            indicator.addEventListener('click', () => {
                this.goToStep(index);
            });
            container.appendChild(indicator);
        });
    }
    
    // Load a specific step
    loadStep(stepIndex) {
        const step = this.installationSteps[stepIndex];
        const contentContainer = document.getElementById('step-content');
        
        // Create step content
        let html = `
            <h3>${step.title}</h3>
            <p>${step.description}</p>
            ${step.content}
            <div class="code-block">
                <div class="code-header">
                    <span class="code-language">bash</span>
                    <button class="copy-btn" data-clipboard-text="${this.escapeHtml(step.code)}" data-i18n="copy">Copy</button>
                </div>
                <pre><code class="language-bash">${this.escapeHtml(step.code)}</code></pre>
            </div>
        `;
        
        // Add troubleshooting section if available
        if (step.troubleshooting && step.troubleshooting.length > 0) {
            html += `
                <div class="troubleshooting-section">
                    <div class="troubleshooting-header">
                        <h4 data-i18n="troubleshootingTips">Troubleshooting Tips</h4>
                        <span class="toggle-icon">▼</span>
                    </div>
                    <div class="troubleshooting-content">
            `;
            
            step.troubleshooting.forEach(item => {
                html += `
                    <div class="troubleshooting-item">
                        <h5>${item.question}</h5>
                        <p>${item.answer}</p>
                    </div>
                `;
            });
            
            html += `
                    </div>
                </div>
            `;
        }
        
        contentContainer.innerHTML = html;
        
        // Apply translations to dynamic content
        this.updateLanguage(this.currentLanguage);
        
        // Bind copy button events
        this.bindCopyButtons();
        
        // Bind troubleshooting toggle events
        this.bindTroubleshootingToggles();
        
        // Reinitialize syntax highlighting
        this.highlightCode();
        
        // Update step indicators
        this.updateStepIndicators(stepIndex);
        
        // Update navigation buttons
        this.updateNavigationButtons();
        
        // Update current step
        this.currentStep = stepIndex;
    }
    
    // Bind copy to clipboard functionality
    bindCopyButtons() {
        const copyButtons = document.querySelectorAll('.copy-btn');
        
        copyButtons.forEach(button => {
            button.addEventListener('click', () => {
                const textToCopy = button.getAttribute('data-clipboard-text');
                this.copyToClipboard(textToCopy, button);
            });
        });
    }
    
    // Bind troubleshooting section toggles
    bindTroubleshootingToggles() {
        const headers = document.querySelectorAll('.troubleshooting-header');
        
        headers.forEach(header => {
            header.addEventListener('click', () => {
                const content = header.nextElementSibling;
                const icon = header.querySelector('.toggle-icon');
                
                content.classList.toggle('expanded');
                icon.classList.toggle('expanded');
            });
        });
    }
    
    // Copy text to clipboard
    copyToClipboard(text, button) {
        // Create a temporary textarea element
        const textarea = document.createElement('textarea');
        textarea.value = text;
        textarea.style.position = 'fixed';
        textarea.style.left = '-999999px';
        textarea.style.top = '-999999px';
        document.body.appendChild(textarea);
        
        // Select and copy the text
        textarea.select();
        document.execCommand('copy');
        
        // Remove the temporary element
        document.body.removeChild(textarea);
        
        // Update button text with translation
        const originalText = button.textContent;
        const copiedText = this.t('copied');
        button.textContent = copiedText;
        button.classList.add('copied');
        
        // Reset button text after 2 seconds
        setTimeout(() => {
            button.textContent = originalText;
            button.classList.remove('copied');
        }, 2000);
    }
    
    // Update step indicators
    updateStepIndicators(currentStep) {
        const indicators = document.querySelectorAll('.step-indicator');
        
        indicators.forEach((indicator, index) => {
            indicator.className = 'step-indicator';
            
            if (index < currentStep) {
                indicator.classList.add('completed');
            } else if (index === currentStep) {
                indicator.classList.add('active');
            }
        });
    }
    
    // Update navigation buttons state
    updateNavigationButtons() {
        const prevBtn = document.getElementById('prev-btn');
        const nextBtn = document.getElementById('next-btn');
        
        prevBtn.disabled = this.currentStep === 0;
        
        // Update button text with translations
        prevBtn.textContent = this.t('previous');
        
        // Change next button text on last step
        if (this.currentStep === this.installationSteps.length - 1) {
            nextBtn.textContent = this.t('finish');
        } else {
            nextBtn.textContent = this.t('next');
        }
    }
    
    // Update progress bar and text
    updateProgress() {
        const progressBar = document.querySelector('.progress-bar');
        const progressText = document.getElementById('progress-text');
        
        const progressPercentage = ((this.currentStep + 1) / this.installationSteps.length) * 100;
        progressBar.style.setProperty('--progress-width', `${progressPercentage}%`);
        progressBar.style.width = `${progressPercentage}%`;
        
        progressText.textContent = this.t('step', {
            current: this.currentStep + 1,
            total: this.installationSteps.length
        });
    }
    
    // Go to previous step
    goToPreviousStep() {
        if (this.currentStep > 0) {
            this.goToStep(this.currentStep - 1);
        }
    }
    
    // Go to next step
    goToNextStep() {
        if (this.currentStep < this.installationSteps.length - 1) {
            this.goToStep(this.currentStep + 1);
        } else {
            // Last step, show verification section
            this.showVerification();
        }
    }
    
    // Go to specific step
    goToStep(stepIndex) {
        if (stepIndex >= 0 && stepIndex < this.installationSteps.length) {
            this.loadStep(stepIndex);
            this.updateProgress();
        }
    }
    
    // Show verification section
    showVerification() {
        // Hide installation section
        document.getElementById('installation-section').style.display = 'none';
        
        // Show verification section
        document.getElementById('verification-section').style.display = 'block';
    }
    
    // Reinitialize syntax highlighting
    highlightCode() {
        // Re-run Prism highlighting
        if (typeof Prism !== 'undefined') {
            const codeBlocks = document.querySelectorAll('pre code');
            codeBlocks.forEach(block => {
                const language = block.getAttribute('class') || 'bash';
                if (language.startsWith('language-')) {
                    block.innerHTML = Prism.highlight(block.textContent, language.slice(9));
                } else {
                    block.innerHTML = Prism.highlight(block.textContent, language);
                }
            });
        }
    }
    
    // Escape HTML for code blocks
    escapeHtml(text) {
        const div = document.createElement('div');
        div.textContent = text;
        return div.innerHTML;
    }
}

// Initialize the application when DOM is loaded
document.addEventListener('DOMContentLoaded', () => {
    new ROSInstallationGuide();
});

// Add service worker for offline support
if ('serviceWorker' in navigator) {
    window.addEventListener('load', () => {
        navigator.serviceWorker.register('/sw.js')
            .then(registration => {
                console.log('ServiceWorker registration successful with scope: ', registration.scope);
            })
            .catch(error => {
                console.log('ServiceWorker registration failed: ', error);
            });
    });
}