#!/usr/bin/env python3
"""
ROS发布者节点示例
发布字符串消息到chatter话题

参考: https://github.com/matreshka15/ROS-based-USV-project
"""

import rospy
from std_msgs.msg import String
from datetime import datetime

class TalkerNode:
    def __init__(self):
        """初始化发布者节点"""
        # 初始化节点，anonymous=True会自动生成唯一节点名
        rospy.init_node('talker_node', anonymous=True)
        
        # 创建发布者对象
        # 话题名称: 'chatter'
        # 消息类型: String
        # 队列大小: 10（消息缓冲区大小）
        self.publisher = rospy.Publisher('chatter', String, queue_size=10)
        
        # 设置发布频率（Hz）
        self.rate = rospy.Rate(10)  # 10Hz，即每秒发布10次
        
        # 节点状态
        self.node_name = rospy.get_name()
        self.start_time = rospy.get_time()
        
        rospy.loginfo(f"[{self.node_name}] 初始化完成，发布频率: {self.rate.sleep_dur.to_sec()}秒")
    
    def run(self):
        """运行节点主循环"""
        message_count = 0
        
        # 循环直到节点被关闭（Ctrl+C）
        while not rospy.is_shutdown():
            # 计算运行时间
            current_time = rospy.get_time()
            elapsed_time = current_time - self.start_time
            
            # 创建消息内容
            message = String()
            message.data = f"Hello ROS! " \
                          f"时间: {datetime.fromtimestamp(current_time).strftime('%H:%M:%S.%f')[:-3]} " \
                          f"计数: {message_count} " \
                          f"运行时间: {elapsed_time:.2f}秒"
            
            # 发布消息
            self.publisher.publish(message)
            
            # 记录日志信息
            rospy.loginfo(f"[{self.node_name}] 发布消息: {message.data}")
            
            # 增加消息计数
            message_count += 1
            
            # 按照设定频率休眠
            self.rate.sleep()
    
    def cleanup(self):
        """节点关闭时的清理工作"""
        rospy.loginfo(f"[{self.node_name}] 正在关闭，共发布消息: {self.message_count}条")

def main():
    """主函数"""
    try:
        # 创建节点实例
        node = TalkerNode()
        
        # 运行节点
        node.run()
        
    except rospy.ROSInterruptException:
        # 捕获Ctrl+C异常，优雅退出
        rospy.loginfo("节点被用户中断")
    
    except Exception as e:
        # 捕获其他异常
        rospy.logerr(f"节点运行出错: {str(e)}")
    
    finally:
        rospy.loginfo("节点已退出")

if __name__ == '__main__':
    main()