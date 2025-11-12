#!/usr/bin/env python3
"""
ROS订阅者节点示例
订阅chatter话题并处理收到的消息

参考: https://github.com/matreshka15/ROS-based-USV-project
"""

import rospy
from std_msgs.msg import String
from datetime import datetime

class ListenerNode:
    def __init__(self):
        """初始化订阅者节点"""
        # 初始化节点
        rospy.init_node('listener_node', anonymous=True)
        
        # 创建订阅者对象
        # 话题名称: 'chatter'
        # 消息类型: String
        # 回调函数: self.callback
        # 队列大小: 10
        self.subscriber = rospy.Subscriber('chatter', String, self.callback, queue_size=10)
        
        # 统计信息
        self.message_count = 0
        self.first_message_time = None
        self.last_message_time = None
        self.total_latency = 0.0
        
        self.node_name = rospy.get_name()
        rospy.loginfo(f"[{self.node_name}] 初始化完成，等待消息...")
    
    def callback(self, data):
        """消息回调函数"""
        # 记录接收时间
        receive_time = rospy.get_time()
        
        # 更新统计信息
        self.message_count += 1
        self.last_message_time = receive_time
        
        if self.first_message_time is None:
            self.first_message_time = receive_time
        
        # 计算消息延迟（如果消息中包含时间信息）
        try:
            # 尝试从消息中提取发送时间
            if "时间: " in data.data:
                time_str = data.data.split("时间: ")[1].split(" ")[0]
                send_time = datetime.strptime(time_str, "%H:%M:%S.%f").timestamp()
                # 由于是同一天，可以直接使用时间戳
                latency = receive_time - send_time
                self.total_latency += latency
                avg_latency = self.total_latency / self.message_count
                latency_info = f"延迟: {latency:.3f}秒 (平均: {avg_latency:.3f}秒)"
            else:
                latency_info = "延迟: 未知"
        except:
            latency_info = "延迟: 计算失败"
        
        # 记录日志
        rospy.loginfo(f"[{self.node_name}] 收到消息 #{self.message_count}: {data.data} | {latency_info}")
    
    def run(self):
        """运行节点"""
        # 保持节点运行，等待消息
        rospy.spin()
    
    def print_statistics(self):
        """打印统计信息"""
        if self.message_count > 0:
            runtime = self.last_message_time - self.first_message_time
            avg_frequency = self.message_count / runtime if runtime > 0 else 0
            
            rospy.loginfo(f"\n[{self.node_name}] 统计信息:")
            rospy.loginfo(f"  总消息数: {self.message_count}")
            rospy.loginfo(f"  运行时间: {runtime:.2f}秒")
            rospy.loginfo(f"  平均频率: {avg_frequency:.2f}Hz")
            rospy.loginfo(f"  平均延迟: {self.total_latency/self.message_count:.3f}秒")
        else:
            rospy.loginfo(f"[{self.node_name}] 未收到任何消息")

def main():
    """主函数"""
    try:
        # 创建节点实例
        node = ListenerNode()
        
        # 运行节点
        node.run()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("节点被用户中断")
    
    except Exception as e:
        rospy.logerr(f"节点运行出错: {str(e)}")
    
    finally:
        # 打印统计信息
        if 'node' in locals():
            node.print_statistics()
        rospy.loginfo("节点已退出")

if __name__ == '__main__':
    main()