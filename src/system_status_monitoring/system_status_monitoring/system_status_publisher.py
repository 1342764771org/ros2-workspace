import rclpy
from rclpy.node import Node
from zack_interfaces.msg import SystemStatus  # 替换为你实际的包名和消息名
from std_msgs.msg import Header
import platform
import psutil
import time

class SystemStatusPublisher(Node):
    def __init__(self):
        super().__init__('system_status_publisher')
        self.publisher_ = self.create_publisher(SystemStatus, 'system_status', 10)
        timer_period = 5  # 发布周期为5秒
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.previous_net_io = psutil.net_io_counters()

    def timer_callback(self):
        msg = SystemStatus()
        
        # 填充 Header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "system_status_frame"

        # 系统信息
        msg.hostname = platform.node()
        msg.os_name = platform.system()
        msg.os_version = platform.version()
        
        # CPU 信息
        msg.cpu_usage_percent = psutil.cpu_percent()
        
        # 内存信息
        mem = psutil.virtual_memory()
        msg.memory_total_mb = mem.total / (1024 ** 2)
        msg.memory_available_mb = mem.available / (1024 ** 2)
        msg.memory_usage_percent = mem.percent
        
        # 磁盘信息
        disk = psutil.disk_usage('/')
        msg.disk_total_gb = disk.total / (1024 ** 3)
        msg.disk_available_gb = disk.free / (1024 ** 3)
        msg.disk_usage_percent = disk.percent
        
        # 网络速率
        current_net_io = psutil.net_io_counters()
        upload_bytes = current_net_io.bytes_sent - self.previous_net_io.bytes_sent
        download_bytes = current_net_io.bytes_recv - self.previous_net_io.bytes_recv
        self.previous_net_io = current_net_io
        msg.network_upload_kbps = upload_bytes / 1024 / 5  # 5秒间隔
        msg.network_download_kbps = download_bytes / 1024 / 5
        
        # 系统启动时间
        msg.uptime_seconds = int(time.time() - psutil.boot_time())
        
        # CPU 温度（需要根据具体硬件和系统调整）
        try:
            temps = psutil.sensors_temperatures()
            if 'coretemp' in temps:
                msg.cpu_temperature_celsius = temps['coretemp'][0].current
            else:
                msg.cpu_temperature_celsius = 0.0  # 无法获取温度
        except:
            msg.cpu_temperature_celsius = 0.0  # 异常处理
        
        # GPU 使用率（需根据具体硬件和库实现，此处占位）
        msg.gpu_usage_percent = 0.0  # 示例值
        
        # 当前活跃进程数
        msg.active_processes = len(psutil.pids())
        
        # 发布消息
        self.publisher_.publish(msg)
        self.get_logger().info('发布系统状态')

def main(args=None):
    rclpy.init(args=args)
    system_status_publisher = SystemStatusPublisher()
    rclpy.spin(system_status_publisher)
    system_status_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()