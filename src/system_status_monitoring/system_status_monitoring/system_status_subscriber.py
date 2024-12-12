import rclpy
from rclpy.node import Node
from zack_interfaces.msg import SystemStatus
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import matplotlib as mpl
from matplotlib.ticker import MultipleLocator
import matplotlib.gridspec as gridspec

class SystemStatusSubscriber(Node):
    def __init__(self):
        super().__init__('system_status_subscriber')
        self.subscription = self.create_subscription(
            SystemStatus,
            'system_status',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning

        # Initialize data buffers
        self.cpu_usage = deque(maxlen=60)        # CPU Usage (%)
        self.memory_usage = deque(maxlen=60)     # Memory Usage (%)
        self.network_upload = deque(maxlen=60)   # Network Upload Rate (KB/s)
        self.network_download = deque(maxlen=60) # Network Download Rate (KB/s)
        self.time_stamps = deque(maxlen=60)      # Timestamps (relative time)

        # Initialize other fields
        self.hostname = ""
        self.os_name = ""
        self.os_version = ""
        self.disk_usage_percent = 0.0
        self.cpu_temperature_celsius = 0.0
        self.gpu_usage_percent = 0.0
        self.uptime_seconds = 0
        self.active_processes = 0

    def listener_callback(self, msg):
        current_time = msg.header.stamp.sec
        self.get_logger().info(f'Received message at {current_time}')
        self.time_stamps.append(current_time)
        self.cpu_usage.append(msg.cpu_usage_percent)
        self.memory_usage.append(msg.memory_usage_percent)
        self.network_upload.append(msg.network_upload_kbps)
        self.network_download.append(msg.network_download_kbps)
        self.disk_usage_percent = msg.disk_usage_percent
        self.cpu_temperature_celsius = msg.cpu_temperature_celsius
        self.gpu_usage_percent = msg.gpu_usage_percent
        self.uptime_seconds = msg.uptime_seconds
        self.active_processes = msg.active_processes

        self.get_logger().info(
            f'CPU: {msg.cpu_usage_percent}%, '
            f'Memory: {msg.memory_usage_percent}%, '
            f'Upload: {msg.network_upload_kbps} KB/s, '
            f'Download: {msg.network_download_kbps} KB/s'
        )

def main(args=None):
    rclpy.init(args=args)
    system_status_subscriber = SystemStatusSubscriber()

    mpl.rcParams['axes.unicode_minus'] = False  # Correct display of minus sign
    plt.style.use('ggplot')

    # Create figure with a GridSpec layout to allocate space for texts and plots
    fig = plt.figure(figsize=(10, 12), constrained_layout=True)
    gs = gridspec.GridSpec(nrows=5, ncols=1, figure=fig, height_ratios=[1, 2, 2, 2, 1])

    # Top section for system information texts
    ax_top = fig.add_subplot(gs[0, 0])
    ax_top.axis('off')  # No axes for the text section

    # Middle sections for plots
    ax1 = fig.add_subplot(gs[1, 0])
    ax2 = fig.add_subplot(gs[2, 0])
    ax3 = fig.add_subplot(gs[3, 0])

    # Bottom section can be used for additional texts or left empty
    ax_bottom = fig.add_subplot(gs[4, 0])
    ax_bottom.axis('off')  # No axes for the text section

    # Set window title
    try:
        fig.canvas.manager.set_window_title('SYSTEM STATUS MONITORING')
    except AttributeError:
        # Some Matplotlib backends may not support set_window_title
        pass

    # Set main title
    fig.suptitle('System Status Monitoring', fontsize=20, fontweight='bold')

    # Initialize line objects for CPU and Memory usage
    cpu_line, = ax1.plot([], [], label='CPU Usage (%)', color='blue', linewidth=2)
    mem_line, = ax2.plot([], [], label='Memory Usage (%)', color='green', linewidth=2)
    upload_line, = ax3.plot([], [], label='Upload (KB/s)', color='red', linewidth=2)
    download_line, = ax3.plot([], [], label='Download (KB/s)', color='purple', linewidth=2)

    # Initialize text objects in the top text area
    # Create a single string with all system info to avoid overlapping
    system_info_text = ax_top.text(
        0.01, 0.6, '', fontsize=12, ha='left', va='center',
        transform=ax_top.transAxes, bbox=dict(facecolor='white', alpha=0.5)
    )

    # Initialize text objects in the bottom text area (dynamic info)
    dynamic_info_text = ax_bottom.text(
        0.99, 0.5, '', fontsize=12, ha='right', va='center',
        transform=ax_bottom.transAxes, bbox=dict(facecolor='white', alpha=0.5)
    )

    # Retrieve data buffers
    cpu_usage = system_status_subscriber.cpu_usage
    memory_usage = system_status_subscriber.memory_usage
    network_upload = system_status_subscriber.network_upload
    network_download = system_status_subscriber.network_download
    time_stamps = system_status_subscriber.time_stamps

    # Configure each subplot
    for ax in [ax1, ax2, ax3]:
        ax.grid(True)
        ax.set_xlim(0, 300)  # Display the last 5 minutes (300 seconds)
        ax.set_ylim(0, 100)  # Percentage metrics

        # Set major ticks every 5 seconds on the x-axis
        ax.xaxis.set_major_locator(MultipleLocator(5))
        ax.tick_params(axis='x', labelsize=10)
        ax.tick_params(axis='y', labelsize=10)

    # Customize individual plots
    ax1.set_title('CPU Usage (%)', fontsize=16)
    ax1.set_xlabel('Time (sec)', fontsize=14)
    ax1.set_ylabel('CPU (%)', fontsize=14)
    ax1.legend(loc='upper right', fontsize=12)

    ax2.set_title('Memory Usage (%)', fontsize=16)
    ax2.set_xlabel('Time (sec)', fontsize=14)
    ax2.set_ylabel('Memory (%)', fontsize=14)
    ax2.legend(loc='upper right', fontsize=12)

    ax3.set_title('Network Upload/Download Rates (KB/s)', fontsize=16)
    ax3.set_xlabel('Time (sec)', fontsize=14)
    ax3.set_ylabel('Rate (KB/s)', fontsize=14)
    ax3.legend(loc='upper right', fontsize=12)

    def update_plot(frame):
        # Process ROS 2 messages
        rclpy.spin_once(system_status_subscriber, timeout_sec=0)

        # Prepare relative time
        if len(time_stamps) > 0:
            start_time = time_stamps[0]
            rel_time = [t - start_time for t in time_stamps]
        else:
            rel_time = []

        # Update x-axis limits to show the last 5 minutes
        if len(rel_time) > 0:
            current_max_time = rel_time[-1]
            for ax in [ax1, ax2, ax3]:
                ax.set_xlim(max(0, current_max_time - 300), current_max_time + 10)
        else:
            for ax in [ax1, ax2, ax3]:
                ax.set_xlim(0, 300)

        # Update CPU usage plot
        cpu_line.set_data(rel_time, list(cpu_usage))
        ax1.set_ylim(0, 100)

        # Update Memory usage plot
        mem_line.set_data(rel_time, list(memory_usage))
        ax2.set_ylim(0, 100)

        # Update Network upload and download plots
        upload_line.set_data(rel_time, list(network_upload))
        download_line.set_data(rel_time, list(network_download))
        max_network_rate = max(
            max(network_upload, default=100),
            max(network_download, default=100)
        ) + 50
        ax3.set_ylim(0, max_network_rate)
        ax3.relim()
        ax3.autoscale_view()

        # Update system information texts
        system_info = (
            f'Hostname: {system_status_subscriber.hostname}\n'
            f'OS: {system_status_subscriber.os_name} {system_status_subscriber.os_version}'
        )
        system_info_text.set_text(system_info)

        # Update dynamic information texts
        dynamic_info = (
            f'Disk Usage: {system_status_subscriber.disk_usage_percent:.1f}%\n'
            f'CPU Temp: {system_status_subscriber.cpu_temperature_celsius:.1f}°C\n'
            f'GPU Usage: {system_status_subscriber.gpu_usage_percent:.1f}%\n'
            f'Uptime: {system_status_subscriber.uptime_seconds} sec\n'
            f'Active Processes: {system_status_subscriber.active_processes}'
        )
        dynamic_info_text.set_text(dynamic_info)

        return (cpu_line, mem_line, upload_line, download_line,
                system_info_text, dynamic_info_text)

    # Create the animation object and keep a reference
    ani = animation.FuncAnimation(fig, update_plot, interval=1000, blit=False)

    # Display the plot
    plt.show()

    # Clean up ROS node when the plot window is closed
    system_status_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()