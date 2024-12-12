# 自定义消息类型，用于获取计算机的基本系统状态

# 主机名
string hostname

# 操作系统名称
string os_name

# 操作系统版本
string os_version

# 时间戳和坐标信息
std_msgs/Header header

# CPU 使用率（百分比）
float32 cpu_usage_percent

# 内存总量（MB）
float64 memory_total_mb

# 内存可用量（MB）
float64 memory_available_mb

# 内存使用率（百分比）
float32 memory_usage_percent

# 磁盘总量（GB）
float64 disk_total_gb

# 磁盘可用量（GB）
float64 disk_available_gb

# 磁盘使用率（百分比）
float32 disk_usage_percent

# 网络上传速率（KB/s）
float32 network_upload_kbps

# 网络下载速率（KB/s）
float32 network_download_kbps

# 系统启动时间（自纪元以来的秒数）
uint64 uptime_seconds

# CPU 温度（摄氏度，可选，根据硬件支持情况）
float32 cpu_temperature_celsius

# GPU 使用率（如果适用，可选）
float32 gpu_usage_percent

# 当前活跃进程数
uint32 active_processes