# 硬件蓝牙连接模块使用说明

## 概述

这是一个基于你的硬件代码(`hardware.c`)和现有蓝牙代码(`bluetooth2.py`)开发的蓝牙连接模块，类似HC蓝牙助手的功能，专门用于接收小车激光雷达发送的数据。

## 功能特性

### 🔗 蓝牙连接管理
- **自动连接**: 支持自动检测和连接蓝牙设备
- **连接状态监控**: 实时监控连接状态和数据流量
- **自动重连**: 连接断开时自动尝试重连
- **错误处理**: 完善的异常处理和错误恢复机制

### 📡 数据解析
- **激光雷达数据**: 解析角度、距离、质量信息
- **IMU数据**: 解析三轴角度(roll, pitch, yaw)
- **电机数据**: 解析电机转速和状态
- **实时处理**: 多线程数据解析，确保实时性

### 🎮 电机控制
- **键盘控制**: 支持键盘实时控制小车
- **命令发送**: 发送0-5数字命令控制电机
- **状态反馈**: 实时显示电机状态和转速

### 📊 数据可视化
- **极坐标雷达图**: 实时显示激光雷达扫描数据
- **传感器数据**: 显示IMU和电机数据
- **交互控制**: 支持鼠标缩放和键盘控制

## 安装依赖

```bash
pip install pyserial matplotlib numpy
```

## 使用方法

### 1. 基本使用

```python
from hardware_bluetooth_module import HardwareBluetoothInterface, HardwareBluetoothVisualizer

# 创建蓝牙接口
bt_interface = HardwareBluetoothInterface(port="COM3", baudrate=115200)

# 连接蓝牙
if bt_interface.connect():
    print("蓝牙连接成功")
    
    # 发送控制命令
    bt_interface.send_motor_command(1)  # 前进
    
    # 获取数据
    lidar_data = bt_interface.get_latest_lidar_data()
    imu_data = bt_interface.get_latest_imu_data()
    
    # 断开连接
    bt_interface.disconnect()
```

### 2. 可视化模式

```python
# 启动完整可视化界面
python hardware_bluetooth_module.py
```

### 3. 键盘控制

在可视化界面中，可以使用以下键盘控制：

- **0**: 停止
- **1**: 前进
- **2**: 后退  
- **3**: 左转
- **4**: 右转
- **5**: 掉头

## 数据格式

### 激光雷达数据
```
A:123.45,D:0.678,Q:10
```
- A: 角度(度)
- D: 距离(米)
- Q: 质量

### IMU数据
```
roll:12.34, pitch:-5.67, yaw:90.0
```
- roll: 横滚角(度)
- pitch: 俯仰角(度)
- yaw: 偏航角(度)

### 电机数据
```
MotorA:123.4,RPMB:567.8
```
- MotorA: A电机转速(RPM)
- RPMB: B电机转速(RPM)

## 配置说明

### 蓝牙配置
```json
{
  "bluetooth": {
    "port": "COM3",           // 串口端口
    "baudrate": 115200,       // 波特率
    "timeout": 0.1,           // 超时时间
    "retry_count": 3,          // 重试次数
    "auto_reconnect": true    // 自动重连
  }
}
```

### 数据解析配置
```json
{
  "data_parsing": {
    "lidar_pattern": "A:([-+]?\\d*\\.?\\d+),\\s*D:([-+]?\\d*\\.?\\d+)m?,\\s*Q:(\\d+)",
    "imu_pattern": "roll:([-+]?\\d*\\.?\\d+),\\s*pitch:([-+]?\\d*\\.?\\d+),\\s*yaw:([-+]?\\d*\\.?\\d+)",
    "motor_pattern": "MotorA:([-+]?\\d*\\.?\\d+),RPMB:([-+]?\\d*\\.?\\d+)",
    "buffer_size": 1000,
    "max_data_age": 10.0
  }
}
```

## 故障排除

### 1. 连接问题
- **检查串口**: 确认COM端口号正确
- **检查波特率**: 确保与硬件一致(115200)
- **检查权限**: 确保有串口访问权限
- **检查占用**: 确保串口未被其他程序占用

### 2. 数据解析问题
- **检查数据格式**: 确保硬件发送的数据格式正确
- **检查正则表达式**: 根据实际数据格式调整解析模式
- **检查编码**: 确保数据编码为UTF-8

### 3. 可视化问题
- **检查matplotlib**: 确保matplotlib正确安装
- **检查后端**: 使用TkAgg后端
- **检查字体**: 确保中文字体正确配置

## 高级用法

### 1. 自定义数据回调

```python
def my_data_callback(data):
    print(f"收到数据: {data}")

bt_interface.add_data_callback(my_data_callback)
```

### 2. 数据过滤

```python
# 获取特定质量的数据
lidar_data = bt_interface.get_latest_lidar_data()
filtered_data = [point for point in lidar_data if point.quality > 10]
```

### 3. 数据保存

```python
import json

# 保存激光雷达数据
lidar_data = bt_interface.get_latest_lidar_data()
data_to_save = [
    {
        'angle': point.angle,
        'distance': point.distance,
        'quality': point.quality,
        'timestamp': point.timestamp
    }
    for point in lidar_data
]

with open('lidar_data.json', 'w') as f:
    json.dump(data_to_save, f, indent=2)
```

## 与HC蓝牙助手的对比

| 功能 | HC蓝牙助手 | 本模块 |
|------|------------|--------|
| 蓝牙连接 | ✅ | ✅ |
| 数据接收 | ✅ | ✅ |
| 数据解析 | ✅ | ✅ |
| 可视化 | ❌ | ✅ |
| 电机控制 | ❌ | ✅ |
| 数据保存 | ❌ | ✅ |
| 错误处理 | 基础 | 完善 |
| 扩展性 | 有限 | 高 |

## 注意事项

1. **数据同步**: 确保硬件和软件的数据格式一致
2. **性能优化**: 大数据量时注意内存使用
3. **线程安全**: 多线程环境下注意数据同步
4. **错误处理**: 网络不稳定时注意重连机制
5. **资源清理**: 程序退出时确保资源正确释放

## 技术支持

如有问题，请检查：
1. 硬件连接是否正常
2. 串口配置是否正确
3. 数据格式是否匹配
4. 依赖库是否完整安装
