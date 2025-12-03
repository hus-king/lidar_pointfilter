# lidar_pointfilter

系统要求: Ubuntu 20.04  
ROS 要求: ROS1 Noetic

ROS安装方式:
1. 下载并安装 fishros：
   ```
   wget http://fishros.com/install -O fishros && . fishros
   ```
   在安装提示中选择 Noetic (ROS1) 桌面版。

使用步骤:
1. 配置本机网口 IP（示例）：
   - 将网口 IP 设置为 `192.168.111.204`（根据实际网口名称和系统工具执行配置）。
2. 编译工作区：
   ```
   cd ~/lidar_pointfilter
   catkin_make
   ```

在编译完成后，将工作区环境添加到你的 ~/.bashrc 并立即生效（推荐）：
```
# 将工作区环境追加到 ~/.bashrc
echo 'source ~/lidar_pointfilter/devel/setup.bash' >> ~/.bashrc
# 立即加载生效
source ~/.bashrc
```
或者手动在 `~/.bashrc` 文件最后添加一行：
```
source ~/lidar_pointfilter/devel/setup.bash
```

3. 启动脚本：
   ```
   cd ~/lidar_pointfilter/src/lidar_process/shell
   ./lidar_process.sh
   ```

参数修改（示例）
- 若需调整 sector_min_filter 节点的参数，可修改或参考如下 launch 片段（将其放入对应的 launch 文件中，例如 `src/lidar_process/launch/sector_min_filter.launch`），其中可修改输入/输出话题、高度阈值、扇区数量和串口配置等：

```xml
<launch>
  <node pkg="lidar_process" type="sector_min_filter" name="sector_min_filter" output="screen">
    <param name="input_topic" value="/tanwaylidar_pointcloud" />
    <param name="output_topic" value="/min_above_floor" />
    <param name="z_floor" value="0.10" />
    <param name="z_ceiling" value="2.00" />
    <param name="sectors" value="12" />
    <param name="serial_device" value="/dev/ttyUSB0" />
    <param name="serial_baud" value="115200" />
  </node>
</launch>
```

- 常用参数说明：
  - input_topic: 输入点云话题（默认 /tanwaylidar_pointcloud）。
  - output_topic: 过滤后输出的话题（默认 /min_above_floor）。
  - z_floor / z_ceiling: 过滤的高度下限和上限（单位 m）。
  - sectors: 水平分扇区数量，用于最小值计算。
  - serial_device / serial_baud: 若节点需要串口通信，用于设置串口设备和波特率。

备注:
- 确保已安装并正确配置 ROS Noetic 环境（例如 `source /opt/ros/noetic/setup.bash` 或通过 fishros 完成）。
- 若使用 tmux 启动脚本，请确保已安装 tmux。

串口开发指南
- 概述：sector_min_filter 节点会把每帧筛选出的扇区最近点通过串口以二进制协议发送到下位机（或其他串口设备）。接收端需按照下列格式解析并校验数据。

报文格式（Little-Endian）
- Header (2 bytes): 0xAA, 0x55
- Count  (1 byte) : N（点数量，uint8，最大 255；通常 ≤ sectors）
- Payload (N × 12 bytes): 每点 3 个 float32（x, y, z），按顺序，均为 IEEE754 小端 float（4B each）
- CRC    (2 bytes) : uint16，小端（低字节先发）
  - CRC 计算（sector_min_filter.cpp 中实现）：先对所有点的 x+y+z 求和（float），取绝对值乘 1000，取低 16 位作为 uint16 校验值：
    crc = (uint16_t)( (uint32_t)(abs(sum_xyz) * 1000.0f) & 0xFFFF )

示例：二进制帧字节序（表示 N=2）
[0xAA][0x55][0x02][x0(4B)][y0(4B)][z0(4B)][x1(4B)][y1(4B)][z1(4B)][crc_lo][crc_hi]

串口配置建议
- 设备：/dev/ttyUSB0（默认，可通过 ROS 参数 serial_device 改变）
- 波特率：115200（默认，可通过 ROS 参数 serial_baud 改变）
- 超时与重连：节点会在串口未打开时定期重试连接（每 ~5s 重试一次），接收端应能容忍短断开。

解析要点与注意事项
- 小端字节序：float32 与 uint16 均为小端（little-endian）。
- 流式接收：上位机可能在任意位置截断或接收到半帧，接收端应实现基于 Header 的帧同步（寻找 0xAA 0x55 起始，然后读取后续长度字节及剩余数据）。
- 部分帧丢失：节点可能在高频率时发送较多帧，接收端需做好缓冲与丢弃策略（别阻塞串口线程）。
- 点数量上限：N 由节点发布点数决定，且不会超过 sectors（通常小于 255）。
- CRC 校验：在接收端按相同算法计算并比较，校验失败则舍弃帧。

Python 解析示例（伪代码）
```python
# 简单示例，阻塞读取 + 帧同步（仅示意，不够健壮）
import serial
import struct

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

def read_frame():
    # 同步 header
    while True:
        b = ser.read(1)
        if not b: return None
        if b[0] == 0xAA:
            b2 = ser.read(1)
            if not b2: return None
            if b2[0] == 0x55:
                break
    cnt_b = ser.read(1)
    if not cnt_b: return None
    n = cnt_b[0]
    payload = ser.read(n * 12)
    crc_bytes = ser.read(2)
    if len(payload) != n*12 or len(crc_bytes) != 2:
        return None
    # 解析点
    pts = []
    ssum = 0.0
    for i in range(n):
        x, y, z = struct.unpack_from('<fff', payload, i*12)
        pts.append((x,y,z))
        ssum += x + y + z
    # 计算 crc
    raw = int(abs(ssum) * 1000.0) & 0xFFFF
    crc_received = struct.unpack('<H', crc_bytes)[0]
    if raw != crc_received:
        # CRC 错误，丢弃
        return None
    return pts

# 使用：
while True:
    frame = read_frame()
    if frame is None:
        continue
    # 处理 frame（列表 of (x,y,z)）
```