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

备注:
- 确保已安装并正确配置 ROS Noetic 环境（例如 `source /opt/ros/noetic/setup.bash` 或通过 fishros 完成）。
- 若使用 tmux 启动脚本，请确保已安装 tmux。