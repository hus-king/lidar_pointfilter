#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>
#include <vector>
#include <limits>
#include <serial/serial.h>
#include <memory>
#include <thread>
#include <chrono>

class SectorMinFilter {
public:
    SectorMinFilter() : nh_("~"), serial_port_(nullptr) {
        std::string input_topic, output_topic, serial_device;
        int sectors = 12;
        int serial_baud = 115200;
        double z_floor = -std::numeric_limits<double>::max();
        double z_ceiling = std::numeric_limits<double>::max();

        // 从参数服务器读取配置
        nh_.param<std::string>("input_topic", input_topic, "/tanwaylidar_pointcloud");
        nh_.param<std::string>("output_topic", output_topic, "/filtered_points");
        nh_.param<int>("sectors", sectors, 12);
        nh_.param<double>("z_floor", z_floor, -1e6);
        nh_.param<double>("z_ceiling", z_ceiling, 1e6);
        nh_.param<std::string>("serial_device", serial_device, "/dev/ttyUSB0");
        nh_.param<int>("serial_baud", serial_baud, 115200);

        // 安全校验
        if (sectors <= 0) {
            ROS_WARN("sectors=%d invalid, fallback to 12", sectors);
            sectors = 12;
        }

        sub_ = nh_.subscribe(input_topic, 1, &SectorMinFilter::cloudCallback, this);
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic, 1);

        // 初始化串口（调试友好：失败不退出，仅告警）
        serial_port_ = std::make_unique<serial::Serial>();
        try {
            serial_port_->setPort(serial_device);
            serial_port_->setBaudrate(serial_baud);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            serial_port_->setTimeout(timeout);
            serial_port_->open();
            if (serial_port_->isOpen()) {
                ROS_INFO("Serial port opened successfully: %s @ %d bps", 
                         serial_device.c_str(), serial_baud);
            } else {
                ROS_WARN("Serial port open() returned false (device may not exist): %s", 
                         serial_device.c_str());
                // 不置空，保留对象以便后续重试（sendToSerial 会检测 isOpen）
            }
        } catch (const std::exception& e) {
            ROS_WARN("Serial init failed (non-fatal, will retry later): %s", e.what());
            // 不置空；sendToSerial 中会检查 isOpen()
        }

        // 打印完整初始化信息
        ROS_INFO("==================================================");
        ROS_INFO("SectorMinFilter Node Initialized");
        ROS_INFO("--------------------------------------------------");
        ROS_INFO("  input_topic : %s", input_topic.c_str());
        ROS_INFO("  output_topic: %s", output_topic.c_str());
        ROS_INFO("  sectors     : %d", sectors);
        ROS_INFO("  z_floor     : %.3f m", z_floor);
        ROS_INFO("  z_ceiling   : %.3f m", z_ceiling);
        ROS_INFO("  serial_device: %s", serial_device.c_str());
        ROS_INFO("  serial_baud : %d", serial_baud);
        ROS_INFO("==================================================");
    }

    ~SectorMinFilter() {
        if (serial_port_ && serial_port_->isOpen()) {
            try {
                serial_port_->close();
                ROS_INFO("Serial port closed.");
            } catch (...) {
                ROS_WARN("Exception during serial close.");
            }
        }
    }

private:
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        // 转换为 PCL 点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        try {
            pcl::fromROSMsg(*msg, *cloud);
        } catch (const std::exception& e) {
            ROS_ERROR("❌ pcl::fromROSMsg failed: %s", e.what());
            return;
        }

        // 动态读取参数（支持 rosparam set 实时调整）
        int sectors = 12;
        double z_floor = -1e6, z_ceiling = 1e6;
        nh_.param<int>("sectors", sectors, 12);
        nh_.param<double>("z_floor", z_floor, -1e6);
        nh_.param<double>("z_ceiling", z_ceiling, 1e6);
        if (sectors <= 0) sectors = 12;

        const double sector_angle = 2.0 * M_PI / sectors;

        // 每个扇区记录最近点索引（-1 表示无有效点）
        std::vector<int> nearest_idx(sectors, -1);
        std::vector<double> min_dist(sectors, std::numeric_limits<double>::max());

        // Step 1: 遍历点云，筛选 z ∈ [z_floor, z_ceiling] 的点，找各扇区最近点
        for (size_t i = 0; i < cloud->points.size(); ++i) {
            const auto& pt = cloud->points[i];
            // 跳过无效点
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) 
                continue;
            // 双重过滤：z_floor ≤ z ≤ z_ceiling
            if (pt.z < z_floor || pt.z > z_ceiling) 
                continue;

            double dist_sq = pt.x * pt.x + pt.y * pt.y;
            if (dist_sq < 1e-6) 
                continue; // 排除原点附近噪声

            // 计算方位角 [0, 2π)
            double angle = std::atan2(pt.y, pt.x);
            if (angle < 0) 
                angle += 2.0 * M_PI;

            int sector = static_cast<int>(angle / sector_angle);
            if (sector >= sectors) 
                sector = sectors - 1;

            double dist = std::sqrt(dist_sq);
            if (dist < min_dist[sector]) {
                min_dist[sector] = dist;
                nearest_idx[sector] = static_cast<int>(i);
            }
        }

        // Step 2: 构建输出点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        output_cloud->header = cloud->header;
        output_cloud->is_dense = true;

        // 自适应宽度的 ASCII 表头（支持任意 sectors）
        int bar_width = 60 + (sectors > 12 ? (sectors - 12) * 2 : 0);
        if (bar_width < 60) bar_width = 60;
        if (bar_width > 100) bar_width = 100;
        std::string bar(bar_width, '-');
        ROS_INFO("+%s+", bar.c_str());
        ROS_INFO("| SectorMinFilter: sectors=%d, z : [%.2f, %.2f] m                |", 
                 sectors, z_floor, z_ceiling);
        ROS_INFO("| [%2d] angle [start-end) | x       y       z                        |", 0);
        ROS_INFO("+%s+", bar.c_str());

        // 输出每个扇区
        for (int s = 0; s < sectors; ++s) {
            double start_deg = s * (360.0 / sectors);
            double end_deg = (s + 1) * (360.0 / sectors);

            if (nearest_idx[s] != -1) {
                const auto& pt = cloud->points[nearest_idx[s]];
                double angle_rad = std::atan2(pt.y, pt.x);
                if (angle_rad < 0) angle_rad += 2.0 * M_PI;
                double angle_deg = angle_rad * 180.0 / M_PI;

                output_cloud->points.push_back(pt);
                ROS_INFO("| [%2d] %6.1f [%5.1f-%5.1f) | x=%7.3f y=%7.3f z=%6.3f |",
                         s, angle_deg, start_deg, end_deg, pt.x, pt.y, pt.z);
            } else {
                ROS_INFO("| [%2d]     --- [%5.1f-%5.1f) | x=      - y=      - z=     - |",
                         s, start_deg, end_deg);
            }
        }

        ROS_INFO("+%s+", bar.c_str());
        ROS_INFO("| Published %2zu points (frame_id: %s)                              |", 
                 output_cloud->points.size(), msg->header.frame_id.c_str());
        ROS_INFO("+%s+", bar.c_str());

        // ←★★★ 核心新增：将筛选出的点通过串口发送给下位机 ★★★
        sendToSerial(output_cloud, sectors);

        // 发布结果（保持原时间戳 & frame_id）
        sensor_msgs::PointCloud2 out_msg;
        try {
            pcl::toROSMsg(*output_cloud, out_msg);
        } catch (const std::exception& e) {
            ROS_ERROR("pcl::toROSMsg failed: %s", e.what());
            return;
        }
        out_msg.header = msg->header;
        pub_.publish(out_msg);
    }

    void sendToSerial(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int sectors) {
        if (!serial_port_->isOpen()) {
            ROS_WARN_THROTTLE(5.0, "Serial port not open, skip sending (will retry auto)");
            // 尝试重连（非阻塞，最多每5秒试一次）
            static ros::Time last_retry = ros::Time::now();
            if ((ros::Time::now() - last_retry).toSec() > 5.0) {
                last_retry = ros::Time::now();
                try {
                    std::string dev; int baud;
                    nh_.param<std::string>("serial_device", dev, "/dev/ttyUSB0");
                    nh_.param<int>("serial_baud", baud, 115200);
                    serial_port_->setPort(dev);
                    serial_port_->setBaudrate(baud);
                    // 先 close 再 open，避免某些驱动状态错误
                    if (serial_port_->isOpen()) serial_port_->close();
                    serial_port_->open();
                    if (serial_port_->isOpen()) {
                        ROS_INFO("Serial reconnected: %s @ %d", dev.c_str(), baud);
                    }
                } catch (const std::exception& e) {
                    ROS_DEBUG_THROTTLE(10.0, "Serial reconnect failed: %s", e.what());
                }
            }
            return;
        }

        // 协议定义（二进制，Little-Endian，主机字节序默认匹配x86/ARM LE）
        // [0xAA, 0x55] (header 2B)
        // [N]          (point count, uint8, max 255)
        // [x0,y0,z0, x1,y1,z1, ...] (N × 12B, float32 each)
        // [crc16]      (uint16, low byte first → little-endian)

        std::vector<uint8_t> buffer;
        buffer.reserve(2 + 1 + sectors * 12 + 2);

        // Header
        buffer.push_back(0xAA);
        buffer.push_back(0x55);

        // Count
        uint8_t count = static_cast<uint8_t>(cloud->points.size());
        if (count > sectors) {
            ROS_WARN("Too many points? count=%d, sectors=%d", count, sectors);
            count = sectors;
        }
        buffer.push_back(count);

        // Points data
        float checksum = 0.0f;
        for (size_t i = 0; i < cloud->points.size() && i < sectors; ++i) {
            const auto& pt = cloud->points[i];

            // 安全 cast：保留 const，用 const uint8_t*
            const uint8_t* px = reinterpret_cast<const uint8_t*>(&pt.x);
            const uint8_t* py = reinterpret_cast<const uint8_t*>(&pt.y);
            const uint8_t* pz = reinterpret_cast<const uint8_t*>(&pt.z);

            buffer.insert(buffer.end(), px, px + 4);
            buffer.insert(buffer.end(), py, py + 4);
            buffer.insert(buffer.end(), pz, pz + 4);

            checksum += pt.x + pt.y + pt.z;
        }

        // CRC16-like checksum: (abs(sum*1000)) mod 65536
        uint32_t raw_sum = static_cast<uint32_t>(std::abs(checksum) * 1000.0f);
        uint16_t crc = static_cast<uint16_t>(raw_sum & 0xFFFF);

        // Append CRC (little-endian: low byte first)
        buffer.push_back(static_cast<uint8_t>(crc & 0xFF));      // LSB
        buffer.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF)); // MSB

        // Send
        try {
            size_t written = serial_port_->write(buffer);
            if (written != buffer.size()) {
                ROS_WARN("Serial write incomplete: %zu / %zu bytes", written, buffer.size());
            } else {
                ROS_DEBUG("Sent %d points (%zu bytes) via %s", 
                          count, written, serial_port_->getPort().c_str());
            }
        } catch (const serial::IOException& e) {
            ROS_ERROR("Serial write IOException: %s", e.what());
            try {
                serial_port_->close();
            } catch (...) {}
        } catch (const std::exception& e) {
            ROS_ERROR("Serial write std::exception: %s", e.what());
        }
    }

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    std::unique_ptr<serial::Serial> serial_port_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "sector_min_filter");
    ros::NodeHandle nh;

    ROS_INFO("Starting SectorMinFilter Node (by Husking, HUST)...");

    SectorMinFilter node;
    ros::spin();
    return 0;
}