#include <cmath>
#include <algorithm>
#include <cstdint>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class LidarMappingNode : public rclcpp::Node
{
public:
    LidarMappingNode() : Node("lidar_mapping_node")
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&LidarMappingNode::scanCallback, this, std::placeholders::_1));

        auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1))
            .transient_local()
            .reliable();

        map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/local_map",
            map_qos
        );

        initMap();

        RCLCPP_INFO(get_logger(), "lidar_mapping_node started");
    }

private:
    double resolution_ = 0.05;
    double map_size_m_ = 10.0;

    int width_ = 200;
    int height_ = 200;

    double origin_x_ = -5.0;
    double origin_y_ = -5.0;

    nav_msgs::msg::OccupancyGrid map_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    bool worldToGrid(double wx, double wy, int & gx, int & gy) const
    {
        gx = static_cast<int>((wx - origin_x_) / resolution_);
        gy = static_cast<int>((wy - origin_y_) / resolution_);

        return gx >= 0 && gx < width_ && gy >= 0 && gy < height_;
    }

    void setCellIfValid(int gx, int gy, int8_t value)
    {
        if (gx < 0 || gx >= width_ || gy < 0 || gy >= height_) {
            return;
        }

        map_.data[gy * width_ + gx] = value;
    }

    void markRayFree(int x0, int y0, int x1, int y1)
    {
        // Bresenham 라인 알고리즘으로 라이다 빔이 지나간 셀을 free(0)로 표시한다.
        int dx = std::abs(x1 - x0);
        int dy = std::abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;

        int x = x0;
        int y = y0;

        while (x != x1 || y != y1) {
            setCellIfValid(x, y, 0);

            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x += sx;
            }
            if (e2 < dx) {
                err += dx;
                y += sy;
            }
        }
    }

    void initMap()
    {
        width_ = static_cast<int>(map_size_m_ / resolution_);
        height_ = static_cast<int>(map_size_m_ / resolution_);

        map_.header.frame_id = "odom";

        map_.info.resolution = resolution_;
        map_.info.width = width_;
        map_.info.height = height_;

        map_.info.origin.position.x = origin_x_;
        map_.info.origin.position.y = origin_y_;
        map_.info.origin.position.z = 0.0;
        map_.info.origin.orientation.w = 1.0;

        // 처음에는 전부 unknown(-1)으로 초기화한다.
        map_.data.assign(width_ * height_, -1);
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped tf_msg;
        try {
            // 스캔이 측정된 정확한 시각의 odom -> lidar 프레임 변환을 가져온다.
            // 이렇게 해야 로봇이 움직이는 중에도 scan과 pose의 시간 불일치가 줄어든다.
            tf_msg = tf_buffer_->lookupTransform(
                "odom",
                msg->header.frame_id.empty() ? "lidar_link" : msg->header.frame_id,
                msg->header.stamp,
                rclcpp::Duration::from_seconds(0.1));
        } catch (const std::exception & e) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "TF lookup failed: %s", e.what());
            return;
        }

        const double robot_x = tf_msg.transform.translation.x;
        const double robot_y = tf_msg.transform.translation.y;
        const auto & q = tf_msg.transform.rotation;
        const double robot_yaw = std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z));

        double angle = msg->angle_min;
        int rx = 0;
        int ry = 0;

        if (!worldToGrid(robot_x, robot_y, rx, ry)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Robot pose is outside map bounds.");
            return;
        }

        for (size_t i = 0; i < msg->ranges.size(); ++i, angle += msg->angle_increment) {
            double r = msg->ranges[i];

            if (!std::isfinite(r)) continue;
            if (r < msg->range_min || r > msg->range_max) continue;

            double lx = r * std::cos(angle);
            double ly = r * std::sin(angle);

            double wx = robot_x
                      + std::cos(robot_yaw) * lx
                      - std::sin(robot_yaw) * ly;

            double wy = robot_y
                      + std::sin(robot_yaw) * lx
                      + std::cos(robot_yaw) * ly;

            int gx = 0;
            int gy = 0;
            if (!worldToGrid(wx, wy, gx, gy)) {
                continue;
            }

            // 빔이 지나간 구간은 free, 끝점은 occupied로 누적한다.
            markRayFree(rx, ry, gx, gy);
            setCellIfValid(gx, gy, 100);
        }

        // 로봇이 현재 있는 셀은 free로 유지한다.
        setCellIfValid(rx, ry, 0);

        map_.header.stamp = now();
        map_pub_->publish(map_);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarMappingNode>());
    rclcpp::shutdown();
    return 0;
}
