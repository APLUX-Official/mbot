/*
 * PLICP激光雷达里程计 - ROS2版本 (包含轮式里程计融合)
 */

#ifndef PLICP_ODOMETRY_ROS2__PLICP_ODOMETRY_HPP_
#define PLICP_ODOMETRY_ROS2__PLICP_ODOMETRY_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

// ROS2核心
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

// 消息类型
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

// TF2
#include <tf2/utils.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // 注意这里推荐用 .hpp

// CSM库
#include <csm/csm_all.h>
#undef min
#undef max

namespace plicp_odometry_ros2
{

class PlicpOdometry : public rclcpp::Node
{
public:
    /**
     * 构造函数
     * 修改点：恢复了默认参数 = rclcpp::NodeOptions()，解决了 standalone node 编译报错的问题
     */
    explicit PlicpOdometry(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    virtual ~PlicpOdometry();

private:
    // === ROS2通信组件 ===
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscription_;
    
    // 【新增】轮式里程计订阅
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_;
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;

    // === 时间管理 ===
    rclcpp::Time last_icp_time_;
    rclcpp::Time current_time_;

    // === 运动状态估计 ===
    geometry_msgs::msg::Twist latest_velocity_; 

    // 【新增】轮式里程计数据缓存
    geometry_msgs::msg::Twist current_wheel_twist_; 
    bool has_wheel_data_; // 标记是否收到过轮式里程计

    // === TF2坐标变换系统 ===
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // === 坐标系间的固定变换 ===
    tf2::Transform base_to_laser_;
    tf2::Transform laser_to_base_;

    // === 位姿状态变量 ===
    tf2::Transform base_in_odom_;
    tf2::Transform base_in_odom_keyframe_;

    // === 系统状态参数 ===
    bool initialized_;

    // === 配置参数 ===
    std::string odom_frame_;
    std::string base_frame_;

    // === 关键帧检测参数 ===
    double kf_dist_linear_;
    double kf_dist_linear_sq_;
    double kf_dist_angular_;
    int kf_scan_count_;
    int scan_count_;

    // === 性能优化缓存 ===
    std::vector<double> a_cos_;
    std::vector<double> a_sin_;

    // === CSM库接口数据结构 ===
    sm_params input_;
    sm_result output_;
    LDP prev_ldp_scan_;

    // === 私有方法声明 ===
    void initParams();
    void createCache(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
    bool getBaseToLaserTf(const std::string & frame_id);
    void laserScanToLDP(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg, LDP & ldp);
    void scanMatchWithPLICP(LDP & curr_ldp_scan, const rclcpp::Time & time);
    void getPrediction(double & prediction_change_x, double & prediction_change_y, 
                      double & prediction_change_angle, double dt);
    void createTfFromXYTheta(double x, double y, double theta, tf2::Transform & t);
    void publishTFAndOdometry();
    bool newKeyframeNeeded(const tf2::Transform & d);
    
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
    // 【新增】轮式里程计回调
    void wheelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
};

}  // namespace plicp_odometry_ros2

#endif  // PLICP_ODOMETRY_ROS2__PLICP_ODOMETRY_HPP_