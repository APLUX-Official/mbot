/*
 * PLICP激光雷达里程计 - 实现文件 (轮式里程计融合版)
 */

#include "plicp_odometry_ros2/plicp_odometry.hpp"

namespace plicp_odometry_ros2
{

// 构造函数
PlicpOdometry::PlicpOdometry(const rclcpp::NodeOptions & options)
: Node("plicp_odometry", options)
{
    RCLCPP_INFO(this->get_logger(), "\033[1;32m----> PLICP odometry (Wheel Fusion) started.\033[0m");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    // 订阅激光雷达
    laser_scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 
        rclcpp::SensorDataQoS(), 
        std::bind(&PlicpOdometry::scanCallback, this, std::placeholders::_1));

    // 【新增】订阅轮式里程计 /odom_wheel
    wheel_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom_wheel", 
        rclcpp::QoS(10),
        std::bind(&PlicpOdometry::wheelOdomCallback, this, std::placeholders::_1));

    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 50);

    initParams();

    scan_count_ = 0;
    initialized_ = false;
    has_wheel_data_ = false; 

    base_in_odom_.setIdentity();
    base_in_odom_keyframe_.setIdentity();

    // 初始化 CSM 输入参数
    input_.laser[0] = 0.0;
    input_.laser[1] = 0.0;
    input_.laser[2] = 0.0;
    output_.cov_x_m = nullptr;
    output_.dx_dy1_m = nullptr;
    output_.dx_dy2_m = nullptr;

    // 初始化速度变量
    latest_velocity_.linear.x = 0.0; latest_velocity_.linear.y = 0.0; latest_velocity_.linear.z = 0.0;
    latest_velocity_.angular.x = 0.0; latest_velocity_.angular.y = 0.0; latest_velocity_.angular.z = 0.0;
    
    current_wheel_twist_.linear.x = 0.0; current_wheel_twist_.angular.z = 0.0;
}

PlicpOdometry::~PlicpOdometry()
{
    if (output_.cov_x_m) gsl_matrix_free(output_.cov_x_m);
    if (output_.dx_dy1_m) gsl_matrix_free(output_.dx_dy1_m);
    if (output_.dx_dy2_m) gsl_matrix_free(output_.dx_dy2_m);
}

void PlicpOdometry::initParams()
{
    odom_frame_ = this->declare_parameter("odom_frame", "odom");
    base_frame_ = this->declare_parameter("base_frame", "base_link");
    
    kf_dist_linear_ = this->declare_parameter("kf_dist_linear", 0.1);
    kf_dist_angular_ = this->declare_parameter("kf_dist_angular", 5.0 * (M_PI / 180.0));
    kf_dist_linear_sq_ = kf_dist_linear_ * kf_dist_linear_;
    kf_scan_count_ = this->declare_parameter("kf_scan_count", 10);

    // CSM 参数 - 针对履带车优化的宽松参数
    input_.max_angular_correction_deg = this->declare_parameter("max_angular_correction_deg", 90.0);
    input_.max_linear_correction = this->declare_parameter("max_linear_correction", 1.0);
    input_.max_iterations = this->declare_parameter("max_iterations", 20);
    input_.epsilon_xy = this->declare_parameter("epsilon_xy", 0.000001);
    input_.epsilon_theta = this->declare_parameter("epsilon_theta", 0.000001);
    input_.max_correspondence_dist = this->declare_parameter("max_correspondence_dist", 1.0);
    input_.sigma = this->declare_parameter("sigma", 0.010);
    input_.use_corr_tricks = this->declare_parameter("use_corr_tricks", 1);
    input_.restart = this->declare_parameter("restart", 1);
    input_.restart_threshold_mean_error = this->declare_parameter("restart_threshold_mean_error", 0.05);
    input_.restart_dt = this->declare_parameter("restart_dt", 1.0);
    input_.restart_dtheta = this->declare_parameter("restart_dtheta", 0.1);
    
    // 其他高级参数
    input_.clustering_threshold = this->declare_parameter("clustering_threshold", 0.25);
    input_.orientation_neighbourhood = this->declare_parameter("orientation_neighbourhood", 20);
    input_.use_point_to_line_distance = this->declare_parameter("use_point_to_line_distance", 1);
    input_.do_alpha_test = this->declare_parameter("do_alpha_test", 0);
    input_.do_alpha_test_thresholdDeg = this->declare_parameter("do_alpha_test_thresholdDeg", 20.0);
    input_.outliers_maxPerc = this->declare_parameter("outliers_maxPerc", 0.90);
    input_.outliers_adaptive_order = this->declare_parameter("outliers_adaptive_order", 0.7);
    input_.outliers_adaptive_mult = this->declare_parameter("outliers_adaptive_mult", 2.0);
    input_.do_visibility_test = this->declare_parameter("do_visibility_test", 0);
    input_.outliers_remove_doubles = this->declare_parameter("outliers_remove_doubles", 1);
    input_.do_compute_covariance = this->declare_parameter("do_compute_covariance", 0);
    input_.debug_verify_tricks = this->declare_parameter("debug_verify_tricks", 0);
    input_.use_ml_weights = this->declare_parameter("use_ml_weights", 0);
    input_.use_sigma_weights = this->declare_parameter("use_sigma_weights", 0);
}

// 【新增】轮式里程计回调
void PlicpOdometry::wheelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // 只取速度 Twist
    current_wheel_twist_ = msg->twist.twist;
    has_wheel_data_ = true;
}

void PlicpOdometry::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
    current_time_ = scan_msg->header.stamp;
    
    if (!initialized_) {
        createCache(scan_msg);
        if (!getBaseToLaserTf(scan_msg->header.frame_id)) {
            RCLCPP_WARN(this->get_logger(), "Waiting for TF (base->laser)...");
            return;
        }
        laserScanToLDP(scan_msg, prev_ldp_scan_);
        last_icp_time_ = current_time_;
        initialized_ = true;
        return;
    }

    LDP curr_ldp_scan;
    laserScanToLDP(scan_msg, curr_ldp_scan);

    scanMatchWithPLICP(curr_ldp_scan, current_time_);
}

void PlicpOdometry::createCache(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
    a_cos_.clear();
    a_sin_.clear();
    double angle;
    for (size_t i = 0; i < scan_msg->ranges.size(); i++) {
        angle = scan_msg->angle_min + static_cast<double>(i) * scan_msg->angle_increment;
        a_cos_.push_back(cos(angle));
        a_sin_.push_back(sin(angle));
    }
    input_.min_reading = scan_msg->range_min;
    input_.max_reading = scan_msg->range_max;
}

bool PlicpOdometry::getBaseToLaserTf(const std::string & frame_id)
{
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
        transform_stamped = tf_buffer_->lookupTransform(
            base_frame_, frame_id, this->now(), tf2::durationFromSec(1.0));
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what());
        return false;
    }
    tf2::fromMsg(transform_stamped.transform, base_to_laser_);
    laser_to_base_ = base_to_laser_.inverse();
    return true;
}

void PlicpOdometry::laserScanToLDP(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg, LDP & ldp)
{
    unsigned int n = scan_msg->ranges.size();
    ldp = ld_alloc_new(n);

    for (unsigned int i = 0; i < n; i++) {
        double r = scan_msg->ranges[i];
        if (r > scan_msg->range_min && r < scan_msg->range_max) {
            ldp->valid[i] = 1;
            ldp->readings[i] = r;
        } else {
            ldp->valid[i] = 0;
            ldp->readings[i] = -1;
        }
        ldp->theta[i] = scan_msg->angle_min + static_cast<double>(i) * scan_msg->angle_increment;
        ldp->cluster[i] = -1;
    }
    ldp->min_theta = ldp->theta[0];
    ldp->max_theta = ldp->theta[n - 1];
    
    ldp->odometry[0] = 0.0; ldp->odometry[1] = 0.0; ldp->odometry[2] = 0.0;
    ldp->true_pose[0] = 0.0; ldp->true_pose[1] = 0.0; ldp->true_pose[2] = 0.0;
}

// 核心匹配逻辑
void PlicpOdometry::scanMatchWithPLICP(LDP & curr_ldp_scan, const rclcpp::Time & time)
{
    prev_ldp_scan_->odometry[0] = 0.0;
    prev_ldp_scan_->odometry[1] = 0.0;
    prev_ldp_scan_->odometry[2] = 0.0;
    prev_ldp_scan_->estimate[0] = 0.0;
    prev_ldp_scan_->estimate[1] = 0.0;
    prev_ldp_scan_->estimate[2] = 0.0;
    prev_ldp_scan_->true_pose[0] = 0.0;
    prev_ldp_scan_->true_pose[1] = 0.0;
    prev_ldp_scan_->true_pose[2] = 0.0;

    input_.laser_ref = prev_ldp_scan_;
    input_.laser_sens = curr_ldp_scan;

    // 1. 获取预测量
    double dt = (time - last_icp_time_).seconds();
    double pr_ch_x, pr_ch_y, pr_ch_a;
    getPrediction(pr_ch_x, pr_ch_y, pr_ch_a, dt);

    tf2::Transform prediction_change;
    createTfFromXYTheta(pr_ch_x, pr_ch_y, pr_ch_a, prediction_change);
    
    // 转换到 Laser 坐标系
    prediction_change = prediction_change * (base_in_odom_ * base_in_odom_keyframe_.inverse());
    tf2::Transform prediction_change_lidar;
    prediction_change_lidar = laser_to_base_ * base_in_odom_.inverse() * prediction_change * base_in_odom_ * base_to_laser_;

    input_.first_guess[0] = prediction_change_lidar.getOrigin().getX();
    input_.first_guess[1] = prediction_change_lidar.getOrigin().getY();
    input_.first_guess[2] = tf2::getYaw(prediction_change_lidar.getRotation());

    // 释放旧内存
    if (output_.cov_x_m) { gsl_matrix_free(output_.cov_x_m); output_.cov_x_m = nullptr; }
    if (output_.dx_dy1_m) { gsl_matrix_free(output_.dx_dy1_m); output_.dx_dy1_m = nullptr; }
    if (output_.dx_dy2_m) { gsl_matrix_free(output_.dx_dy2_m); output_.dx_dy2_m = nullptr; }
    
    // 2. 执行算法
    sm_icp(&input_, &output_);

    tf2::Transform corr_ch;

    if (output_.valid) {
        // === 匹配成功 ===
        tf2::Transform corr_ch_l;
        createTfFromXYTheta(output_.x[0], output_.x[1], output_.x[2], corr_ch_l);
        corr_ch = base_to_laser_ * corr_ch_l * laser_to_base_;

        latest_velocity_.linear.x = corr_ch.getOrigin().getX() / dt;
        latest_velocity_.angular.z = tf2::getYaw(corr_ch.getRotation()) / dt;
    } else {
        // === 匹配失败，使用预测值 (Dead Reckoning) ===
        RCLCPP_WARN(this->get_logger(), "PLICP Failed/Drop (dt=%.3f), using Wheel prediction!", dt);
        
        // 重构预测的变换矩阵 (prediction_change_x/y/a 是基于轮速算出来的)
        tf2::Transform dr_pose;
        createTfFromXYTheta(pr_ch_x, pr_ch_y, pr_ch_a, dr_pose);
        corr_ch = dr_pose;
        
        // 不更新 latest_velocity_，保持惯性
    }

    base_in_odom_ = base_in_odom_keyframe_ * corr_ch;
    
    publishTFAndOdometry();

    if (newKeyframeNeeded(corr_ch)) {
        ld_free(prev_ldp_scan_);
        prev_ldp_scan_ = curr_ldp_scan;
        base_in_odom_keyframe_ = base_in_odom_;
    } else {
        ld_free(curr_ldp_scan);
    }

    last_icp_time_ = time;
}

void PlicpOdometry::getPrediction(double & prediction_change_x,
                                 double & prediction_change_y,
                                 double & prediction_change_angle,
                                 double dt)
{
    double vx = 0.0;
    double vy = 0.0;
    double vth = 0.0;

    // 【核心】优先使用轮式里程计速度
    if (has_wheel_data_) {
        vx = current_wheel_twist_.linear.x;
        vy = current_wheel_twist_.linear.y;
        vth = current_wheel_twist_.angular.z;
    } else {
        vx = latest_velocity_.linear.x;
        vy = latest_velocity_.linear.y;
        vth = latest_velocity_.angular.z;
    }
    
    // 去除极小值噪声
    if (std::abs(vx) < 1e-4) vx = 0.0;
    if (std::abs(vth) < 1e-4) vth = 0.0;

    prediction_change_x = vx * dt;
    prediction_change_y = vy * dt;
    prediction_change_angle = vth * dt;

    if (prediction_change_angle >= M_PI) prediction_change_angle -= 2.0 * M_PI;
    else if (prediction_change_angle < -M_PI) prediction_change_angle += 2.0 * M_PI;
}

void PlicpOdometry::createTfFromXYTheta(double x, double y, double theta, tf2::Transform & t)
{
    t.setOrigin(tf2::Vector3(x, y, 0.0));
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta);
    t.setRotation(q);
}

void PlicpOdometry::publishTFAndOdometry()
{
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = current_time_;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;
    tf_msg.transform = tf2::toMsg(base_in_odom_);
    tf_broadcaster_->sendTransform(tf_msg);

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_time_;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;
    tf2::toMsg(base_in_odom_, odom_msg.pose.pose);
    
    if (has_wheel_data_) {
        odom_msg.twist.twist = current_wheel_twist_;
    } else {
        odom_msg.twist.twist = latest_velocity_;
    }
    odom_publisher_->publish(odom_msg);
}

bool PlicpOdometry::newKeyframeNeeded(const tf2::Transform & d)
{
    scan_count_++;
    if (std::abs(tf2::getYaw(d.getRotation())) > kf_dist_angular_) return true;
    if (scan_count_ >= kf_scan_count_) {
        scan_count_ = 0;
        return true;
    }
    double x = d.getOrigin().getX();
    double y = d.getOrigin().getY();
    if (x * x + y * y > kf_dist_linear_sq_) return true;
    return false;
}

}  // namespace plicp_odometry_ros2

// 注册宏必须在 namespace 之外
RCLCPP_COMPONENTS_REGISTER_NODE(plicp_odometry_ros2::PlicpOdometry)