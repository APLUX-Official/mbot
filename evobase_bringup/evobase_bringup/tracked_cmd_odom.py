#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import time
import math
import tf2_ros
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry

# =============================================================================
# 核心控制类
# - 增加了 set_setpoint 方法来智能地重置PID状态，解决电机嗡嗡声问题。
# =============================================================================
class AdvancedPID:
    def __init__(self, kp=0, ki=0, kd=0, kf=0, out_min=-255, out_max=255, integral_limit=200, deadband=0):
        self.kp, self.ki, self.kd, self.kf = kp, ki, kd, kf
        self.out_min, self.out_max = out_min, out_max
        self.integral_limit, self.deadband = integral_limit, deadband
        self.setpoint, self.last_error, self.integral, self.last_output = 0.0, 0.0, 0.0, 0
        self.p_term, self.i_term, self.d_term, self.f_term = 0.0, 0.0, 0.0, 0.0

    def set_setpoint(self, new_setpoint):
        """设置新的目标值，并在目标从非零变为零时重置PID状态。"""
        if new_setpoint == 0.0 and self.setpoint != 0.0:
            self.reset()
        self.setpoint = new_setpoint

    def compute(self, feedback, dt):
        if dt <= 0: return self.last_output
        error = self.setpoint - feedback
        self.integral += error * dt
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        derivative = (error - self.last_error) / dt
        self.last_error = error
        self.p_term = self.kp * error
        self.i_term = self.ki * self.integral
        self.d_term = self.kd * derivative
        self.f_term = self.kf * self.setpoint
        output = self.p_term + self.i_term + self.d_term + self.f_term
        self.last_output = int(max(self.out_min, min(self.out_max, output)))
        return self.last_output

    def reset(self):
        """重置积分项、误差历史和所有PID分项及输出。"""
        self.last_error, self.integral, self.last_output = 0.0, 0.0, 0
        self.p_term, self.i_term, self.d_term, self.f_term = 0.0, 0.0, 0.0, 0.0

# =============================================================================
# 里程计计算类
# - 使用 "有效轮距" (effective_wheel_base) 来校准转向。
# =============================================================================
class DifferentialDriveOdometry:
    def __init__(self, wheel_base, wheel_radius, turning_correction_factor=1.0):
        self.effective_wheel_base = wheel_base * turning_correction_factor
        self.wheel_radius = wheel_radius
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.last_left_angle, self.last_right_angle = 0.0, 0.0
        self.initialized = False

    def update(self, left_angle, right_angle):
        if not self.initialized:
            self.last_left_angle, self.last_right_angle = left_angle, right_angle
            self.initialized = True
            return 0.0, 0.0
        delta_left_dist = (left_angle - self.last_left_angle) * self.wheel_radius
        delta_right_dist = (right_angle - self.last_right_angle) * self.wheel_radius
        self.last_left_angle, self.last_right_angle = left_angle, right_angle
        delta_center = (delta_left_dist + delta_right_dist) / 2.0
        delta_theta = (delta_right_dist - delta_left_dist) / self.effective_wheel_base
        self.x += delta_center * math.cos(self.theta + delta_theta / 2.0)
        self.y += delta_center * math.sin(self.theta + delta_theta / 2.0)
        self.theta = (self.theta + delta_theta + math.pi) % (2 * math.pi) - math.pi
        return delta_center, delta_theta

# =============================================================================
# ROS 2 节点主类
# =============================================================================
class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')
        
        self._declare_and_load_params()
        
        self.ser_left = self._open_serial(self.left_port)
        self.ser_right = self._open_serial(self.right_port)
        self.ser_motor = self._open_serial(self.motor_port)
        if not all([self.ser_left, self.ser_right, self.ser_motor]):
            self.get_logger().error("未能打开所有串口，节点将退出。"); rclpy.shutdown(); return
        
        time.sleep(2)

        pid_params = self.pid_config
        self.pids = {'L': AdvancedPID(**pid_params), 'R': AdvancedPID(**pid_params)}
        self.odometry = DifferentialDriveOdometry(self.wheel_base, self.wheel_radius, self.turning_correction_factor)
        
        self._send_pwm_command(0, 0)

        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.control_timer = self.create_timer(0.01, self.control_loop_callback)
        self.last_control_time = self.get_clock().now().nanoseconds / 1e9
        
        self.last_cmd_vel_time = self.get_clock().now()
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_callback)
        
        self.get_logger().info("PID履带车控制器节点已启动。")

    def _declare_and_load_params(self):
        # 声明所有参数
        self.declare_parameter('serial_ports.left', '/dev/ttyUSB1')
        self.declare_parameter('serial_ports.right', '/dev/ttyUSB2')
        self.declare_parameter('serial_ports.motor', '/dev/ttyUSB0')
        self.declare_parameter('robot_params.wheel_base', 0.35)
        self.declare_parameter('robot_params.wheel_radius', 0.11)
        self.declare_parameter('robot_params.turning_correction_factor', 1.0)
        self.declare_parameter('pid_params.kp', 150.0)
        self.declare_parameter('pid_params.ki', 300.0)
        self.declare_parameter('pid_params.kd', 2.0)
        self.declare_parameter('pid_params.kf', 30.0)
        self.declare_parameter('pid_params.out_min', -255)
        self.declare_parameter('pid_params.out_max', 255)
        self.declare_parameter('pid_params.integral_limit', 200)
        self.declare_parameter('pid_params.deadband', 15)
        self.declare_parameter('publish_tf', True) 
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('cmd_vel_timeout', 0.5)
        
        # 获取参数值
        self.left_port = self.get_parameter('serial_ports.left').value
        self.right_port = self.get_parameter('serial_ports.right').value
        self.motor_port = self.get_parameter('serial_ports.motor').value
        self.wheel_base = self.get_parameter('robot_params.wheel_base').value
        self.wheel_radius = self.get_parameter('robot_params.wheel_radius').value
        self.turning_correction_factor = self.get_parameter('robot_params.turning_correction_factor').value

        self.pid_config = {
            'kp': self.get_parameter('pid_params.kp').value, 'ki': self.get_parameter('pid_params.ki').value,
            'kd': self.get_parameter('pid_params.kd').value, 'kf': self.get_parameter('pid_params.kf').value,
            'out_min': self.get_parameter('pid_params.out_min').value, 'out_max': self.get_parameter('pid_params.out_max').value,
            'integral_limit': self.get_parameter('pid_params.integral_limit').value,
            'deadband': self.get_parameter('pid_params.deadband').value
        }
        self.deadband = self.pid_config['deadband'] 

        self.publish_tf = self.get_parameter('publish_tf').value
        self.base_frame = self.get_parameter('base_frame').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        self.last_data = {'L_speed_rad': 0.0, 'R_speed_rad': 0.0, 'L_angle': 0.0, 'R_angle': 0.0}

        # 打印加载后的参数
        self.get_logger().info("="*50)
        self.get_logger().info("PID控制器节点参数已加载:")
        self.get_logger().info(f"  - 串口 (Left): {self.left_port}, (Right): {self.right_port}, (Motor): {self.motor_port}")
        self.get_logger().info(f"  - 机器人: 轮距={self.wheel_base}m, 轮半径={self.wheel_radius}m, 转向校准系数={self.turning_correction_factor}")
        for key, value in self.pid_config.items():
            self.get_logger().info(f"  - PID - {key}: {value}")
        self.get_logger().info(f"  - 行为 - 发布TF: {self.publish_tf}, 基座标系: '{self.base_frame}', 超时: {self.cmd_vel_timeout}s")
        effective_wheel_base = self.wheel_base * self.turning_correction_factor
        self.get_logger().info(f"  - 校准结果 - 有效轮距: {effective_wheel_base:.3f}m")
        self.get_logger().info("="*50)

    def control_loop_callback(self):
        current_time_ns = self.get_clock().now().nanoseconds
        dt = (current_time_ns / 1e9) - self.last_control_time
        if dt <= 0: return
        self.last_control_time = current_time_ns / 1e9
        
        l_omega_raw, l_angle_raw = self._read_sensor_data(self.ser_left)
        r_omega_raw, r_angle_raw = self._read_sensor_data(self.ser_right)
        
        if l_omega_raw is not None: self.last_data['L_speed_rad'] = l_omega_raw
        if l_angle_raw is not None: self.last_data['L_angle'] = l_angle_raw
        if r_omega_raw is not None: self.last_data['R_speed_rad'] = r_omega_raw
        if r_angle_raw is not None: self.last_data['R_angle'] = r_angle_raw
        
        left_speed = self.last_data['L_speed_rad'] * self.wheel_radius
        right_speed = -self.last_data['R_speed_rad'] * self.wheel_radius
        
        left_pwm = self.pids['L'].compute(left_speed, dt)
        right_pwm = self.pids['R'].compute(right_speed, dt)
        
        self._send_pwm_command(left_pwm, right_pwm)
        self._update_and_publish_odom(dt)
        
        lp, rp = self.pids['L'], self.pids['R']
        odom_x, odom_y, odom_theta_deg = self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)
        
        log_str = (
            f"L:[SP:{lp.setpoint:.2f} FB:{left_speed:.2f} PWM:{left_pwm:4d}] | "
            f"R:[SP:{rp.setpoint:.2f} FB:{right_speed:.2f} PWM:{right_pwm:4d}] | "
            f"Odom: x={odom_x:.3f}, y={odom_y:.3f}, theta={odom_theta_deg:.2f}°"
        )
        self.get_logger().debug(log_str) # 改为debug级别，避免在正常运行时刷屏

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_vel_time = self.get_clock().now()
        v_l = msg.linear.x - msg.angular.z * self.wheel_base / 2.0
        v_r = msg.linear.x + msg.angular.z * self.wheel_base / 2.0
        self.pids['L'].set_setpoint(v_l)
        self.pids['R'].set_setpoint(v_r)

    def watchdog_callback(self):
        if (self.get_clock().now() - self.last_cmd_vel_time).nanoseconds / 1e9 > self.cmd_vel_timeout:
            if self.pids['L'].setpoint != 0.0 or self.pids['R'].setpoint != 0.0:
                 self.get_logger().warn(f"超时未收到 cmd_vel，正在停止电机...")
                 self.pids['L'].set_setpoint(0.0)
                 self.pids['R'].set_setpoint(0.0)

    def _update_and_publish_odom(self, dt):
        delta_s, delta_theta = self.odometry.update(self.last_data['L_angle'], -self.last_data['R_angle'])
        now = self.get_clock().now().to_msg()
        
        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = self.base_frame
        
        odom_msg.pose.pose.position.x = self.odometry.x
        odom_msg.pose.pose.position.y = self.odometry.y
        q = self._euler_to_quaternion(0, 0, self.odometry.theta)
        odom_msg.pose.pose.orientation = q
        
        # 使用从控制循环传递来的dt来精确计算速度
        odom_msg.twist.twist.linear.x = delta_s / dt
        odom_msg.twist.twist.angular.z = delta_theta / dt
        
        self.odom_pub.publish(odom_msg)
        
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = "odom"
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.odometry.x
            t.transform.translation.y = self.odometry.y
            t.transform.rotation = q
            self.tf_broadcaster.sendTransform(t)
            
    def _read_sensor_data(self, ser_obj):
        last_line = ""
        if not ser_obj or not ser_obj.is_open: return None, None
        try:
            while ser_obj.in_waiting > 0:
                line = ser_obj.readline().decode('utf-8', 'ignore').strip()
                if line: last_line = line
        except (serial.SerialException, UnicodeDecodeError):
            if ser_obj and ser_obj.is_open: ser_obj.read(ser_obj.in_waiting)
            return None, None
        if last_line:
            try:
                parts = last_line.split(',')
                if len(parts) == 2: return float(parts[0]), float(parts[1])
            except (ValueError, IndexError): pass
        return None, None

    def _send_pwm_command(self, left_pwm, right_pwm):
        final_left_pwm, final_right_pwm = int(left_pwm), int(right_pwm)
        
        is_stopped_mode = (self.pids['L'].setpoint == 0 and self.pids['R'].setpoint == 0)
        if is_stopped_mode:
            if abs(left_pwm) < self.deadband: final_left_pwm = 0
            if abs(right_pwm) < self.deadband: final_right_pwm = 0

        if self.ser_motor and self.ser_motor.is_open:
            try:
                cmd = f"{final_left_pwm},{final_right_pwm}\n"
                self.ser_motor.write(cmd.encode('utf-8'))
            except serial.SerialException as e:
                self.get_logger().error(f"发送PWM指令时出错: {e}")

    def _open_serial(self, port, baudrate=115200):
        try:
            ser = serial.Serial(port, baudrate, timeout=0.1)
            self.get_logger().info(f"成功打开串口: {port}")
            return ser
        except serial.SerialException as e:
            self.get_logger().error(f"未能打开串口 {port}: {e}")
            return None

    def _euler_to_quaternion(self, roll, pitch, yaw):
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

    def destroy_node(self):
        self.get_logger().info("节点正在关闭，停止电机...")
        self._send_pwm_command(0, 0)
        time.sleep(0.1)
        for ser_name in ['ser_left', 'ser_right', 'ser_motor']:
            ser = getattr(self, ser_name, None)
            if ser and ser.is_open:
                ser.close()
                self.get_logger().info(f"串口 {ser.port} 已关闭。")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    if rclpy.ok():
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()