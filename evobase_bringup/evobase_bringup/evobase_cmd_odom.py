#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import time
import math
import os
import yaml
import tf2_ros
from geometry_msgs.msg import Twist # 用于接收 cmd_vel 消息
from nav_msgs.msg import Odometry # 用于发布里程计消息
from geometry_msgs.msg import Quaternion # 用于里程计中的姿态四元数
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped  # 新增导入 tf2_ros 和 TransformStamped



# 全局变量定义，将在类初始化时设置
SERIAL_PORT_LEFT = None
SERIAL_PORT_RIGHT = None
BAUDRATE = 115200
WHEEL_BASE = None
WHEEL_RADIUS = None
ENCODER_CPR = None
GEAR_RATIO = None
MOTOR_CONTROL_DT = 0.010 # 电机驱动控制周期 (秒)
WHEEL_CIRCUMFERENCE = None
PULSE_CONVERSION_FACTOR = None
MIN_START_PULSES = 0

def load_config():
    """加载配置文件"""
    # 获取包的共享目录，确保配置文件 config.yaml 能够被正确找到
    try:
        package_share_directory = get_package_share_directory('evobase_bringup')
        config_path = os.path.join(package_share_directory, 'config', 'evobase_config.yaml')
    except Exception as e:
        # 如果在开发环境中，尝试使用相对路径
        config_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'config', 'evobase_config.yaml')

    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        return config, config_path
    except FileNotFoundError:
        print(f"错误: 配置文件 '{config_path}' 未找到。请确保文件存在。", flush=True)
        exit()
    except yaml.YAMLError as e:
        print(f"错误: 解析配置文件时出错: {e}", flush=True)
        exit()

class CmdVelSerialCommander(Node):
    def __init__(self):
        super().__init__('cmd_vel_serial_commander_no_comp')
        
        # 声明参数
        self.declare_parameter('publish_tf', True)  # 默认发布TF变换
        self.declare_parameter('odom_topic')  # odom话题，由launch文件控制
        self.declare_parameter('base_frame')  # 默认base坐标系，由launch文件控制
        
        # 获取参数值
        self.publish_tf = self.get_parameter('publish_tf').value
        self.odom_topic = self.get_parameter('odom_topic').value or 'odom'
        self.base_frame = self.get_parameter('base_frame').value or 'base_footprint'
        
        self.get_logger().info(f"odom_topic: {self.odom_topic}")
        self.get_logger().info(f"base_frame: {self.base_frame}")
        
        # 首先加载配置
        self._load_and_initialize_config()
        
        # --- 串口初始化 ---
        self.ser_left = self._open_serial(self.SERIAL_PORT_LEFT)
        self.ser_right = self._open_serial(self.SERIAL_PORT_RIGHT)

        if self.ser_left is None or self.ser_right is None:
            self.get_logger().error("未能打开一个或两个串口。节点将退出。")
            rclpy.shutdown()  # 自动关闭节点和进程

        # 给电机驱动一些时间进行初始化
        time.sleep(2)
        # sleep期间串口可能堆积数据，再次清空缓冲区
        if self.ser_left and self.ser_left.is_open:
            self.ser_left.reset_input_buffer()
        if self.ser_right and self.ser_right.is_open:
            self.ser_right.reset_input_buffer()
        self._stop_motors(self.ser_left, self.ser_right) # 启动时确保电机停止

        # --- ROS 2 订阅器 ---
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10 # QoS 历史深度
        )
        self.subscription  # 防止未使用的变量警告

        # --- 可选: 用于读取编码器反馈的定时器 (用于调试/监控) ---
        self.encoder_read_timer = self.create_timer(0.1, self.read_and_print_encoders)

        # 里程计初始化
        self._init_odom()

        # --- 看门狗机制初始化 ---
        self.last_cmd_vel_time = self.get_clock().now() # 初始化为当前时间
        self.cmd_vel_timeout_duration = 0.5 # cmd_vel 消息超时时间（秒），可以根据需要调整
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_callback) # 看门狗定时器，每0.1秒检查一次

        # 新增：TF 广播器（仅在需要时创建）
        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
            self.get_logger().info(f"TF广播器已启用，将发布 {self.odom_topic} -> {self.base_frame} 变换")
        else:
            self.tf_broadcaster = None
            self.get_logger().info("TF广播器已禁用，不会发布变换")

        self.get_logger().info("ROS 2 Cmd Vel 串口控制器节点已启动。等待 /cmd_vel 消息...")

    def _open_serial(self, port):
        """辅助函数，用于打开串口。"""
        try:
            ser = serial.Serial(port, BAUDRATE, timeout=0.5)
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            self.get_logger().info(f"成功打开串口: {port}")
            return ser
        except serial.SerialException as e:
            self.get_logger().error(f"未能打开串口 {port}: {e}")
            return None

    def _velocity_to_encoder_pulse(self, v_linear):
        """
        将线速度 (m/s) 转换为 MOTOR_CONTROL_DT (10ms) 时间内所需的编码器脉冲数。
        """
        pulses = v_linear * PULSE_CONVERSION_FACTOR
        return int(round(pulses))

    def _cmdvel_to_wheel_targets(self, v_robot, w_robot):
        """
        将机器人线速度和角速度转换为左右轮的目标脉冲数，并乘以方向系数。
        """
        # 应用角速度方向系数以纠正旋转方向
        w_robot *= self.angular_velocity_direction
        
        # 标准差速驱动模型
        v_l = v_robot - w_robot * WHEEL_BASE / 2.0
        v_r = v_robot + w_robot * WHEEL_BASE / 2.0

        # 将线速度转换为编码器脉冲数
        left_target = self._velocity_to_encoder_pulse(v_l)
        right_target = self._velocity_to_encoder_pulse(v_r)

        # 限幅
        left_target = max(min(left_target, 60), -60)
        right_target = max(min(right_target, 60), -60)

        # 乘以方向系数，适配装配/线序/编码器方向
        left_target *= LEFT_WHEEL_DIR
        right_target *= RIGHT_WHEEL_DIR

        return left_target, right_target

    def _set_target_velocity(self, ser_obj, target_pulses):
        """
        通过串口向电机驱动发送目标脉冲数。
        """
        if ser_obj is None or not ser_obj.is_open:
            self.get_logger().warning(f"串口 {ser_obj.port if ser_obj else 'N/A'} 未打开，无法设置目标速度。")
            return False
        
        try:
            cmd = f"{int(target_pulses)}\n"
            ser_obj.write(cmd.encode('utf-8'))
            return True 
        except serial.SerialTimeoutException:
            self.get_logger().warning(f"串口 {ser_obj.port} 写入超时。")
            return False
        except Exception as e:
            self.get_logger().error(f"设置 {ser_obj.port} 目标速度时出错: {e}")
            return False

    def _stop_motors(self, ser_left, ser_right):
        """向两个电机发送停止命令 (0 脉冲)。"""
        self._set_target_velocity(ser_left, 0)
        self._set_target_velocity(ser_right, 0)
        self.get_logger().info("电机已停止。")

    def cmd_vel_callback(self, msg: Twist):
        """
        /cmd_vel 消息的回调函数。
        """
        self.last_cmd_vel_time = self.get_clock().now()

        linear_x = msg.linear.x
        angular_z = msg.angular.z

        left_target_pulses, right_target_pulses = self._cmdvel_to_wheel_targets(
            linear_x, angular_z
        )
        
        self._set_target_velocity(self.ser_left, left_target_pulses)
        self._set_target_velocity(self.ser_right, right_target_pulses)

        if left_target_pulses != 0 or right_target_pulses != 0:
            self.get_logger().info(
            f"收到 cmd_vel: v={linear_x:.2f} m/s, w={angular_z:.2f} rad/s -> "
            f"L_target={left_target_pulses}, R_target={right_target_pulses} pulse/10ms"
        )

    def watchdog_callback(self):
        """
        看门狗定时器回调函数。
        """
        time_since_last_msg = (self.get_clock().now() - self.last_cmd_vel_time).nanoseconds / 1e9

        if time_since_last_msg > self.cmd_vel_timeout_duration:
            self.get_logger().warn(
                f"超过 {self.cmd_vel_timeout_duration} 秒未收到 /cmd_vel 消息，停止电机！"
            )
            self._stop_motors(self.ser_left, self.ser_right)
            self.last_cmd_vel_time = self.get_clock().now()

    def _init_odom(self):
        """初始化里程计相关变量和发布器"""
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.left_total_ticks = 0.0
        self.right_total_ticks = 0.0
        self.last_left_total_ticks = 0.0
        self.last_right_total_ticks = 0.0
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)

    def read_and_print_encoders(self):
        """
        定时器回调函数：读取、解析并打印编码器反馈，然后发布里程计(odom)消息。
        """
        delta_l_ticks = 0.0
        delta_r_ticks = 0.0
        
        last_left_target, last_left_actual = 0.0, 0.0
        last_right_target, last_right_actual = 0.0, 0.0


        MAX_LINES_PER_CYCLE = 20  # 限制单次读取行数，防止阻塞执行线程

        # --- 左轮 ---
        if self.ser_left and self.ser_left.is_open:
            lines_read = 0
            while self.ser_left.in_waiting > 0 and lines_read < MAX_LINES_PER_CYCLE:
                try:
                    line_bytes = self.ser_left.readline()
                    lines_read += 1
                    line = line_bytes.decode('utf-8', errors='ignore').replace('\x00', '').strip()
                    if not line: continue
                    parts = line.split()
                    if len(parts) >= 2:
                        target = float(parts[0])
                        actual = float(parts[1])
                        # 乘以方向系数，适配编码器反馈方向
                        delta_l_ticks += actual * LEFT_WHEEL_DIR
                        last_left_target, last_left_actual = target, actual
                except (ValueError, IndexError):
                    self.get_logger().warning(f"解析左轮数据失败: '{line}'", throttle_duration_sec=5)
                except Exception as e:
                    self.get_logger().error(f"读取左轮串口时发生未知错误: {e}")
                    break
            # 如果读满上限仍有数据，丢弃剩余避免积压
            if self.ser_left.in_waiting > 100:
                self.ser_left.reset_input_buffer()
                self.get_logger().warning("左轮串口缓冲区积压过多，已清空", throttle_duration_sec=5)

        # --- 右轮 ---
        if self.ser_right and self.ser_right.is_open:
            lines_read = 0
            while self.ser_right.in_waiting > 0 and lines_read < MAX_LINES_PER_CYCLE:
                try:
                    line_bytes = self.ser_right.readline()
                    lines_read += 1
                    line = line_bytes.decode('utf-8', errors='ignore').replace('\x00', '').strip()
                    if not line: continue
                    parts = line.split()
                    if len(parts) >= 2:
                        target = float(parts[0])
                        actual = float(parts[1])
                        # 乘以方向系数，适配编码器反馈方向
                        delta_r_ticks += actual * RIGHT_WHEEL_DIR
                        last_right_target, last_right_actual = target, actual
                except (ValueError, IndexError):
                    self.get_logger().warning(f"解析右轮数据失败: '{line}'", throttle_duration_sec=5)
                except Exception as e:
                    self.get_logger().error(f"读取右轮串口时发生未知错误: {e}")
                    break
            # 如果读满上限仍有数据，丢弃剩余避免积压
            if self.ser_right.in_waiting > 100:
                self.ser_right.reset_input_buffer()
                self.get_logger().warning("右轮串口缓冲区积压过多，已清空", throttle_duration_sec=5)

        self.left_total_ticks += delta_l_ticks
        self.right_total_ticks += delta_r_ticks

        delta_for_odom_l = self.left_total_ticks - self.last_left_total_ticks
        delta_for_odom_r = self.right_total_ticks - self.last_right_total_ticks

        self.last_left_total_ticks = self.left_total_ticks
        self.last_right_total_ticks = self.right_total_ticks
        
        if last_left_target != 0 and last_right_target != 0:
            print(f"左轮: 目标={last_left_target:.1f}, 实际(增量)={last_left_actual:.1f} | "
                f"右轮: 目标={last_right_target:.1f}, 实际(增量)={last_right_actual:.1f}", flush=True)
            print(f"[Python累计] 左轮={self.left_total_ticks:.1f}, 右轮={self.right_total_ticks:.1f}", flush=True)
        
        self._update_and_publish_odom(delta_for_odom_l, delta_for_odom_r)

    def _update_and_publish_odom(self, delta_left_ticks, delta_right_ticks):
        """
        用脉冲数增量推算并发布odom (支持转向校准系数)。
        """
        # 脉冲数增量转距离增量
        dist_left = delta_left_ticks / (ENCODER_CPR * GEAR_RATIO) * WHEEL_CIRCUMFERENCE
        dist_right = delta_right_ticks / (ENCODER_CPR * GEAR_RATIO) * WHEEL_CIRCUMFERENCE

        # 运动学推算 (支持转向校准)
        delta_s = (dist_left + dist_right) / 2.0
        delta_theta = (dist_right - dist_left) / WHEEL_BASE
        # 应用转向校准系数
        delta_theta /= getattr(self, 'turning_correction_factor', 1.0)
        # 应用角速度方向系数以匹配 cmd_vel 的方向
        delta_theta *= self.angular_velocity_direction

        # 更新全局位姿
        self.x += delta_s * math.cos(self.theta + delta_theta / 2.0)
        self.y += delta_s * math.sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta

        now = self.get_clock().now().to_msg()

        # 发布nav_msgs/Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        q = self.euler_to_quaternion(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        # 速度估算
        dt = 0.1 # 与定时器频率一致
        vx = delta_s / dt
        vth = delta_theta / dt
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.angular.z = vth
        
        # 填充协方差
        odom_msg.pose.covariance[0] = 0.001
        odom_msg.pose.covariance[7] = 0.001
        odom_msg.pose.covariance[35] = 0.001
        odom_msg.twist.covariance[0] = 0.001
        odom_msg.twist.covariance[35] = 0.001

        self.odom_pub.publish(odom_msg)

        # 发布 TF 变换（仅在启用时）
        if self.publish_tf and self.tf_broadcaster:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'odom'
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(t)

        if abs(vx) != 0 or abs(vth) != 0:
            print(f"[ODOM] x={self.x:.3f} y={self.y:.3f} theta={self.theta:.3f} vx={vx:.3f} vth={vth:.3f}", flush=True)

    def euler_to_quaternion(self, roll, pitch, yaw):
        """将欧拉角 (roll, pitch, yaw) 转换为四元数 (x, y, z, w)"""
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def destroy_node(self):
        """节点销毁时的清理工作。"""
        self._stop_motors(self.ser_left, self.ser_right)
        
        if self.ser_left and self.ser_left.is_open:
            self.ser_left.close()
            self.get_logger().info(f"串口 {SERIAL_PORT_LEFT} 已关闭。")
        if self.ser_right and self.ser_right.is_open:
            self.ser_right.close()
            self.get_logger().info(f"串口 {SERIAL_PORT_RIGHT} 已关闭。")
        super().destroy_node()

    def _load_and_initialize_config(self):
        """加载配置文件并初始化全局变量，支持转向校准系数"""
        config, config_path = load_config()
        self.config_path = config_path

        global SERIAL_PORT_LEFT, SERIAL_PORT_RIGHT, WHEEL_BASE, WHEEL_RADIUS
        global ENCODER_CPR, GEAR_RATIO, WHEEL_CIRCUMFERENCE
        global PULSE_CONVERSION_FACTOR, MOTOR_CONTROL_DT
        global MIN_START_PULSES, LEFT_WHEEL_DIR, RIGHT_WHEEL_DIR

        SERIAL_PORT_LEFT = config['serial_ports']['left']
        SERIAL_PORT_RIGHT = config['serial_ports']['right']
        self.SERIAL_PORT_LEFT = SERIAL_PORT_LEFT
        self.SERIAL_PORT_RIGHT = SERIAL_PORT_RIGHT

        params = config['robot_params']
        WHEEL_BASE = params['wheel_base']
        WHEEL_RADIUS = params['wheel_radius']
        ENCODER_CPR = params['encoder_cpr']
        GEAR_RATIO = params['gear_ratio']
        MIN_START_PULSES = params.get('min_start_pulses', 0)

        # 轮子方向系数（从配置文件加载，若无则默认1）
        LEFT_WHEEL_DIR = params.get('left_wheel_dir', 1)
        RIGHT_WHEEL_DIR = params.get('right_wheel_dir', 1)

        # 角速度方向系数（从配置文件加载，若无则默认1）
        self.angular_velocity_direction = params.get('angular_velocity_direction', 1)

        # 转向校准系数（从配置文件加载，若无则为1.0）
        self.turning_correction_factor = params.get('turning_correction_factor', 1.0)

        WHEEL_CIRCUMFERENCE = 2 * math.pi * WHEEL_RADIUS
        PULSE_CONVERSION_FACTOR = (ENCODER_CPR * GEAR_RATIO / WHEEL_CIRCUMFERENCE) * MOTOR_CONTROL_DT

        self.get_logger().info("--- 机器人参数配置 (无补偿版本) ---")
        self.get_logger().info(f"TF广播: {'启用' if self.publish_tf else '禁用'}")
        self.get_logger().info(f"odom_topic: {self.odom_topic}")
        self.get_logger().info(f"左轮串口: {SERIAL_PORT_LEFT}")
        self.get_logger().info(f"右轮串口: {SERIAL_PORT_RIGHT}")
        self.get_logger().info(f"波特率: {BAUDRATE}")
        self.get_logger().info(f"轮距 (L): {WHEEL_BASE:.3f} 米")
        self.get_logger().info(f"轮半径 (r): {WHEEL_RADIUS:.3f} 米")
        self.get_logger().info(f"编码器 CPR: {ENCODER_CPR}")
        self.get_logger().info(f"减速比: {GEAR_RATIO:.1f}")
        self.get_logger().info(f"轮子周长: {WHEEL_CIRCUMFERENCE:.3f} 米")
        self.get_logger().info(f"最小启动脉冲: {MIN_START_PULSES}")
        self.get_logger().info(f"左轮方向系数: {LEFT_WHEEL_DIR}")
        self.get_logger().info(f"右轮方向系数: {RIGHT_WHEEL_DIR}")
        self.get_logger().info(f"转向校准系数: {self.turning_correction_factor}")
        self.get_logger().info(f"角速度方向系数: {self.angular_velocity_direction}")
        self.get_logger().info("------------------------------------")

def main(args=None):
    rclpy.init(args=args)
    commander_node = CmdVelSerialCommander()
    try:
        rclpy.spin(commander_node)
    except KeyboardInterrupt:
        commander_node.get_logger().info("节点被用户中断。")
    finally:
        commander_node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
