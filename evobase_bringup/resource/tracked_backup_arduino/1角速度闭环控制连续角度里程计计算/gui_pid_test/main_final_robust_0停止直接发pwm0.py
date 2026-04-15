# ==================== 最终稳健版完整代码 (v8.0 - Arduino计算) ====================
import serial
import time
import json
import math
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from collections import deque
import PySimpleGUI as sg

def load_config(path='config.json'):
    try:
        with open(path, 'r') as f: cfg = json.load(f)
        pid_cfg = cfg['pid_params']
        speed_cfg = cfg['target_speeds']
        serial_cfg = cfg['serial_ports']['motor_controller']
        robot_cfg = cfg['robot_params']
        if 'wheel_radius' not in robot_cfg or 'wheel_base' not in robot_cfg:
            raise KeyError("'robot_params' 中缺少 'wheel_radius' 或 'wheel_base'")
        return pid_cfg, speed_cfg, serial_cfg, robot_cfg
    except Exception as e:
        sg.popup_error(f"加载或解析配置文件 '{path}' 失败:\n{e}", title="配置错误"); exit()

class AdvancedPID:
    def __init__(self, kp=0, ki=0, kd=0, kf=0, out_min=-255, out_max=255, integral_limit=200, deadband=0):
        self.kp, self.ki, self.kd, self.kf = kp, ki, kd, kf
        self.out_min, self.out_max = out_min, out_max
        self.integral_limit, self.deadband = integral_limit, deadband
        self.setpoint, self.last_error, self.integral, self.last_output = 0, 0, 0, 0
    def compute(self, feedback, dt):
        if dt <= 0: return self.last_output
        error = self.setpoint - feedback
        self.integral += error * dt
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        derivative = (error - self.last_error) / dt
        self.last_error = error
        pid_out = self.kp*error + self.ki*self.integral + self.kd*derivative
        ff_out = self.kf * self.setpoint
        output = ff_out + pid_out
        if abs(output) < self.deadband and self.setpoint == 0: output = 0
        self.last_output = int(max(self.out_min, min(self.out_max, output)))
        return self.last_output
    def reset(self): self.last_error, self.integral = 0, 0

class DifferentialDriveOdometry:
    def __init__(self, wheel_base, wheel_radius):
        self.wheel_base = wheel_base
        self.wheel_radius = wheel_radius
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.last_left_angle, self.last_right_angle = 0.0, 0.0
        self.initialized = False
    def update(self, left_angle, right_angle):
        if not self.initialized:
            self.last_left_angle, self.last_right_angle = left_angle, right_angle
            self.initialized = True
            return
        
        # 通过角度变化量计算距离变化量，这种方法比直接用总距离的差值更精确
        delta_left_dist = (left_angle - self.last_left_angle) * self.wheel_radius
        delta_right_dist = (right_angle - self.last_right_angle) * self.wheel_radius
        self.last_left_angle, self.last_right_angle = left_angle, right_angle
        
        delta_center = (delta_left_dist + delta_right_dist) / 2.0
        delta_theta = (delta_right_dist - delta_left_dist) / self.wheel_base
        
        self.x += delta_center * math.cos(self.theta + delta_theta / 2.0)
        self.y += delta_center * math.sin(self.theta + delta_theta / 2.0)
        self.theta = (self.theta + delta_theta + math.pi) % (2 * math.pi) - math.pi
    def get_pose_str(self):
        return f"X={self.x:.2f}m, Y={self.y:.2f}m, Theta={math.degrees(self.theta):.2f}°"

def read_speed_and_angle(ser):
    """读取 "速度,角度" 格式的串口数据并返回一个元组 (speed, angle)"""
    last_line = ""
    if not ser.is_open: return None, None
    while ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8').strip()
            if line: last_line = line
        except:
            ser.read(ser.in_waiting)
            return None, None
    if last_line:
        try:
            parts = last_line.split(',')
            if len(parts) == 2:
                return float(parts[0]), float(parts[1])
        except:
            pass
    return None, None

def update_pid_from_gui(pid, values, prefix):
    try:
        pid.kp=float(values[f'{prefix}_kp']); pid.ki=float(values[f'{prefix}_ki'])
        pid.kd=float(values[f'{prefix}_kd']); pid.kf=float(values[f'{prefix}_kf'])
        new_setpoint = float(values[f'{prefix}_sp'])
        if pid.setpoint != new_setpoint: pid.setpoint = new_setpoint; pid.reset()
    except:
        print(f"警告：GUI中输入的 {prefix} 参数无效。")

def plot_waveform(ax1, ax2, histories, pids, cfg):
    ax1.clear()
    title = (f'L: Kp={pids["L"].kp:.1f} Ki={pids["L"].ki:.1f} SP={pids["L"].setpoint:.2f} | R: Kp={pids["R"].kp:.1f} Ki={pids["R"].ki:.1f} SP={pids["R"].setpoint:.2f}')
    ax1.set_title(title, fontsize=9)
    ax1.plot(histories['L_speed'], label=f'L Speed'); ax1.plot(histories['R_speed'], label=f'R Speed')
    ax1.axhline(y=pids['L'].setpoint, color='c', linestyle='--'); ax1.axhline(y=pids['R'].setpoint, color='m', linestyle='--')
    ax1.legend(loc='upper left', fontsize='small'); ax1.grid(True); ax1.set_ylabel('Speed (m/s)')
    ax2.clear()
    ax2.plot(histories['L_pwm'], label=f'L PWM'); ax2.plot(histories['R_pwm'], label=f'R PWM')
    ax2.legend(loc='upper left', fontsize='small'); ax2.grid(True); ax2.set_ylabel('PWM'); ax2.set_xlabel('Samples')
    ax2.set_ylim(cfg['out_min'] * 1.1, cfg['out_max'] * 1.1)
    plt.tight_layout(); plt.pause(0.001)

# ==================== 主程序入口 ====================
if __name__ == "__main__":
    pid_cfg, speed_cfg, serial_cfg, robot_cfg = load_config()

    try:
        left_ser = serial.Serial(serial_cfg['left_port'], 115200, timeout=0.01)
        right_ser = serial.Serial(serial_cfg['right_port'], 115200, timeout=0.01)
        arduino_ser = serial.Serial(serial_cfg['arduino_port'], 115200, timeout=0.01)
        print("INFO: 所有串口连接成功，等待2秒让Arduino启动...")
        time.sleep(2)
    except serial.SerialException as e:
        sg.popup_error(f"无法打开串口: {e}", title="串口错误"); exit()

    pids = {'L': AdvancedPID(**pid_cfg), 'R': AdvancedPID(**pid_cfg)}
    pids['L'].setpoint = speed_cfg.get('left_wheel', 0)
    pids['R'].setpoint = speed_cfg.get('right_wheel', 0)
    
    odometry = DifferentialDriveOdometry(robot_cfg['wheel_base'], robot_cfg['wheel_radius'])
    
    # --- GUI 和 绘图初始化 ---
    sg.theme('DarkBlue')
    def pid_layout(p, side):
        return [sg.Text(f'{side}轮PID'),
                sg.Text('Kp'), sg.Input(str(pid_cfg.get('kp',0)), k=f'{p}_kp', s=(6,1)),
                sg.Text('Ki'), sg.Input(str(pid_cfg.get('ki',0)), k=f'{p}_ki', s=(6,1)),
                sg.Text('Kd'), sg.Input(str(pid_cfg.get('kd',0)), k=f'{p}_kd', s=(6,1)),
                sg.Text('Kf'), sg.Input(str(pid_cfg.get('kf',0)), k=f'{p}_kf', s=(6,1)),
                sg.Text('目标'), sg.Input(str(speed_cfg.get(f'{side.lower()}_wheel',0)), k=f'{p}_sp', s=(6,1))]
    layout = [pid_layout('L', 'Left'), pid_layout('R', 'Right'),
              [sg.Button('应用参数'), sg.Button('暂停/继续', key='-PAUSE-'), sg.Button('保存波形'), sg.Button('退出')],
              [sg.Text("里程计: N/A", key='-ODOM_TEXT-', size=(60,1))]]
    window = sg.Window('PID & Odometry 实时监控 (v8.0-Robust)', layout, finalize=True)
    paused = False
    plt.ion(); fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7))
    history_len = 300
    histories = {
        'L_speed': deque(maxlen=history_len), 'R_speed': deque(maxlen=history_len),
        'L_pwm': deque(maxlen=history_len),   'R_pwm': deque(maxlen=history_len)
    }
    
    # --- 主循环 ---
    CONTROL_INTERVAL, PLOT_INTERVAL = 0.01, 0.1
    last_control_time, last_plot_time = time.time(), time.time()
    
    # 保存上一次的有效数据
    last_data = {'L_speed': 0.0, 'R_speed': 0.0, 'L_angle': 0.0, 'R_angle': 0.0}

    try:
        while True:
            current_time = time.time()
            event, values = window.read(timeout=1)
            if event in (sg.WIN_CLOSED, '退出'): break
            if event == '应用参数': update_pid_from_gui(pids['L'], values, 'L'); update_pid_from_gui(pids['R'], values, 'R')
            if event == '-PAUSE-': paused = not paused; window['-PAUSE-'].update('继续' if paused else '暂停')
            if event == '保存波形': fig.savefig(f'pid_wave_{time.strftime("%Y%m%d_%H%M%S")}.png')

            if paused:
                arduino_ser.write(b"0,0\n"); time.sleep(0.1)
                last_control_time = current_time; continue
            
            if current_time - last_control_time < CONTROL_INTERVAL: continue
            dt = current_time - last_control_time; last_control_time = current_time

            is_stopped_mode = (pids['L'].setpoint == 0 and pids['R'].setpoint == 0)

            # --- 数据处理 ---
            l_omega, l_angle = read_speed_and_angle(left_ser)
            r_omega, r_angle = read_speed_and_angle(right_ser)

            # 如果收到新数据，就更新；否则使用上一次的有效数据
            if l_omega is not None: last_data['L_speed'] = l_omega * robot_cfg['wheel_radius']
            if l_angle is not None: last_data['L_angle'] = l_angle
            
            if r_omega is not None: last_data['R_speed'] = -r_omega * robot_cfg['wheel_radius'] # 右轮取反
            if r_angle is not None: last_data['R_angle'] = r_angle
            
            left_speed, right_speed = last_data['L_speed'], last_data['R_speed']
            
            # --- 控制逻辑 ---
            if is_stopped_mode:
                left_pwm, right_pwm = 0, 0
                pids['L'].reset(); pids['R'].reset()
            else:
                left_pwm = pids['L'].compute(left_speed, dt)
                right_pwm = pids['R'].compute(right_speed, dt)

            odometry.update(last_data['L_angle'], -last_data['R_angle']) # 里程计右轮角度取反
            arduino_ser.write(f"{left_pwm},{right_pwm}\n".encode())

            # --- 终端打印 ---
            print(f"\r[Mode]: {'STOP' if is_stopped_mode else 'RUN'} | "
                  f"Speed L:{left_speed: 6.2f} R:{right_speed: 6.2f} | "
                  f"PWM L:{left_pwm: 4d} R:{right_pwm: 4d} | "
                  f"Odom: {odometry.get_pose_str()}")

            # --- 更新历史和绘图 ---
            histories['L_speed'].append(left_speed); histories['R_speed'].append(right_speed)
            histories['L_pwm'].append(left_pwm); histories['R_pwm'].append(right_pwm)

            if current_time - last_plot_time > PLOT_INTERVAL:
                if histories['L_speed']: plot_waveform(ax1, ax2, histories, pids, pid_cfg)
                window['-ODOM_TEXT-'].update(f"里程计: {odometry.get_pose_str()}")
                last_plot_time = current_time

    finally:
        print("\n")
        print("正在停止电机并关闭资源...")
        if 'arduino_ser' in locals() and arduino_ser.is_open: arduino_ser.write(b"0,0\n"); arduino_ser.close()
        if 'left_ser' in locals() and left_ser.is_open: left_ser.close()
        if 'right_ser' in locals() and right_ser.is_open: right_ser.close()
        if 'window' in locals(): window.close()
        plt.ioff(); plt.close()
        print("所有资源已安全关闭。")