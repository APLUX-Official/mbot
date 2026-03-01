# evobase_bringup 项目说明

## 项目简介

本项目为ROS2差速/履带机器人底盘的通用bringup包，支持多种电机驱动器、编码器、任意装配方式，具备高可配置性、易调试、强健的异常处理能力。

**核心特性：**
- 支持任意轮子/编码器方向，参数全配置化
- 运动学/里程计推算，支持转向校准系数、动态漂移补偿
- 串口异常自动退出，便于launch管理
- 丰富的调试脚本，便捷验证编码器数据、死区、漂移等
- udev规则固定串口名，硬件接线无忧

---

## 目录结构

```
evobase_bringup/
├── evobase_bringup/
│   ├── __init__.py
│   ├── evobase_cmd_odom.py              # 主节点(差速，支持转向校准)
│   ├── tracked_cmd_odom.py           # 履带专用PID节点(支持转向校准)
│   ├── cmd_odom_compensation.py      # (可选)带静态/动态补偿版本
│   └── cmd_odom_no_compensation.py   # (可选)无补偿版本
├── config/
│   ├── evobase_config.yaml              # 差速/履带通用配置
│   └── tracked_config.yaml           # 履带专用配置
├── scripts/
│   ├── find_serial.py                # 自动查找可用串口
│   ├── read_encoders_once.py         # 实时读取/验证编码器数据
├── udev/
│   ├── 99-differential-drive.rules   # udev规则示例
│   ├── install_udev_rules.sh         # 一键安装udev规则
│   ├── uninstall_udev_rules.sh       # 卸载udev规则
│   ├── test_device_mapping.sh        # 测试串口映射
│   └── README.md                     # 说明文档
├── launch/
│   └── cmd_odom.launch.py            # 启动文件
├── package.xml
├── setup.py
├── README.md
└── ...
```

---

## 1. 配置文件说明

### 1.1 evobase_config.yaml (差速/履带通用)
```yaml
robot_params:
  wheel_base: 0.155
  wheel_radius: 0.0335
  encoder_cpr: 52
  gear_ratio: 30.0
  min_start_pulses: 3
  left_wheel_dir: -1
  right_wheel_dir: 1
  turning_correction_factor: 1.363  # 转向校准系数(实测里程计角度/实际角度)
  #drift_compensation_map:           # (可选)动态漂移补偿表
  #  - [0.1, 0.0125]
  #  - [0.5, 0.0150]
serial_ports:
  left: /dev/drive_left
  right: /dev/drive_right
```

### 1.2 tracked_config.yaml (履带专用)
同上，增加PID参数、motor串口、base_frame等。

---

## 2. 主要节点与脚本

### 2.1 evobase_cmd_odom.py
- 差速节点，支持方向系数、转向校准、死区脉冲、异常自动退出。
- 运动学推算支持turning_correction_factor，配置灵活。
- 适配所有常见装配/线序/编码器方向。

### 2.2 tracked_cmd_odom.py
- 履带专用PID节点，支持有效轮距校准(转向校准系数)、全参数化。
- 适合高精度履带底盘。

### 2.3 scripts目录
- test_cmd_vel.py：循环/单次发送cmd_vel，测试运动响应。
- find_serial.py：自动检测可用串口，辅助配置udev。
- read_encoders_once.py：实时读取左右编码器数据，验证线序/方向/死区。
- cmd_odom_compensation.py、cmd_odom_no_compensation.py：补偿算法测试。

---

## 3. udev规则与串口映射

### 3.1 作用
避免USB串口号变化导致左右轮混淆，强烈建议使用udev规则固定设备名。

### 3.2 快速配置
```bash
cd udev
sudo bash install_udev_rules.sh
# 查看/dev/drive_left /dev/drive_right 是否存在
```
详细说明见udev/README.md。

---

## 4. 关键参数与标定

| 参数名 | 作用 | 标定方法 |
|--------|------|----------|
| wheel_base | 轮距 | 物理测量/原地旋转一圈，调到里程计角度=实际角度 |
| wheel_radius | 轮半径 | 物理测量/直线跑1m，调到里程计距离=实际距离 |
| left_wheel_dir/right_wheel_dir | 方向系数 | 让前进时编码器为正，反之-1 |
| min_start_pulses | 死区脉冲 | 让电机刚好能启动的最小脉冲 |
| turning_correction_factor | 转向校准 | 让里程计角度=实际角度，填(里程计角度/实际角度) |
| drift_compensation_map | 漂移补偿 | 直线跑偏时微调，见注释 |

---

## 5. 典型调试流程

1. 配置udev，确保左右串口名固定。
2. 用scripts/read_encoders_once.py实时查看编码器数据，确认方向系数设置正确。
3. 用test_cmd_vel.py测试前进/后退/原地转，观察里程计与实际是否一致。
4. 若原地转角度偏大/小，调整turning_correction_factor。
5. 若直线跑偏，微调drift_compensation_map。
6. 若电机不转，调大min_start_pulses。

---

## 6. 常见问题与排查

- 串口打不开：检查udev、权限、线序，或用find_serial.py辅助排查。
- 方向反了：调整left_wheel_dir/right_wheel_dir。
- 角度累计偏大/小：调整turning_correction_factor。
- 直线跑偏：调整drift_compensation_map。
- 电机不转：调大min_start_pulses。
- 节点异常不退出：已支持自动shutdown，launch可自动重启。

---

## 7. 运动学与补偿说明

- 差速/履带运动学推算，详见evobase_cmd_odom.py、tracked_cmd_odom.py。
- 支持转向校准系数(有效轮距/角度补偿)，适配履带打滑。
- 支持动态漂移补偿表，适配不同速度下的跑偏。

---

## 8. 贡献与扩展

- 欢迎提交PR、Issue，支持多种硬件/协议扩展。
- 代码结构清晰，便于二次开发。

---

## 9. 参考资料

- [ROS2官方文档](https://docs.ros.org/en/humble/)
- [差速驱动运动学](https://en.wikipedia.org/wiki/Differential_wheeled_robot)
- [TF2教程](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html)

---

## 10. 常用命令速查

```bash
# 编译
colcon build --packages-select evobase_bringup
# 运行
ros2 run evobase_bringup evobase_cmd_odom
ros2 launch evobase_bringup cmd_odom.launch.py
# 测试
ros2 run evobase_bringup test_cmd_vel
# 监控
ros2 topic echo /odom
ros2 topic echo /cmd_vel
# 动态调参
ros2 param set /evobase_cmd_odom publish_tf false
```
