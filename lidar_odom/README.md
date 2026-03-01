# PLICP 激光雷达里程计 (轮式融合版) - ROS2

## 项目概述

这是一个经过深度优化的 2D 激光雷达里程计系统，基于 **PLICP (Point-to-Line ICP)** 算法，并集成了 **轮式里程计辅助 (Wheel Odometry Aided)** 功能。

针对**履带式机器人**、**剧烈旋转**以及**低纹理环境**（如长走廊、玻璃墙）进行了专门优化。系统能够利用底层轮式里程计的速度作为先验估计（First Guess），有效解决了纯激光里程计在丢帧或打滑时容易“飞车”的问题。

## 主要特性

### 核心功能更新

- **多传感器融合**：订阅 `/odom_wheel` 话题，融合轮速与激光数据。
- **抗丢帧机制 (Dead Reckoning)**：当激光雷达丢帧或匹配失败时，自动平滑切换至轮式里程计推算模式，保证 TF 树不断裂。
- **改进的运动预测**：摒弃了传统的匀速模型，改用实时轮速进行位姿预测，极大提高了快速转向时的匹配成功率。
- **自动重启策略**：在匹配误差过大时支持算法自动重启搜索。

### ROS2 架构特性

- **现代 C++**：基于 C++14/17 标准。
- **组件化设计**：支持 `rclcpp_components`，可作为组件加载到容器中，零拷贝传输。
- **生命周期管理**：完善的异常处理和 QoS 配置。

## 依赖项

### 系统依赖

```bash
# ROS2 基础包 (Foxy/Galileo/Humble)
sudo apt install ros-$ROS_DISTRO-desktop

# CSM (Canonical Scan Matcher) 算法库
sudo apt install libcsm-dev

# 构建工具
sudo apt install python3-colcon-common-extensions
```

### ROS2 消息依赖

- `nav_msgs` (新增: 用于接收轮式里程计)
- `sensor_msgs`
- `tf2_ros`

## 编译与安装

Bash

```
# 1. 进入工作空间
cd ~/evobase_ws/src

# 2. 克隆/放置代码
# (确保代码在 plicp_odometry_ros2 文件夹下)

# 3. 安装依赖
cd ~/evobase_ws
rosdep install --from-paths src --ignore-src -r -y

# 4. 编译
colcon build --packages-select plicp_odometry_ros2

# 5. 配置环境
source install/setup.bash
```

## 运行方法

### 1. 确保底层驱动运行

在启动本节点前，请确保你的机器人底盘驱动已启动，并正在发布轮式里程计话题：

Bash

```
# 检查话题是否存在，且频率稳定 (建议 >10Hz)
ros2 topic hz /odom_wheel
```

### 2. 启动节点

Bash

```
# 推荐使用 Launch 文件启动
ros2 launch plicp_odometry_ros2 plicp_odometry.launch.py

# 或者直接运行节点
ros2 run plicp_odometry_ros2 plicp_odometry_node
```

## 话题接口 (重要)

### 订阅话题 (Input)

| **话题名**    | **消息类型**                | **必需性**   | **描述**                                                     |
| ------------- | --------------------------- | ------------ | ------------------------------------------------------------ |
| `/scan`       | `sensor_msgs/msg/LaserScan` | **必须**     | 2D 激光雷达扫描数据。                                        |
| `/odom_wheel` | `nav_msgs/msg/Odometry`     | **强烈推荐** | 底盘发布的轮式里程计。节点仅提取其中的 `Twist` (速度) 信息用于预测。如果不提供，算法将回退到纯激光模式。 |
| `/tf`         | `tf2_msgs/msg/TFMessage`    | 必须         | 必须存在 `base_link` 到 `laser` 的静态变换 (Static TF)。     |

### 发布话题 (Output)

| **话题名** | **消息类型**             | **描述**                                                     |
| ---------- | ------------------------ | ------------------------------------------------------------ |
| `/odom`    | `nav_msgs/msg/Odometry`  | 融合后的高精度里程计。`pose` 为融合位姿，`twist` 为融合速度。 |
| `/tf`      | `tf2_msgs/msg/TFMessage` | 发布 `odom` -> `base_link` 的坐标变换。                      |

## 关键参数配置

针对履带车和融合模式，默认参数已进行调整。你可以在 `param` 文件中覆盖这些设置：

| **参数名**                     | **推荐值**  | **说明**                                                     |
| ------------------------------ | ----------- | ------------------------------------------------------------ |
| `odom_frame`                   | `odom`      | 里程计坐标系名称                                             |
| `base_frame`                   | `base_link` | 机器人基座标系名称                                           |
| `max_angular_correction_deg`   | **90.0**    | 允许两帧之间最大的旋转角度。融合模式下设大，防止急转弯丢失。 |
| `max_linear_correction`        | **1.0**     | 允许两帧之间最大的平移距离 (米)。                            |
| `restart`                      | **1**       | 开启重启功能。当匹配误差大时尝试重新搜索。                   |
| `restart_threshold_mean_error` | **0.05**    | 触发重启的误差阈值。                                         |
| `kf_dist_linear`               | 0.1         | 关键帧生成的距离阈值。                                       |

## 算法原理图解

该节点采用**松耦合**方式融合轮式里程计与激光里程计：

1. **预测阶段 (Prediction)**:
   - 读取 `/odom_wheel` 的实时线速度($v$)和角速度($\omega$)。
   - 计算 $dt$ 时间内的位姿增量 $\Delta P_{wheel}$。
   - 将 $\Delta P_{wheel}$ 作为 PLICP 算法的初值 (First Guess)。
2. **匹配阶段 (Correction)**:
   - PLICP 算法在初值附近搜索最优匹配，计算精确位姿 $\Delta P_{laser}$。
3. **防丢帧保护 (Protection)**:
   - **IF** 匹配成功：使用 $\Delta P_{laser}$ 更新位姿，并修正速度估计。
   - **ELSE** 匹配失败 (如雷达丢帧/环境退化)：直接使用 $\Delta P_{wheel}$ 进行航位推算 (Dead Reckoning)，确保里程计平滑连续。

## 常见问题 (FAQ)

**Q: 如果我不发布 /odom_wheel 会怎样？**

A: 节点会自动检测。如果没有 `/odom_wheel` 数据，它会回退到原始的“匀速模型预测”模式。但在履带车快速转向时可能会跟丢。

**Q: /odom_wheel 需要非常准吗？**

A: 不需要。它只需要提供一个“大概的趋势”（First Guess）。激光雷达会修正轮子的打滑累积误差。但轮子的**方向**必须正确（左转就是左转）。

**Q: 为什么输出的 odom 频率和雷达频率一致？**

A: 因为核心定位触发源仍然是激光雷达的回调。轮式里程计仅作为辅助输入。

------

**维护者**: WALT (BytePeace)

**License**: Apache 2.0