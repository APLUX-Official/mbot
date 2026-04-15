# udev Rules for Two Differential Drive Robot

这个目录包含了用于差分驱动机器人的udev规则配置，可以将CP2102 USB转串口设备映射到固定的设备名称。

## 功能

- 将序列号为 "left" 的CP2102设备映射到 `/dev/drive_left`
- 将序列号为 "right" 的CP2102设备映射到 `/dev/drive_right`  
- 将序列号为 "lidar" 的CP2102设备映射到 `/dev/drive_lidar`

## 文件说明

- `99-differential-drive.rules` - udev规则文件
- `install_udev_rules.sh` - 安装脚本
- `uninstall_udev_rules.sh` - 卸载脚本
- `test_device_mapping.sh` - 测试脚本
- `README.md` - 说明文档

## 安装步骤

1. 给脚本添加执行权限：
   ```bash
   chmod +x *.sh
   ```

2. 运行安装脚本：
   ```bash
   ./install_udev_rules.sh
   ```

3. 如果提示需要重新登录，请注销后重新登录。

4. 重新连接USB设备或运行：
   ```bash
   sudo udevadm trigger
   ```

## 验证安装

1. 使用测试脚本：
   ```bash
   ./test_device_mapping.sh
   ```

2. 手动检查设备：
   ```bash
   ls -la /dev/drive_*
   ```

3. 查看设备详细信息：
   ```bash
   udevadm info --name=/dev/drive_left
   ```

## 使用说明

安装成功后，你的ROS2配置文件中可以使用固定的设备路径：

```yaml
serial_ports:
  left: /dev/drive_left
  right: /dev/drive_right
```

这样无论USB设备插入的顺序如何，设备名称都会保持一致。

## 卸载

如果需要移除udev规则：

```bash
./uninstall_udev_rules.sh
```

## 故障排除

### 设备没有映射成功

1. 检查设备是否连接：
   ```bash
   python3 ../resource/backup数据备份/find_serial.py
   ```

2. 检查设备序列号是否正确：
   ```bash
   udevadm info --name=/dev/ttyUSB0 | grep ID_SERIAL_SHORT
   ```

3. 检查udev规则是否正确安装：
   ```bash
   ls -la /etc/udev/rules.d/99-differential-drive.rules
   ```

4. 重新加载udev规则：
   ```bash
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```

### 权限问题

确保用户在dialout组中：
```bash
groups $USER | grep dialout
```

如果不在，添加用户到dialout组：
```bash
sudo usermod -a -G dialout $USER
```

然后注销重新登录。

## 多台设备部署

1. 将整个 `udev` 目录复制到目标设备
2. 在目标设备上运行 `./install_udev_rules.sh`
3. 确保CP2102设备的序列号正确设置为 "left", "right", "lidar"

## 注意事项

- 确保你的CP2102设备序列号设置正确
- 如果序列号不匹配，需要修改 `99-differential-drive.rules` 文件中的序列号
- 设备权限设置为666，属于dialout组，普通用户可以读写
