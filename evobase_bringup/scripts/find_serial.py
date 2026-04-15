import serial.tools.list_ports

def find_cp2102_by_vid_pid(target_vid=0x10C4, target_pid=0xEA60):
    """
    查找具有指定 VID 和 PID 的 CP2102 设备。

    Args:
        target_vid (int): 目标设备的供应商 ID (VID)。默认为 CP2102 的默认 VID。
        target_pid (int): 目标设备的产品 ID (PID)。默认为 CP2102 的默认 PID。

    Returns:
        list: 找到的匹配设备的端口路径列表（例如 '/dev/ttyUSB0'）。
              如果没有找到，则返回空列表。
    """
    found_devices = []
    ports = serial.tools.list_ports.comports() # 获取所有可用的串口

    print("--- 发现的串口设备列表 ---")
    if not ports:
        print("未检测到任何串口设备。请确保设备已连接。")
    for port in ports:
        print(f"  设备路径: {port.device}")
        print(f"    描述: {port.description}")
        print(f"    硬件ID: {port.hwid}")
        print(f"    供应商ID (VID): {hex(port.vid) if port.vid else 'N/A'}")
        print(f"    产品ID (PID): {hex(port.pid) if port.pid else 'N/A'}")
        print(f"    序列号 (Serial): {port.serial_number if port.serial_number else 'N/A'}")
        print("-" * 20)

        # 检查 VID 和 PID 是否匹配
        if port.vid == target_vid and port.pid == target_pid:
            found_devices.append(port.device)

    print("--- 查找结果 ---")
    if found_devices:
        print(f"成功找到 {len(found_devices)} 个匹配 CP2102 默认 VID/PID 的设备:")
        for device_path in found_devices:
            print(f"  - {device_path}")
    else:
        print(f"未找到 VID={hex(target_vid)} 和 PID={hex(target_pid)} 的 CP2102 设备。")
        print("请确保 CP2102 设备已连接且其 VID/PID 未被修改。")

    return found_devices

# --- 主程序入口 ---
if __name__ == "__main__":
    # 使用 CP2102 的默认 VID 和 PID 进行查找
    # 你可以修改这些值来查找其他特定设备
    default_cp2102_vid = 0x10C4
    #default_cp2102_pid = 0xEA60

    # 调用函数查找设备
    matched_ports = find_cp2102_by_vid_pid(default_cp2102_vid)

    # 如果找到了设备，你可以选择打开第一个匹配的设备进行通信
    if matched_ports:
        first_device_path = matched_ports[0]
        try:
            # 示例：打开串口，波特率为 115200
            ser = serial.Serial(first_device_path, 115200, timeout=1)
            print(f"\n成功打开串口: {first_device_path}")
            # 这里可以添加你的串口通信逻辑，例如读取数据或发送数据
            # ser.write(b'Hello from Python\n')
            # line = ser.readline().decode('utf-8').strip()
            # print(f"接收到: {line}")
            ser.close()
            print("串口已关闭。")
        except Exception as e:
            print(f"\n错误：无法打开串口 {first_device_path} - {e}")
    else:
        print("\n未找到符合条件的串口设备，无法进行通信。")