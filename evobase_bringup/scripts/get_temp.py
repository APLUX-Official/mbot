import os
import sys
import glob
import time

def read_aidlux_temperature():
    """
    尝试读取 Aidlux (基于安卓内核的高通平台) 的 CPU 温度。
    """
    # 优先测试最可能找到 CPU 核心温度的路径（高通芯片的 die temp）
    high_priority_paths = [
        # 高通 PMK8350 芯片核心温度 (最可能是 CPU 温度)
        "/sys/devices/platform/soc/c440000.qcom,spmi/spmi-0/spmi0-00/c440000.qcom,spmi:qcom,pmk8350@0:vadc@3100/iio:device0/in_temp_pmk8350_die_temp_input",
        # 另一个高通电源管理芯片的温度
        "/sys/devices/platform/soc/c440000.qcom,spmi/spmi-0/spmi0-00/c440000.qcom,spmi:qcom,pmk8350@0:vadc@3100/iio:device0/in_temp_pm7325_die_temp_input",
    ]

    # 标准 thermal zone 路径集合 (Aidlux 有很多个，需要遍历)
    thermal_zone_paths = (
        glob.glob("/sys/class/thermal/thermal_zone*/temp") +
        glob.glob("/sys/devices/virtual/thermal/thermal_zone*/temp")
    )
    
    # 将标准路径添加到优先路径之后
    temp_paths = high_priority_paths + thermal_zone_paths

    print("--- 正在尝试读取 Aidlux CPU 温度 (高通内核模式) ---")
    
    # 存储找到的所有有效温度，方便比较
    valid_temperatures = []
    
    for i, path in enumerate(temp_paths):
        if os.path.exists(path):
            try:
                with open(path, 'r') as f:
                    temp_str = f.read().strip()
                    temp_raw = int(temp_str)
                    
                    # 假定单位是毫摄氏度 (mC)，这是最常见的格式
                    temp_celsius = temp_raw / 1000.0
                    
                    # 检查数值是否合理
                    if temp_celsius > 1000:
                        # 如果是像 35000 这样的值，除以 1000 得到 35.0 C，这是合理的
                        pass
                    elif temp_celsius < 5.0 or temp_celsius > 100.0:
                        # 如果转换后的值不在合理范围，可能这个文件是其他数据 (比如计数器或错误值)
                        # print(f"路径 {path} 读取到的温度 {temp_celsius:.2f} °C 不合理，忽略...")
                        continue

                    # 如果是高优先级的路径，直接返回结果
                    if i < len(high_priority_paths):
                         print(f"✅ 核心温度路径找到: {path}")
                         print(f"当前 CPU 温度: {temp_celsius:.2f} °C")
                         return temp_celsius

                    # 对于 thermal zone，记录下来，稍后比较
                    valid_temperatures.append((temp_celsius, path))
            
            except Exception:
                # 忽略读取失败或格式错误的路径
                pass

    if valid_temperatures:
        # 如果找到了多个 thermal zone 温度，通常最高的就是 CPU 核心
        valid_temperatures.sort(key=lambda x: x[0], reverse=True)
        max_temp, max_path = valid_temperatures[0]
        
        print(f"⚠️ 核心路径未找到，使用最高 Thermal Zone 温度。")
        print(f"来源路径: {max_path}")
        print(f"当前 CPU 温度: {max_temp:.2f} °C")
        return max_temp
    
    # 如果所有路径都尝试失败
    print("---------------------------------------")
    print("❌ 错误：无法在 Aidlux 环境中找到任何有效的 CPU 温度传感器数据。")
    print("请确认 'aidlux' 用户的权限是否足够读取 /sys 目录下的这些文件。")
    sys.exit(1)

if __name__ == "__main__":
    read_aidlux_temperature()