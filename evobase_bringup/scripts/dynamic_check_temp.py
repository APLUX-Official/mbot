#!/usr/bin/env python3
import os
import time

def get_temperatures():
    """
    从 /sys/class/thermal/ 路径读取所有 thermal_zone 的温度和类型。
    返回一个包含 (zone, type_name, temp) 元组的列表。
    """
    # 过滤出所有 thermal_zoneX 目录
    thermal_zones = [d for d in os.listdir('/sys/class/thermal/') if d.startswith('thermal_zone')]
    temperatures = []
    
    for zone in thermal_zones:
        # 完整的路径
        temp_path = f'/sys/class/thermal/{zone}/temp'
        type_path = f'/sys/class/thermal/{zone}/type'
        
        try:
            # 读取温度 (通常是毫摄氏度) 并转换为摄氏度 (除以 1000.0)
            with open(temp_path, 'r') as f:
                temp = int(f.read().strip()) / 1000.0
            
            # 读取类型名称
            with open(type_path, 'r') as f:
                type_name = f.read().strip()
                
            temperatures.append((zone, type_name, temp))
            
        except Exception:
            # 忽略任何读取文件时发生的错误
            pass
            
    return temperatures

def display_average_temperatures(temps):
    """
    计算并打印分类的平均温度。
    """
    
    categories = {
        'CPU': [],
        'GPU': [],
        '内存 (DDR)': [],
        '其他': []
    }
    
    for _, type_name, temp in temps:
        # 仅处理温度大于 0 的有效数据
        if temp > 0:
            # 温度分类
            if 'cpu' in type_name.lower():
                categories['CPU'].append(temp)
            elif 'gpu' in type_name.lower():
                categories['GPU'].append(temp)
            elif 'ddr' in type_name.lower():
                categories['内存 (DDR)'].append(temp)
            else:
                categories['其他'].append(temp)

    
    # --- 打印结果 ---
    
    print("=" * 40)
    print(f"[{time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())}] 设备分类平均温度:")
    print("-" * 40)

    for cat, temps_list in categories.items():
        if temps_list:
            avg = sum(temps_list) / len(temps_list)
            # 打印平均温度，保留一位小数，并显示参与计算的传感器数量
            print(f"  - {cat}: {avg:.1f}°C ({len(temps_list)}个传感器)")
        else:
            print(f"  - {cat}: 无有效数据")
            
    print("=" * 40)


if __name__ == "__main__":
    
    # 动态更新间隔 (秒)
    UPDATE_INTERVAL = 5
    
    try:
        while True:
            temps = get_temperatures()
            display_average_temperatures(temps)
            
            # 等待设定的时间间隔
            time.sleep(UPDATE_INTERVAL)
            
    except KeyboardInterrupt:
        # 允许用户通过 Ctrl+C 退出程序
        print("\n程序已终止。")
