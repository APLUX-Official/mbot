#!/usr/bin/env python3
import os

def get_temperatures():
    thermal_zones = [d for d in os.listdir('/sys/class/thermal/') if d.startswith('thermal_zone')]
    temperatures = []
    
    for zone in thermal_zones:
        try:
            with open(f'/sys/class/thermal/{zone}/temp', 'r') as f:
                temp = int(f.read().strip()) / 1000.0
            with open(f'/sys/class/thermal/{zone}/type', 'r') as f:
                type_name = f.read().strip()
            temperatures.append((zone, type_name, temp))
        except:
            pass
    
    return temperatures

if __name__ == "__main__":
    temps = get_temperatures()
    print("Android设备温度传感器:")
    
    categories = {
        'CPU': [],
        'GPU': [],
        '内存': [],
        '其他': []
    }
    
    for zone, type_name, temp in temps:
        if temp > 0:
            if 'cpu' in type_name.lower():
                categories['CPU'].append(temp)
            elif 'gpu' in type_name.lower():
                categories['GPU'].append(temp)
            elif 'ddr' in type_name.lower():
                categories['内存'].append(temp)
            else:
                categories['其他'].append(temp)
            print(f"{zone}: {type_name} - {temp:.1f}°C")
        else:
            print(f"{zone}: {type_name} - {temp:.1f}°C (无效)")
    
    print("\n分类平均温度:")
    for cat, temps_list in categories.items():
        if temps_list:
            avg = sum(temps_list) / len(temps_list)
            print(f"{cat}: {avg:.1f}°C ({len(temps_list)}个传感器)")
        else:
            print(f"{cat}: 无有效数据")