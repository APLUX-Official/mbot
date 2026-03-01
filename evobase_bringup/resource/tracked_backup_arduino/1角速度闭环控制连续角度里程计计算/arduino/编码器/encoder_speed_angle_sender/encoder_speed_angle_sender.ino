/*
 * AS5600 速度计与里程计数据源 (最终稳健版 v8.0)
 * - 功能: 在Arduino端完成对时间精度要求最高的“速度计算”和“滤波”，向上层提供干净的数据。
 * - 频率: 100Hz
 * - 输出格式: "滤波后的角速度,连续角度" (e.g., "10.12,35.48")
 */

#include <Wire.h>

// ========== 低通滤波器类 ==========
class LowPassFilter {
public:
    LowPassFilter(float time_constant) : y(0.0f), timestamp_us(0) {
        Tf = time_constant;
    }

    float operator()(float x) {
        unsigned long now = micros();
        if (timestamp_us == 0) { // 第一次调用时初始化
            y = x;
            timestamp_us = now;
            return y;
        }
        
        float dt = (float)(now - timestamp_us) * 1e-6f;
        timestamp_us = now;
        
        if (dt <= 0) return y; // 防止时间戳回绕等问题

        float alpha = dt / (Tf + dt);
        y = alpha * x + (1.0f - alpha) * y;
        return y;
    }
private:
    float y, Tf;
    unsigned long timestamp_us;
};

// ========== 全局变量 ==========
const float _2PI = 6.28318530718f;
const unsigned int SEND_INTERVAL_MS = 10; // 10ms -> 100Hz

float angle_prev_read = 0.0f;       // 用于getContinuousAngle内部
float angle_prev_vel = 0.0f;        // 用于getRawVelocity内部
unsigned long time_prev_vel_us = 0; // 用于getRawVelocity内部 (使用微秒)
unsigned long last_send_time_ms = 0;  // 用于loop中的定时发送 (使用毫秒)
int32_t full_rotations = 0;
bool first_read = true;

// 滤波器实例，Tf=0.02s提供了较好的平滑效果
LowPassFilter velocity_filter(0.02f); 

// ========== 函数声明 ==========
float getRawAngle();
float getContinuousAngle();
float getRawVelocity();

// ========== Arduino 主程序 ==========
void setup() {
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000); // 提高I2C速率
    delay(500); // 等待传感器稳定
    
    // 初始化函数，填充初始值
    getContinuousAngle();
    getRawVelocity();
    
    Serial.println("Encoder ready. Sending: filtered_velocity,continuous_angle");
}

void loop() {
    // 持续计算，以保持所有内部状态（角度、速度、时间戳）的更新
    float raw_velocity = getRawVelocity();
    float filtered_velocity = velocity_filter(raw_velocity);
    
    // 使用millis()定时发送数据
    unsigned long current_millis = millis();
    if (current_millis - last_send_time_ms >= SEND_INTERVAL_MS) {
        last_send_time_ms = current_millis;
        
        // 发送两个值：1. 滤波后的角速度 (用于PID), 2. 最新的连续角度 (用于Odom)
        Serial.print(filtered_velocity, 4);
        Serial.print(",");
        // angle_prev_vel 变量在 getRawVelocity() 中被更新，保存着最新的连续角度值
        Serial.println(angle_prev_vel, 4); 
    }
}

// ========== 核心函数实现 ==========
float getRawAngle() {
    Wire.beginTransmission(0x36); Wire.write(0x0C);
    if (Wire.endTransmission() != 0) return -1.0f;
    
    Wire.requestFrom(0x36, 2);
    if (Wire.available() < 2) return -1.0f;
    
    uint16_t rawValue = (Wire.read() << 8) | Wire.read();
    return (float)rawValue / 4096.0f * _2PI;
}

float getContinuousAngle() {
    float current_angle = getRawAngle();
    if (current_angle < 0) return angle_prev_read + (float)full_rotations * _2PI;
    
    if (first_read) {
        angle_prev_read = current_angle;
        first_read = false;
    }
    
    float d_angle = current_angle - angle_prev_read;
    if (d_angle < -_2PI * 0.8f) full_rotations++;
    else if (d_angle > _2PI * 0.8f) full_rotations--;
    
    angle_prev_read = current_angle;
    return (float)full_rotations * _2PI + current_angle;
}

float getRawVelocity() {
    unsigned long now_us = micros();
    float current_angle = getContinuousAngle();
    
    if (time_prev_vel_us == 0) {
        time_prev_vel_us = now_us;
        angle_prev_vel = current_angle;
        return 0.0f;
    }
    
    float dt_s = (float)(now_us - time_prev_vel_us) * 1e-6f;
    // 防止dt异常
    if (dt_s <= 0 || dt_s > 0.1) {
        time_prev_vel_us = now_us;
        angle_prev_vel = current_angle;
        return 0.0f;
    }
    
    float velocity = (current_angle - angle_prev_vel) / dt_s;
    
    time_prev_vel_us = now_us;
    angle_prev_vel = current_angle;
    
    return velocity;
}