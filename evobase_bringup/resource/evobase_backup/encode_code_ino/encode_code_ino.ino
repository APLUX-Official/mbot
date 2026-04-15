#include <MsTimer2.h>  // 定时中断库

// --- 电机驱动引脚定义 ---
#define IN1 6
#define IN2 5

// --- 编码器引脚定义 ---
#define ENCODER_A 2 // 外部中断0 (通常是数字引脚2)
#define ENCODER_B 3 // 外部中断1 (通常是数字引脚3)

// --- 常量定义 ---
const int MAX_PWM = 255;
// VELOCITY_SCALE 是 PID 调优参数，用于缩放 PID 增益。
// 它的值通常通过实验调优来确定，以达到期望的控制响应和稳定性。
const float VELOCITY_SCALE = 26.0; 
const unsigned long TIMER_INTERVAL = 10;  // 定时器中断周期（ms），PID循环将严格每10ms执行
const int MIN_VELOCITY_PULSES = -100; // 最小目标脉冲数（对应 Arduino 接收的范围）
const int MAX_VELOCITY_PULSES = 100;  // 最大目标脉冲数（对应 Arduino 接收的范围）

// --- 电机控制相关变量 ---
// 使用 volatile long 确保中断和主循环之间的数据同步，避免数据竞争
volatile long encoder_count = 0; // 编码器累积脉冲数 (10ms 周期内)
//volatile long encoder_isr_total_count = 0; // 记录编码器中断触发的总次数 (用于调试)
volatile long total_accumulated_encoder_pulses = 0; // **自上电以来累计的总脉冲数**


struct Motor {
  float current_velocity_pulses; // 当前 10ms 内的实际脉冲数 (从编码器读取)
  float target_velocity_pulises = 0; // 目标 10ms 内的脉冲数 (从上位机接收)
  float motor_pwm_output; // PID 计算出的 PWM 值
  
  float velocity_KP = 91.9; // PID Kp 参数
  float velocity_KI = 6.5;  // PID Ki 参数

  // --- PID 内部状态变量 (现在是结构体成员) ---
  float bias_prev = 0; // 存储上次的偏差 (对应 Incremental_PI 中的 last_bias)
  float pwm_accum = 0; // 存储累积的 PWM 值 (对应 Incremental_PI 中的 pwm)
};

// 电机实例
Motor motor = {};

// --- 编码器中断服务例程 (ISR) ---
// 实现了四倍频计数：A相和B相的CHANGE都会触发中断
// 移除了所有 Serial.print()，以确保 ISR 快速执行。
void READ_ENCODER_A() { 
  //encoder_isr_total_count++; // 每次中断触发时递增

  if (digitalRead(ENCODER_A) == HIGH) {
    if (digitalRead(ENCODER_B) == LOW) {
      encoder_count--;
      total_accumulated_encoder_pulses--; // **更新累计总脉冲数**
    } else {
      encoder_count++;
      total_accumulated_encoder_pulses++; // **更新累计总脉冲数**
    }
  } else {
    if (digitalRead(ENCODER_B) == LOW) {
      encoder_count++;
      total_accumulated_encoder_pulses++; // **更新累计总脉冲数**
    } else {
      encoder_count--;
      total_accumulated_encoder_pulses--; // **更新累计总脉冲数**
    }
  }
}

void READ_ENCODER_B() { 
  //encoder_isr_total_count++; // 每次中断触发时递增

  if (digitalRead(ENCODER_B) == LOW) {                     //如果是下降沿触发的中断
    if (digitalRead(ENCODER_A) == LOW) {
      encoder_count--;
      total_accumulated_encoder_pulses--; // **更新累计总脉冲数**
    } else {
      encoder_count++;
      total_accumulated_encoder_pulses++; // **更新累计总脉冲数**
    }
  } else {                                                 //如果是上升沿触发的中断
    if (digitalRead(ENCODER_A) == LOW) {
      encoder_count++;
      total_accumulated_encoder_pulses++; // **更新累计总脉冲数**
    } else {
      encoder_count--;
      total_accumulated_encoder_pulses--; // **更新累计总脉冲数**
    }
  }
}


// --- 赋值给 PWM 寄存器的函数 ---
void Set_Pwm(Motor& motor_obj) {
  int pwm = motor_obj.motor_pwm_output;

  if (pwm < 0) {
    analogWrite(IN1, abs(pwm));
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    analogWrite(IN2, abs(pwm));
  }
}

// --- 增量 PI 控制器 ---
// current_encoder_pulses: 当前时间间隔内编码器读取到的脉冲数
// target_encoder_pulses: 目标时间间隔内应有的脉冲数
int Incremental_PI(float current_encoder_pulses, float target_encoder_pulses, Motor& motor_obj) {
  // PID 内部状态变量现在是 motor_obj 的成员
  float bias = target_encoder_pulses - current_encoder_pulses; // 目标值 - 实际值

  // 增量 PI 计算
  float delta_pwm = motor_obj.velocity_KP * (bias - motor_obj.bias_prev) / VELOCITY_SCALE + motor_obj.velocity_KI * bias / VELOCITY_SCALE;
  
  motor_obj.pwm_accum += delta_pwm; // 累加 PWM 值

  // 积分限幅 (Anti-windup): 防止 PWM 值无限累积超出有效范围
  motor_obj.pwm_accum = constrain(motor_obj.pwm_accum, -MAX_PWM, MAX_PWM);

  motor_obj.bias_prev = bias; // 更新上次偏差

  return (int)motor_obj.pwm_accum; // 返回整数 PWM 值
}


// --- 定时中断回调函数 (每 10ms 执行一次) ---
void control() {
  // 读取并重置位置计数器和调试计数器
  noInterrupts(); // 禁用所有中断，保护 volatile 变量
  float pulses_10ms = encoder_count;
  
  encoder_count = 0; // 重置编码器计数
  interrupts(); // 重新使能所有中断

  motor.current_velocity_pulses = pulses_10ms;

  // 执行 PID 计算
  motor.motor_pwm_output = Incremental_PI(motor.current_velocity_pulses, motor.target_velocity_pulises, motor);

  // 当目标速度和实际速度都为 0 时，强制归零 PID 状态和 PWM 输出。
  if (motor.target_velocity_pulises == 0 && motor.current_velocity_pulses == 0) {
    motor.pwm_accum = 0;   // 重置积分项
    motor.bias_prev = 0;   // 重置上次偏差
    motor.motor_pwm_output = 0; // 确保 PWM 输出也归零
  }

  // --- 串口输出：简洁模式 ---
  // 输出格式：目标值\t实际值（脉冲数/10ms，用于Python端累积里程计）
  Serial.print(motor.target_velocity_pulises, 2); // 目标速度（脉冲数/10ms）
  Serial.print("\t");
  Serial.println(motor.current_velocity_pulses, 2); // 实际速度（脉冲数/10ms）
  // 移除了第三列数据，因为它与第二列完全相同，避免冗余


  // --- 输出 PWM 控制信号 ---
  Set_Pwm(motor);
}

// --- Setup 函数 ---
void setup() {
  Serial.begin(115200); // 初始化串口

  // 初始化电机引脚
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  analogWrite(IN1, 0); // 初始 PWM 为 0
  analogWrite(IN2, 0); // 初始 PWM 为 0

  // 初始化编码器引脚
  // 编码器通常需要上拉电阻，INPUT_PULLUP 会启用内部上拉
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  // 设置定时中断
  MsTimer2::set(TIMER_INTERVAL, control);  // 设置定时中断，每 10ms 调用一次 control()
  MsTimer2::start();                       // 启动定时中断

  // --- 关键修复: 编码器中断 (使用 CHANGE 模式，同时监听 ENCODER_A 和 ENCODER_B) ---
  // 这将实现真正的四倍频计数 (X4)
  // 请确保 ENCODER_A 和 ENCODER_B 连接到支持外部中断的引脚 (如 Arduino Uno 的数字引脚 2 和 3)
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), READ_ENCODER_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), READ_ENCODER_B, CHANGE); // 启用 B 相中断

  //Serial.println("System Initialized - 10ms Sampling Mode");
  //Serial.println("输出格式：目标\t实际\tISR总次数\t累计脉冲"); // 调整输出提示
}

// --- Loop 函数 ---
void loop() {
  if (Serial.available()) { // 检查是否有数据可读
    String command = Serial.readStringUntil('\n'); // 读取直到换行符的整行数据
    command.trim(); // 移除首尾空白字符

    // 直接尝试将整个字符串解析为浮点数
    float input_value = command.toFloat(); 

    // 检查输入范围
    if (input_value >= MIN_VELOCITY_PULSES && input_value <= MAX_VELOCITY_PULSES) {
      motor.target_velocity_pulises = input_value;
      // 可以添加调试信息来确认是否成功设置了目标值
      // Serial.print("Target set to: ");
      // Serial.println(motor.target_velocity_pulises);
    } else {
      // 可以在这里打印错误信息，比如超出范围或无效输入
      // Serial.print("Invalid input or out of range: ");
      // Serial.println(command);
    }
  }
}