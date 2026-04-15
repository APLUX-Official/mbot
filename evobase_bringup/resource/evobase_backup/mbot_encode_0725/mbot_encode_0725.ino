#include <MsTimer2.h>

// ======================= 配置开关 =======================
//！！！！！！！！！目前不用！！！！！！！！！！！！！！
// !! 重要 !!
// 为左轮编译上传时，将此值设为 0
// 为右轮编译上传时，将此值设为 1
#define IS_RIGHT_WHEEL 0
// =======================================================


#define IN1 6
#define IN2 5
#define ENCODER_A 2
#define ENCODER_B 3

volatile long encoder_count = 0;
volatile uint8_t encoder_state;
const int8_t encoder_lut[] = {
  0, -1,  1,  0,
  1,  0,  0, -1,
 -1,  0,  0,  1,
  0,  1, -1,  0
};

float target_pulses = 0;
float pwm_accum = 0;
float bias_prev = 0;

const int MAX_PWM = 255;
const float KP = 90.0;
const float KI = 6.0;
const float SCALE = 26.0;

void readEncoder() {
  uint8_t current_state = (digitalRead(ENCODER_A) << 1) | digitalRead(ENCODER_B);
  uint8_t index = (encoder_state << 2) | current_state;
  //电机
  encoder_count += encoder_lut[index];
  //encoder_count += encoder_lut[index]
  encoder_state = current_state;
}

void Set_Pwm(int pwm) {
  // --- MODIFICATION 2: 处理右轮方向反转 ---
  // 如果是右轮，则将PWM指令的符号反转，以匹配Python节点的逻辑
#if IS_RIGHT_WHEEL
  pwm = -pwm;
#endif

  pwm = constrain(pwm, -MAX_PWM, MAX_PWM);
  if (pwm < 0) {
    analogWrite(IN1, abs(pwm));
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    analogWrite(IN2, abs(pwm));
  }
}

void control() {
  noInterrupts();
  float pulses = encoder_count;
  encoder_count = 0;
  interrupts();

  float bias = target_pulses - pulses;
  float delta_pwm = KP * (bias - bias_prev) / SCALE + KI * bias / SCALE;
  pwm_accum += delta_pwm;
  pwm_accum = constrain(pwm_accum, -MAX_PWM, MAX_PWM);
  bias_prev = bias;

  if (target_pulses == 0 && abs(pulses) <= 1) {
    pwm_accum *= 0.9;
    if(abs(pwm_accum) < 1.0) pwm_accum = 0;
    bias_prev = 0;
  }

  Set_Pwm((int)pwm_accum);

  // --- MODIFICATION 1: 更改串口输出格式 ---
  // 输出格式: "目标脉冲 实际脉冲\n" (用空格分隔)
  // 以匹配Python节点的解析逻辑
  Serial.print(target_pulses, 2);
  Serial.print(" ");
  Serial.println(pulses, 2);
}

void setup() {
  Serial.begin(115200);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  
  encoder_state = (digitalRead(ENCODER_A) << 1) | digitalRead(ENCODER_B);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), readEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), readEncoder, CHANGE);

  MsTimer2::set(10, control);
  MsTimer2::start();
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    float val = cmd.toFloat();
    // 对目标值进行一个合理的范围限制，防止异常指令
    if (val >= -100 && val <= 100) {
      target_pulses = val;
    }
  }
}