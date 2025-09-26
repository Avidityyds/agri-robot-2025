#include <Stepper.h>
#include <Servo.h> 
#include "HX711.h"

// 指令：
//   PING                  // 回 PONG（測試用）
//   L:<vL> R:<vR>         // 車輛馬達控制 PWM（-255..255）
//   STOP                  // 車輛停止
//   LED <R|G|Y>           // LED亮指定色（會關掉其他色），保持亮直到 LED_STOP
//   LED_STOP              // LED全關
//   STRETCH               // 步進馬達伸5圈→收5圈（測試用）
//   STRETCH <grams>       // 步進馬達伸5圈→等重量達目標→立刻收回3圈→ 回 Orin STRETCH_OK / STRETCH_ERR
//   UP [deg]              // 伺服相對「正轉」deg（預設 15°）；回 UP_OK <now_deg>
//   DOWN [deg]            // 伺服相對「反轉」deg（預設 15°）；回 DOWN_OK <now_deg>

// ====== 腳位、參數設定 ======
// 左馬達
const int L_EN_FWD = 2;
const int L_EN_REV = 4;
const int L_PWM_FWD = 3;   
const int L_PWM_REV = 6;   
// 右馬達
const int R_EN_FWD = 7;
const int R_EN_REV = 8;
const int R_PWM_FWD = 5;   
const int R_PWM_REV = 9;   
// LED
const int LED_R = A5;
const int LED_G = 12;
const int LED_Y = 13;
// 步進馬達 28BYJ48 + ULN2003
const int STEPS_PER_REV = 2048;
const int STP_IN1 = A0;   // IN1
const int STP_IN2 = A1;   // IN2
const int STP_IN3 = A2;   // IN3
const int STP_IN4 = A3;   // IN4
Stepper stepperMotor(STEPS_PER_REV, STP_IN1, STP_IN3, STP_IN2, STP_IN4);
// 重量感測器 HX711
const int HX_DOUT = A4;   // DOUT
const int HX_SCK  = 11;   // SCK  
HX711 scale;
float SCALE_CAL = -670.0f;   // 校正係數（校正後更新
const float WEIGHT_TOL_G   = 3.0f;        // 允許誤差 ±g
const int   RPM_EXTEND     = 20;          // 伸出速度（RPM）
const int   RPM_RETRACT    = 20;          // 收回速度（RPM）
const unsigned long MAX_WAIT_MS = 20000;  // 等待收料的最長時間（安全逾時）
// 伺服馬達 SG90
const int SERVO_PIN            = 10; // Servo 用 Timer1；attach 期間會關掉 D9/D10 PWM
const int SERVO_INIT_DEG       = 0;  // 程式啟動預設角度（內部狀態）
const int SERVO_SETTLE_MS      = 15;  // 每 1° 的等待（平滑用）
const int SERVO_DEFAULT_DELTA  = 15;  // UP/DOWN 未給角度時的預設位移
const int SERVO_US_MIN         = 500;
const int SERVO_US_MAX         = 2400;
const int SERVO_MIN_DEG        = 0;
const int SERVO_MAX_DEG        = 180;
int servoAngle = SERVO_INIT_DEG;    // 目前記錄的伺服角度（0..180）
Servo feederServo;
// =====================

String buf;

// ------- 小工具 -------
int clampInt(int v, int lo, int hi) { if (v < lo) return lo; if (v > hi) return hi; return v; }

// ------- 馬達控制 -------
void setMotorPWM(int enFwd, int enRev, int pwmFwd, int pwmRev, int speed) {
  int pwm = abs(speed);
  if (pwm > 255) pwm = 255;

  if (speed > 0) {
    // 前進：FWD PWM 有值、REV PWM=0
    digitalWrite(enFwd, HIGH);
    digitalWrite(enRev, HIGH);
    analogWrite(pwmFwd, pwm);
    analogWrite(pwmRev, 0);
  } else if (speed < 0) {
    // 後退：REV PWM 有值、FWD PWM=0
    digitalWrite(enFwd, HIGH);
    digitalWrite(enRev, HIGH);
    analogWrite(pwmFwd, 0);
    analogWrite(pwmRev, pwm);
  } else {
    // 停止：PWM 皆 0
    digitalWrite(enFwd, HIGH);
    digitalWrite(enRev, HIGH);
    analogWrite(pwmFwd, 0);
    analogWrite(pwmRev, 0);
  }
}

void stopMotors() {
  setMotorPWM(L_EN_FWD, L_EN_REV, L_PWM_FWD, L_PWM_REV, 0);
  setMotorPWM(R_EN_FWD, R_EN_REV, R_PWM_FWD, R_PWM_REV, 0);
}

// ------- LED 控制 -------
void ledsAllOff() {
  digitalWrite(LED_R, LOW);
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_Y, LOW);
}
void setLEDColor(char c) {
  ledsAllOff();
  c = toupper(c);
  if (c == 'R') digitalWrite(LED_R, HIGH);
  else if (c == 'G') digitalWrite(LED_G, HIGH);
  else if (c == 'Y') digitalWrite(LED_Y, HIGH);
}

// ------- 解析 L:<vL> R:<vR> -------
bool handleLRCommand(const String& s) {
  int lpos = s.indexOf("L:");
  int rpos = s.indexOf("R:");
  if (lpos < 0 || rpos < 0) return false;

  if (rpos < lpos) {
    int lpos2 = rpos; 
    int rpos2 = lpos;
    lpos = lpos2; 
    rpos = rpos2;
  }

  // 抓數值
  int vL = s.substring(lpos + 2, rpos).toInt();
  int vR = s.substring(rpos + 2).toInt();

  setMotorPWM(L_EN_FWD, L_EN_REV, L_PWM_FWD, L_PWM_REV, vL);
  setMotorPWM(R_EN_FWD, R_EN_REV, R_PWM_FWD, R_PWM_REV, vR);

  Serial.print("LR_OK "); Serial.print(vL); Serial.print(' '); Serial.println(vR); // ← debug 回覆

  return true;
}

// ------- 平均取重（已 set_scale/tare） -------
float readWeightAvg(uint8_t n = 6) {
  if (!scale.is_ready()) return NAN;
  float sum = 0; int cnt = 0;
  for (uint8_t i = 0; i < n; i++) {
    float g = scale.get_units(1);
    if (!isnan(g)) { sum += g; cnt++; }
    delay(5);
  }
  return (cnt > 0) ? (sum / cnt) : NAN;
}

// ------- STRETCH <g> -------
bool stretch_to_weight(float target_g) {
  if (target_g <= 0) { Serial.println("STRETCH_ERR BAD_ARG"); return false; }
  if (!scale.is_ready()) { Serial.println("STRETCH_ERR SCALE_NOT_READY"); return false; }

  // 歸零
  scale.tare(20);
  delay(80);

  // 伸 5 圈
  stepperMotor.setSpeed(RPM_EXTEND);
  stepperMotor.step(-5 * STEPS_PER_REV);  // 往外伸；若方向反了就把符號換成

  // 盯重量直到達標（目標 - 容差），一到就結束等待
  bool reached = false;
  unsigned long t0 = millis();
  while (millis() - t0 < MAX_WAIT_MS) {
    float w = readWeightAvg(4);
    if (!isnan(w) && w >= (target_g - WEIGHT_TOL_G)) {
      reached = true;
      break;               // 達標就立刻收回
    }
    delay(20);
  }

  // 收回 3 圈
  stepperMotor.setSpeed(RPM_RETRACT);
  stepperMotor.step(3 * STEPS_PER_REV);

  // 回報結果
  if (reached) {
    Serial.println("STRETCH_OK");
    return true;
  } else {
    Serial.println("STRETCH_ERR TIMEOUT");
    return false;
  }
}

// ------- 伺服馬達相對移動（UP/DOWN 共用） -------
void servoMoveRelative(int deltaDeg, bool reportUp) {
  // 安全：停車時才轉伺服
  stopMotors();

  // 目標角度（夾在 0..180）
  long target = (long)servoAngle + (long)deltaDeg;
  target = clampInt((int)target, SERVO_MIN_DEG, SERVO_MAX_DEG);

  if ((int)target == servoAngle) {
    Serial.print(reportUp ? "UP_OK " : "DOWN_OK ");
    Serial.println(servoAngle);
    return;
  }

  int step = (target >= servoAngle) ? 1 : -1;

  // 擴脈寬範圍以利到達接近 180°
  feederServo.attach(SERVO_PIN, SERVO_US_MIN, SERVO_US_MAX);
  feederServo.write(servoAngle);
  delay(SERVO_SETTLE_MS);

  for (int a = servoAngle + step; (step > 0) ? (a <= target) : (a >= target); a += step) {
    feederServo.write(a);
    delay(SERVO_SETTLE_MS);
  }
  servoAngle = (int)target;

  feederServo.detach();                 // 釋放 Timer1

  Serial.print(reportUp ? "UP_OK " : "DOWN_OK ");
  Serial.println(servoAngle);
}

// ------- 指令處理 -------
void handleCommand(String s) {
  s.trim();
  if (!s.length()) return;

  // L:<vL> R:<vR>
  if (handleLRCommand(s)) return;

  String t = s;
  t.toUpperCase();

  if (t == "PING") {
    Serial.println("PONG");
    return;
  }

  if (t == "STOP") {
    stopMotors();
    Serial.println("STOP OK");
    return;
  }

  if (t == "LED_STOP") {
    ledsAllOff();
    Serial.println("LED_STOP OK");
    return;
  }

  // LED <R|G|Y>
  if (t.startsWith("LED ")) {
    int sp1 = s.indexOf(' ');
    if (sp1 > 0 && s.length() >= sp1 + 2) {
      char c = s.substring(sp1 + 1, sp1 + 2)[0];
      setLEDColor(c);     // 亮該色，保持亮直到 LED_STOP
      Serial.println("LED OK");
    }
    return;
  }

  // STRETCH（伸縮機構動作）
  if (t.startsWith("STRETCH")) {
    int sp1 = s.indexOf(' ');
    if (sp1 > 0) {
      // STRETCH <grams>
      float grams = s.substring(sp1 + 1).toFloat();
      stretch_to_weight(grams);  // 完成印 STRETCH_OK；逾時印 STRETCH_ERR TIMEOUT
    } else {
      // STRETCH（測試）
      stepperMotor.setSpeed(20);
      stepperMotor.step(-5 * STEPS_PER_REV); //往外伸
      stepperMotor.step(+5 * STEPS_PER_REV); //往內縮
      Serial.println("STRETCH TEST OK");
    }
    return;
  }

  // SCALE?（重量感測器測試）
  if (t == "SCALE?") {
    if (!scale.is_ready()) {
      Serial.println("WEIGHT NAN");
      return;
    }
    float w = scale.get_units(8);   // 取 8 次平均
    Serial.print("WEIGHT ");
    Serial.println(w, 1);
    return;
  }

  // UP [deg]：相對「正轉」 → 送正角度
  if (t == "UP" || t.startsWith("UP ")) {
    int delta = SERVO_DEFAULT_DELTA;
    int sp1 = s.indexOf(' ');
    if (sp1 > 0) {
      String num = s.substring(sp1 + 1); num.trim();
      if (num.length() > 0) delta = num.toInt();
    }
    delta = clampInt(delta, 0, 180);
    servoMoveRelative(+delta, true);
    return;
  }

  // DOWN [deg]：相對「反轉」 → 送負角度
  if (t == "DOWN" || t.startsWith("DOWN ")) {
    int delta = SERVO_DEFAULT_DELTA;
    int sp1 = s.indexOf(' ');
    if (sp1 > 0) {
      String num = s.substring(sp1 + 1); num.trim();
      if (num.length() > 0) delta = num.toInt();
    }
    delta = clampInt(delta, 0, 180);
    servoMoveRelative(-delta, false);
    return;
  }

  // 其它代補指令
}

void setup() {
  // 馬達腳位
  pinMode(L_EN_FWD, OUTPUT);
  pinMode(L_EN_REV, OUTPUT);
  pinMode(L_PWM_FWD, OUTPUT);
  pinMode(L_PWM_REV, OUTPUT);
  pinMode(R_EN_FWD, OUTPUT);
  pinMode(R_EN_REV, OUTPUT);
  pinMode(R_PWM_FWD, OUTPUT);
  pinMode(R_PWM_REV, OUTPUT);

  // LED 腳位
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_Y, OUTPUT);
  ledsAllOff();

  // 步進馬達初始化 
  stepperMotor.setSpeed(20);  // RPM

  // 重量感測器
  scale.begin(HX_DOUT, HX_SCK);
  scale.set_scale(SCALE_CAL); // 校正因子
  scale.tare(20);

  // 伺服馬達（不 attach，等用到再 attach）
  servoAngle = SERVO_INIT_DEG;  // 起始 0°

  Serial.begin(9600);         
  stopMotors();               // 安全起步
}

void loop() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (buf.length() > 0) {
        handleCommand(buf);
        buf = "";
      }
    } else {
      buf += ch;
      if (buf.length() > 120) buf = ""; // 防呆
    }
  }
}
