// ==== 前左馬達 ====
const int EN_FL_FWD   = 2;   
const int EN_FL_REV   = 4;   
const int PWM_FL_FWD  = 3;   
const int PWM_FL_REV  = 6;   

// ==== 前右馬達 ====
const int EN_FR_FWD   = 7;   
const int EN_FR_REV   = 8;   
const int PWM_FR_FWD  = 5;   
const int PWM_FR_REV  = 9;   

// speed > 0 前進，speed < 0 後退，speed = 0 停止
void setMotorPWM(int enFwd, int enRev, int pwmFwd, int pwmRev, int speed) {
  int pwm = abs(speed);
  if (pwm > 255) pwm = 255;

  if (speed > 0) {
    // 方向：前進 → FWD PWM 有值，REV PWM = 0
    digitalWrite(enFwd, HIGH);
    digitalWrite(enRev, HIGH);      
    analogWrite(pwmFwd, pwm);
    analogWrite(pwmRev, 0);
  } else if (speed < 0) {
    // 方向：後退 → REV PWM 有值，FWD PWM = 0
    digitalWrite(enFwd, HIGH);
    digitalWrite(enRev, HIGH);
    analogWrite(pwmFwd, 0);
    analogWrite(pwmRev, pwm);
  } else {
    // 停止：PWM 皆 = 0
    digitalWrite(enFwd, HIGH);
    digitalWrite(enRev, HIGH);
    analogWrite(pwmFwd, 0);
    analogWrite(pwmRev, 0);
  }
}

void setup() {
  Serial.begin(9600);

  int pins[] = {
    EN_FL_FWD, EN_FL_REV, PWM_FL_FWD, PWM_FL_REV,
    EN_FR_FWD, EN_FR_REV, PWM_FR_FWD, PWM_FR_REV
  };
  for (int i = 0; i < (int)(sizeof(pins)/sizeof(pins[0])); i++) {
    pinMode(pins[i], OUTPUT);
    digitalWrite(pins[i], LOW);
  }

  Serial.println("READY for PWM commands: L:<-255~255> R:<-255~255>");
}


void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n'); 
    input.trim();                                

    if (input.startsWith("L:")) {
      int pos = input.indexOf("R:");
      if (pos != -1) {
        int left  = input.substring(2, pos).toInt();
        int right = input.substring(pos + 2).toInt();

        Serial.print("OK L="); Serial.print(left);
        Serial.print(" R=");   Serial.println(right);

        setMotorPWM(EN_FL_FWD, EN_FL_REV, PWM_FL_FWD, PWM_FL_REV, left);
        setMotorPWM(EN_FR_FWD, EN_FR_REV, PWM_FR_FWD, PWM_FR_REV, right);
      } 

      else {
        Serial.println("ERR format. Use: L:<val> R:<val>");
      }
    }
  }
}
