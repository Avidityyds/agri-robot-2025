# Orin ⇄ Arduino PWM 控制

## 概要
- Orin（Python / pySerial）經 USB 串列埠送指令給 Arduino
- Arduino 解析指令後，輸出**左/右馬達的 PWM 與方向**

---

## 接線對照

**前左馬達（FL）**

| 訊號       | Arduino 腳位 |
|------------|--------------|
| EN_FL_FWD  | 2            |
| EN_FL_REV  | 4            |
| PWM_FL_FWD | 3 (PWM)      |
| PWM_FL_REV | 6 (PWM)      |

**前右馬達（FR）**

| 訊號       | Arduino 腳位 |
|------------|--------------|
| EN_FR_FWD  | 7            |
| EN_FR_REV  | 8            |
| PWM_FR_FWD | 5 (PWM)      |
| PWM_FR_REV | 9 (PWM)      |

---

## 串列通訊格式
- 指令：`<L_PWM> <R_PWM>\n`
- 範例：`120 120\n`
- 回應：`OK L=120 R=120`

---

## 程式碼
- Arduino：`testMotor_Arduino_outdoor.ino`  
- Orin（Python）：`testMotor_Orin.py`

---

## 使用步驟
1. **接線**依上表連好驅動與 Arduino 腳位  
2. **燒錄 Arduino**（波特率 9600）  
3. **確認 Orin 裝置**
    
        ls /dev/ttyACM*
    
   看到 `/dev/ttyACM0` 後，在程式中設定 `PORT`。  
4. **執行 Python 程式**：於終端輸入數值，例如 `120 120`、`80 0`；`q` 離開

---

## 常見注意事項
- `ls /dev/ttyACM*` 找不到：檢查 USB 與供電
- 權限不足：使用 `sudo` 或把使用者加入 `dialout` 群組
- PWM 範圍：0–255
