# Orin ⇄ Arduino 室內任務執行

## 概要
此模組整合室內任務流程，包括：
- 秤重檢測
- 伸縮步進機構
- LED 狀態顯示
- 馬達/伺服協同控制

由 **Orin (Python)** 負責任務邏輯與影像/感測器判斷，並透過 USB 串列傳送命令給 **Arduino** 控制子系統。

---

## 程式碼
- Arduino：`testMission_Arduino_Indoor.ino`  
- Orin (Python)：`testMission_indoor.py`
