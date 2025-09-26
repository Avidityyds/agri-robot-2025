# 🤖 Agricultural Robot 2025 | 2025 農業機器人競賽

This repository contains our work for the **2025 Agricultural Robotics Competition**, 
including outdoor red-line following, indoor mission execution, Arduino/Orin control integration, 
and technical documentation.

本專案包含 2025 農業機器人競賽相關程式與文件，涵蓋 **室外紅線循跡、室內任務執行、Orin⇄Arduino 控制整合** 等模組。

- [Project Notes on Notion](https://www.notion.so/26589de4c16a80ebbe8ad6b20c20b73c)
---

## Hardware Setup
- Jetson Orin Nano
- Arduino Uno
- Motor Driver (BTS7960 H-Bridge)
- DC Motors (Left/Right)
- Stepper Motor (28BYJ48 + ULN2003)
- Servo SG90
- Weight Sensor HX711
- USB Cameras
- Indicator LEDs (R/G/Y)

## Software Environment
- Python 3.9+
- OpenCV
- NumPy
- Ultralytics YOLOv12n
- pySerial
- Arduino IDE

---

## Repository Structure
```text
agri-robot-2025/
├── docs/                       # 技術文件（影像處理 / 控制 / 室內任務）
│   ├── vision/                 # OpenCV 紅線檢測（室內 / 室外）
│   ├── control/                # Orin⇄Arduino PWM / PID 控制
│   ├── indoor/                 # 室內任務、HX711 校正、指令測試
│   ├── model/                  # 模型測試
│   ├── HARDWARE.md             # 硬體接線總表
│   ├── COMPETITION_BOOK.pdf    # 秩序冊 PDF
│   └── images/
│       └── robot.jpg           # 機器人照片
│
├── code_indoor/                     # 室內任務程式碼
│   ├── testMission_indoor.py
│   ├── testMission_command.py
│   └── testMission_Arduino_Indoor.ino
│
├── code_control/                    # 控制相關程式碼
│   ├── testMotor_Orin.py
│   ├── testMotor_Arduino_Outdoor.ino
│   ├── testPID_Indoor.py
│   └── testPID_Outdoor.py
│
├── code_vision/                     # 影像處理程式碼
│   ├── testCam_Indoor.py
│   ├── testCam_Outdoor.py
│   └── testCam_Index.py
│
├── code_model/                      # 模型測試程式
│   ├── testModel_Orin.py
│   └── testModel_PC.py
│
├── code_sensors/                    # 感測器程式
│   └── testHX711Calibration.ino
│
├── README.md                   # 專案說明文件（本檔案）
└── LICENSE                     # MIT 授權
```
---

## Competition Reference | 競賽參考資料

- [2025 三久生物機電盃 全國農業機器人競賽 秩序冊 (PDF)](docs/COMPETITION_BOOK.pdf)

---

## Our Robot | 我們的機器人

![Robot Photo](docs/images/robot.jpg)
