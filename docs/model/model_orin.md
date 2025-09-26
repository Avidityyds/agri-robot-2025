# Orin 測試模型偵測

## 概要
這份程式在 **Jetson Orin** 上執行，用來測試 YOLO 模型能否即時進行物件偵測。

主要功能：
- 載入 YOLO 模型（使用 `best.pt` 權重）
- 開啟攝影機，設定解析度與編碼
- 即時推論並顯示結果
- 疊加 FPS 於畫面
- 按下 `q` 鍵退出程式

---

## 程式碼
- Orin（Python）：`testModel_Orin.py`
