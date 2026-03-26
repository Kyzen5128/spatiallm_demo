# SLAM3R → SpatialLM 完整推論管線

從影片生成 3D 點雲，到自動偵測室內佈局的端對端流程。

---

## 📋 總覽

```
影片 (.mp4)
  ↓  SLAM3R
原始點雲 (.ply)  ← 通常歪斜、無比例尺
  ↓  pipeline/preprocess_aligned.py
對齊點雲 (.ply)  ← 地板水平、牆面軸對齊、高度 2.5m
  ↓  inference.py
佈局文字 (.txt)  ← 牆壁/門/窗/家具座標
  ↓  visualize.py
3D 視覺化 (.rrd) ← 用 Rerun 開啟
```

---

## 🚀 快速開始（一鍵執行）

```bash
cd /home/kyzen/SpatialLM
conda activate spatiallm

bash pipeline/run_pipeline.sh <輸入點雲.ply> <輸出目錄> [--flip]
```

**實際範例：**

```bash
bash pipeline/run_pipeline.sh \
    /home/kyzen/SLAM3R/results/Replica_demo/Replica_demo_room0_recon.ply \
    result/Replica_demo/improved \
    --flip
```

完成後用 Rerun 查看結果：

```bash
rerun result/Replica_demo/improved/result.rrd --web-viewer
# 瀏覽器開 http://localhost:9090
```

---

## 📖 分步執行

如果需要逐步控制，可以手動執行每一步：

### 第 0 步：用 SLAM3R 從影片生成點雲

```bash
cd /home/kyzen/SLAM3R

# 準備影片幀（放在 data/<場景名稱>/ 下）
# 執行 SLAM3R 重建
bash scripts/demo_replica.sh
```

輸出位置：`/home/kyzen/SLAM3R/results/<場景名稱>/<場景名稱>_recon.ply`

> **注意**：SLAM3R 輸出的點雲通常有三個問題：
> - 地板不水平（歪斜或顛倒）
> - 牆面沒有對齊座標軸
> - 比例尺不是公尺制

### 第 1 步：點雲前處理（對齊 + 縮放）

```bash
cd /home/kyzen/SpatialLM
conda activate spatiallm

python pipeline/preprocess_aligned.py \
    --input /home/kyzen/SLAM3R/results/Replica_demo/Replica_demo_room0_recon.ply \
    --output result/Replica_demo/improved/aligned_pointcloud.ply \
    --target_height 2.5 \
    --flip
```

**前處理做了什麼（7 步驟）：**

| 步驟 | 說明 | 方法 |
|------|------|------|
| 1 | 載入點雲 | `open3d.io.read_point_cloud` |
| 2 | 找地板平面 | RANSAC (`segment_plane`) |
| 3 | 對齊地板到水平 | Rodrigues 旋轉公式 |
| 4 | 去噪 | Statistical Outlier Removal |
| 5 | 縮放到 2.5m 高度 | `scale = 2.5 / 當前高度` |
| 6 | 對齊牆面到座標軸 | OpenCV `minAreaRect` |
| 7 | 位置調整 | 地板移到 z=0，中心移到原點 |

**參數說明：**

| 參數 | 說明 | 預設值 |
|------|------|--------|
| `--input` | 輸入點雲 (.ply) | 必填 |
| `--output` | 輸出點雲 (.ply) | 必填 |
| `--target_height` | 目標房間高度（公尺） | `2.5` |
| `--flip` | 上下翻轉 180°（SLAM3R 常需要） | 關閉 |

> **什麼時候要加 `--flip`？**
> 如果 SLAM3R 產出的點雲是上下顛倒的就要加。可以先不加跑一次，如果推論結果很差再加 `--flip` 重跑。

### 第 2 步：SpatialLM 推論

```bash
python inference.py \
    --point_cloud result/Replica_demo/improved/aligned_pointcloud.ply \
    --output result/Replica_demo/improved/layout.txt \
    --model_path models/SpatialLM1.1-Qwen-0.5B
```

> ⚠️ `--model_path` 一定要用 **本地路徑** `models/SpatialLM1.1-Qwen-0.5B`，
> 不要用 `manycore-research/SpatialLM1.1-Qwen-0.5B`（那會重新從網路下載）。

### 第 3 步：生成 3D 視覺化

```bash
python visualize.py \
    --point_cloud result/Replica_demo/improved/aligned_pointcloud.ply \
    --layout result/Replica_demo/improved/layout.txt \
    --save result/Replica_demo/improved/result.rrd
```

### 第 4 步：用 Rerun 查看結果

```bash
rerun result/Replica_demo/improved/result.rrd --web-viewer
# 瀏覽器開 http://localhost:9090
```

---

## 📁 檔案結構

```
SpatialLM/
├── pipeline/                        # 前處理工具
│   ├── preprocess_aligned.py        #   7 步驟點雲對齊腳本
│   ├── run_pipeline.sh              #   一鍵執行完整流程
│   └── README.md                    #   技術細節說明
├── inference.py                     # SpatialLM 推論
├── visualize.py                     # 生成 Rerun 視覺化
├── models/
│   └── SpatialLM1.1-Qwen-0.5B/     # 本地模型權重
├── result/                          # 輸出結果
│   └── Replica_demo/
│       └── improved/
│           ├── aligned_pointcloud.ply  # 對齊後點雲
│           ├── layout.txt              # 佈局偵測結果
│           └── result.rrd              # Rerun 視覺化檔
└── PIPELINE_README.md               # ← 本文件
```

---

## ❓ 常見問題

### 推論結果很差（物件偵測不到或框歪了）

1. 先確認是否加了 `--flip`（或移除 `--flip`），重跑前處理
2. 檢查前處理輸出的最終尺寸，高度應為 `2.500m`
3. SpatialLM 對「中國式公寓」場景辨識最佳，非典型房間可能效果較差（模型先天偏見）

### 推論跑很久 / GPU 沒在動

- 如果是第一次用 HuggingFace ID 當 `--model_path`，它會先下載模型（~2.4GB）
- 改用本地路徑 `models/SpatialLM1.1-Qwen-0.5B` 就不用下載

### Rerun 打不開

- SSH 遠端連線要加 `--web-viewer`，然後瀏覽器開 `http://<伺服器IP>:9090`
- 本機可以直接 `rerun result.rrd`（會開桌面版）
