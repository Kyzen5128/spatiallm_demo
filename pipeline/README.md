# SpatialLM 點雲預處理優化

解決 SLAM3R 生成的點雲方向不正確的問題，讓 SpatialLM 能正確檢測房間佈局。

## 問題

SLAM3R 輸出的點雲通常：
- 地板不在水平面（歪斜或顛倒）
- 牆面沒有對齊到座標軸
- 導致 SpatialLM 無法正確理解場景

## 解決方案

使用 `preprocess_aligned.py` 自動對齊點雲：
1. **RANSAC 找地板** - 自動檢測地板平面
2. **Rodrigues 旋轉** - 將地板對齊到水平
3. **去噪** - 移除 SLAM 噪點
4. **縮放** - 調整到標準高度 2.5m
5. **牆面對齊** - 用 MinAreaRect 對齊牆面到座標軸
6. **位置調整** - 地板移到 z=0，中心移到原點

## 快速開始

### 方法一：一鍵運行

```bash
cd /home/kyzen/SpatialLM
bash 優化/run_pipeline.sh <輸入點雲> <輸出目錄> [--flip]

# 範例
bash 優化/run_pipeline.sh \
    /home/kyzen/SLAM3R/results/Replica_demo/Replica_demo_room0_recon.ply \
    result/Replica_demo/improved \
    --flip
```

### 方法二：分步執行

```bash
# 1. 激活環境
source ~/miniconda3/bin/activate spatiallm
cd /home/kyzen/SpatialLM

# 2. 預處理（如果顛倒加 --flip）
python 優化/preprocess_aligned.py \
    --input /home/kyzen/SLAM3R/results/Replica_demo/Replica_demo_room0_recon.ply \
    --output result/Replica_demo/improved/aligned_pointcloud.ply \
    --target_height 2.5 \
    --flip

# 3. 推理
python inference.py \
    --point_cloud result/Replica_demo/improved/aligned_pointcloud.ply \
    --output result/Replica_demo/improved/layout.txt \
    --model_path models/SpatialLM1.1-Qwen-0.5B

# 4. 可視化
python visualize.py \
    --point_cloud result/Replica_demo/improved/aligned_pointcloud.ply \
    --layout result/Replica_demo/improved/layout.txt \
    --save result/Replica_demo/improved/result.rrd

# 5. 查看結果
rerun result/Replica_demo/improved/result.rrd --web-viewer
```

## 參數說明

### preprocess_aligned.py

| 參數 | 說明 | 預設值 |
|------|------|--------|
| `--input` | 輸入點雲檔案 (.ply) | 必填 |
| `--output` | 輸出點雲檔案 (.ply) | 必填 |
| `--target_height` | 目標房間高度（公尺） | 2.5 |
| `--flip` | 上下翻轉點雲 | 關閉 |

## 常見問題

### Q: 結果上下顛倒怎麼辦？
A: 加上 `--flip` 參數重新處理

### Q: 牆面還是有點斜怎麼辦？
A: MinAreaRect 對齊可能不完美，可以手動調整 yaw 角度

### Q: DBScan 聚類當機怎麼辦？
A: 這個腳本已經移除了 DBScan（太耗記憶體），只保留基本去噪

## 檔案說明

| 檔案 | 說明 |
|------|------|
| `preprocess_aligned.py` | 主要預處理腳本（自動對齊） |
| `run_pipeline.sh` | 一鍵運行完整流程 |
| `README.md` | 本文件 |

## 技術細節

### Rodrigues 旋轉

用於將任意方向的地板平面旋轉到水平：

```python
# 找地板法向量
normal = RANSAC_find_plane(points)

# 目標：讓法向量對齊到 Z 軸
target = [0, 0, 1]

# 計算旋轉軸和角度
axis = cross(normal, target)
angle = arccos(dot(normal, target))

# Rodrigues 公式
K = skew_matrix(axis)
R = I + sin(θ)K + (1-cos(θ))K²
```

### MinAreaRect 牆面對齊

用 OpenCV 的最小外接矩形找牆面主方向：

```python
wall_points = points[0.3m < z < 2.0m]
rect = cv2.minAreaRect(wall_points[:, :2])
angle = rect[2]  # 旋轉角度
```

---

**更新日期：2026-03-23**
