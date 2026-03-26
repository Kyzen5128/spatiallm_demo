#!/bin/bash
#
# SpatialLM 完整處理流程（一鍵運行）
#
# 用法：
#   bash run_pipeline.sh <輸入點雲> <輸出目錄> [--flip]
#
# 範例：
#   bash run_pipeline.sh /home/kyzen/SLAM3R/results/Replica_demo/Replica_demo_room0_recon.ply result/Replica_demo/improved --flip
#

set -e

# 檢查參數
if [ "$#" -lt 2 ]; then
    echo "用法: bash run_pipeline.sh <輸入點雲> <輸出目錄> [--flip]"
    echo ""
    echo "範例:"
    echo "  bash run_pipeline.sh input.ply result/output"
    echo "  bash run_pipeline.sh input.ply result/output --flip"
    exit 1
fi

INPUT_PLY=$1
OUTPUT_DIR=$2
FLIP_FLAG=${3:-""}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SPATIALLM_DIR="$(dirname "$SCRIPT_DIR")"

# 激活環境
echo "=== 激活 conda 環境 ==="
source ~/miniconda3/bin/activate spatiallm
cd "$SPATIALLM_DIR"

# 創建輸出目錄
mkdir -p "$OUTPUT_DIR"

# 步驟 1: 預處理
echo ""
echo "=== 步驟 1/4: 預處理點雲 ==="
python "$SCRIPT_DIR/preprocess_aligned.py" \
    --input "$INPUT_PLY" \
    --output "$OUTPUT_DIR/aligned_pointcloud.ply" \
    --target_height 2.5 \
    $FLIP_FLAG

# 步驟 2: 推理
echo ""
echo "=== 步驟 2/4: SpatialLM 推理 ==="
python inference.py \
    --point_cloud "$OUTPUT_DIR/aligned_pointcloud.ply" \
    --output "$OUTPUT_DIR/layout.txt" \
    --model_path models/SpatialLM1.1-Qwen-0.5B

# 步驟 3: 可視化
echo ""
echo "=== 步驟 3/4: 生成可視化 ==="
python visualize.py \
    --point_cloud "$OUTPUT_DIR/aligned_pointcloud.ply" \
    --layout "$OUTPUT_DIR/layout.txt" \
    --save "$OUTPUT_DIR/result.rrd"

# 步驟 4: 顯示結果
echo ""
echo "=== 步驟 4/4: 完成！ ==="
echo ""
echo "輸出檔案："
echo "  - 對齊後點雲: $OUTPUT_DIR/aligned_pointcloud.ply"
echo "  - 佈局結果:   $OUTPUT_DIR/layout.txt"
echo "  - 可視化:     $OUTPUT_DIR/result.rrd"
echo ""
echo "查看結果："
echo "  rerun $OUTPUT_DIR/result.rrd --web-viewer"
echo ""
