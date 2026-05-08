#!/usr/bin/env python3
"""
SpatialLM 點雲預處理腳本 v3（最終版）

改善記錄：
- v1: 原始版（flip 在 scale 前，永遠失效）
- v2: 修正 flip 順序（scale 後才 flip），但去噪多餘且 threshold 硬碼
- v3: 移除去噪（SpatialLM inference.py 已有 cleanup_pcd）
      RANSAC 改用相對 threshold（Z range 的 0.5%）
      flip 偵測改用相對閾值（total_height 的 2%~50%）
      → scale 前後都可正確偵測，不再依賴特定步驟順序

步驟：
  1. 載入
  2. RANSAC 找最大水平面（相對 threshold）
  3. Rodrigues 旋轉對齊到水平
  4. 縮放到 2.5m
  5. 自動偵測地板方向（相對閾值，scale 無關）
  6. 牆面對齊（MinAreaRect）
  7. 地板移到 z=0，中心移到原點
"""

import argparse
import os
import numpy as np
import open3d as o3d
import cv2


def find_floor_normal(pcd, points):
    """用 RANSAC 找最大水平面的法向量。
    threshold 用 Z range 的 0.5%，避免硬碼公尺制假設。
    """
    z_range = np.max(points[:, 2]) - np.min(points[:, 2])
    threshold = max(1e-4, 0.005 * z_range)  # 0.5% of Z range, min 0.1mm

    plane_model, inliers = pcd.segment_plane(
        distance_threshold=threshold,
        ransac_n=3,
        num_iterations=1000
    )
    a, b, c, d = plane_model
    normal = np.array([a, b, c])
    normal = normal / np.linalg.norm(normal)
    return normal, inliers, threshold


def rodrigues_rotation(points, normal, target=np.array([0, 0, 1])):
    """Rodrigues 旋轉：讓偵測到的平面法向量對齊到 Z 軸"""
    if normal[2] < 0:
        normal = -normal

    axis = np.cross(normal, target)
    axis_norm = np.linalg.norm(axis)

    if axis_norm < 0.001:
        return points, 0.0

    axis /= axis_norm
    angle = np.arccos(np.clip(np.dot(normal, target), -1, 1))

    K = np.array([
        [0,       -axis[2],  axis[1]],
        [axis[2],  0,       -axis[0]],
        [-axis[1], axis[0],  0      ]
    ])
    R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K
    return points @ R.T, np.degrees(angle)


def score_floor_candidate(points, low_ratio=0.02, high_ratio=0.50):
    """
    計算「假設此方向為地板朝下時」的家具複雜度分數。

    使用相對閾值（total_height 的百分比），不依賴 scale：
    - 下界: 2% of height（貼地板的薄薄一層不算）
    - 上界: 50% of height（家具不超過房間高度的一半）

    分數 = 家具帶點數 × Z 軸標準差
    - 家具高度不一 → Z std 高
    - 天花板那側只有牆面/空氣 → Z std 低
    """
    pts = points.copy()
    pts[:, 2] -= np.min(pts[:, 2])
    total_h = np.max(pts[:, 2])

    if total_h < 1e-6:
        return 0.0

    low  = low_ratio  * total_h
    high = high_ratio * total_h

    band = pts[(pts[:, 2] > low) & (pts[:, 2] < high)]
    if len(band) < 50:
        return 0.0

    return len(band) * np.std(band[:, 2])


def auto_detect_flip(points):
    """
    比較兩個方向的家具複雜度，選分數高的。
    使用相對閾值，scale 前後皆可正確判斷。
    """
    score_a = score_floor_candidate(points)

    flipped = points.copy()
    flipped[:, 2] = -flipped[:, 2]
    score_b = score_floor_candidate(flipped)

    if score_b > score_a:
        return flipped, True, score_a, score_b
    return points, False, score_a, score_b


def align_walls(points):
    """MinAreaRect 對齊牆面到軸對齊（在 scale 後執行，0.3~2.0m 有效）"""
    wall_pts = points[(points[:, 2] > 0.3) & (points[:, 2] < 2.0)]

    if len(wall_pts) < 100:
        print("[!] 牆壁點數不足，跳過牆面對齊")
        return points, 0.0

    xy   = wall_pts[:, :2].astype(np.float32)
    rect = cv2.minAreaRect(xy)
    angle = rect[2]

    while angle <= -45:
        angle += 90
    while angle > 45:
        angle -= 90

    yaw    = np.radians(angle)
    cos_a  = np.cos(-yaw)
    sin_a  = np.sin(-yaw)
    rot_z  = np.array([
        [cos_a, -sin_a, 0],
        [sin_a,  cos_a, 0],
        [0,      0,     1]
    ])
    return points @ rot_z.T, angle


def main():
    parser = argparse.ArgumentParser(
        description="SpatialLM 點雲預處理 v3"
    )
    parser.add_argument("--input",         required=True,  help="輸入 .ply")
    parser.add_argument("--output",        required=True,  help="輸出 .ply")
    parser.add_argument("--target_height", type=float, default=2.5,
                        help="目標高度（公尺），預設 2.5")
    parser.add_argument("--flip",          action="store_true",
                        help="強制翻轉（覆蓋自動偵測）")
    args = parser.parse_args()

    # ── 1. 載入 ─────────────────────────────────────────────────
    print(f"[1/6] 載入點雲: {args.input}")
    pcd    = o3d.io.read_point_cloud(args.input)
    points = np.asarray(pcd.points)
    print(f"      點數: {len(points):,}")

    # ── 2. RANSAC 找最大水平面 ───────────────────────────────────
    print("[2/6] RANSAC 尋找最大水平面...")
    normal, inliers, threshold = find_floor_normal(pcd, points)
    print(f"      自動 threshold: {threshold:.5f}")
    print(f"      法向量: ({normal[0]:.3f}, {normal[1]:.3f}, {normal[2]:.3f})")
    print(f"      平面點數: {len(inliers):,}")

    # ── 3. Rodrigues 旋轉 ────────────────────────────────────────
    print("[3/6] Rodrigues 旋轉對齊到水平...")
    points, rot_angle = rodrigues_rotation(points, normal)
    print(f"      旋轉角度: {rot_angle:.1f}°")

    # ── 4. 縮放 ─────────────────────────────────────────────────
    print(f"[4/6] 縮放到 {args.target_height}m...")
    z_min, z_max = np.min(points[:, 2]), np.max(points[:, 2])
    scale  = args.target_height / (z_max - z_min)
    points = points * scale
    print(f"      縮放比例: {scale:.4f}x")

    # ── 5. 自動偵測地板方向 ──────────────────────────────────────
    print("[5/6] 自動偵測地板方向（相對閾值，scale 無關）...")

    if args.flip:
        points[:, 2] = -points[:, 2]
        did_flip = True
        sa = score_floor_candidate(points)
        sb = -1.0
        print(f"      [強制翻轉] --flip 旗標啟用")
    else:
        points, did_flip, sa, sb = auto_detect_flip(points)

    print(f"      ┌─ 不翻轉 家具分數: {sa:>12.1f}")
    print(f"      └─ 翻轉後 家具分數: {sb:>12.1f}")
    print(f"      → {'自動翻轉 Z 軸 ✓' if did_flip else '方向正確，不翻轉 ✓'}")

    # ── 6. 牆面對齊 + 位置調整 ──────────────────────────────────
    print("[6/6] 牆面對齊 + 位置調整...")
    points, wall_angle = align_walls(points)
    print(f"      牆面旋轉: {wall_angle:.2f}°")

    points[:, 2] -= np.min(points[:, 2])
    points[:, 0] -= (np.max(points[:, 0]) + np.min(points[:, 0])) / 2
    points[:, 1] -= (np.max(points[:, 1]) + np.min(points[:, 1])) / 2
    print(f"      地板對齊 z=0，中心移到原點")

    # ── 保存 ─────────────────────────────────────────────────────
    pcd.points = o3d.utility.Vector3dVector(points)
    os.makedirs(os.path.dirname(args.output) or '.', exist_ok=True)
    o3d.io.write_point_cloud(args.output, pcd)

    print()
    print("=== 最終尺寸 ===")
    print(f"X: {points[:,0].min():.3f} ~ {points[:,0].max():.3f}  "
          f"(寬 {points[:,0].max()-points[:,0].min():.3f}m)")
    print(f"Y: {points[:,1].min():.3f} ~ {points[:,1].max():.3f}  "
          f"(深 {points[:,1].max()-points[:,1].min():.3f}m)")
    print(f"Z: {points[:,2].min():.3f} ~ {points[:,2].max():.3f}  "
          f"(高 {points[:,2].max()-points[:,2].min():.3f}m)")
    print()
    print(f"[完成] 已保存: {args.output}")


if __name__ == "__main__":
    main()
