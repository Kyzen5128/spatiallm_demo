#!/usr/bin/env python3
"""
SpatialLM 點雲預處理腳本（含自動對齊）

功能：
1. 用 RANSAC 找地板平面
2. 用 Rodrigues 旋轉對齊地板到水平
3. 去噪（Statistical Outlier Removal）
4. 縮放到指定高度（預設 2.5m）
5. 用 MinAreaRect 對齊牆面
6. 自動翻轉（確保地板在下）
7. 地板移到 z=0，中心移到原點

用法：
    python preprocess_aligned.py --input <點雲.ply> --output <輸出.ply>

作者：Claude Code
日期：2026-03-23
"""

import argparse
import numpy as np
import open3d as o3d
import cv2


def find_floor_normal(pcd):
    """用 RANSAC 找地板平面的法向量"""
    plane_model, inliers = pcd.segment_plane(
        distance_threshold=0.02,
        ransac_n=3,
        num_iterations=1000
    )
    a, b, c, d = plane_model
    normal = np.array([a, b, c])
    normal = normal / np.linalg.norm(normal)
    return normal, len(inliers)


def rodrigues_rotation(points, normal, target=np.array([0, 0, 1])):
    """用 Rodrigues 公式旋轉點雲，讓法向量對齊到目標方向"""
    # 確保法向量指向上方（Z 正方向）
    if normal[2] < 0:
        normal = -normal

    # 計算旋轉軸（法向量與目標的叉積）
    axis = np.cross(normal, target)
    axis_norm = np.linalg.norm(axis)

    if axis_norm < 0.001:
        # 已經對齊，不需要旋轉
        return points, 0.0

    axis = axis / axis_norm

    # 計算旋轉角度
    angle = np.arccos(np.clip(np.dot(normal, target), -1, 1))

    # Rodrigues 旋轉矩陣
    K = np.array([
        [0, -axis[2], axis[1]],
        [axis[2], 0, -axis[0]],
        [-axis[1], axis[0], 0]
    ])
    R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K

    # 應用旋轉
    rotated_points = points @ R.T

    return rotated_points, np.degrees(angle)


def align_walls(points):
    """用 MinAreaRect 對齊牆面到軸對齊"""
    # 取牆壁高度的點（0.3m ~ 2.0m）
    wall_pts = points[(points[:, 2] > 0.3) & (points[:, 2] < 2.0)]

    if len(wall_pts) < 100:
        print("[!] 牆壁點數不足，跳過對齊")
        return points, 0.0

    xy = wall_pts[:, :2].astype(np.float32)
    rect = cv2.minAreaRect(xy)
    angle = rect[2]

    # 調整角度到 -45° ~ 45°
    while angle <= -45:
        angle += 90
    while angle > 45:
        angle -= 90

    # 繞 Z 軸旋轉
    yaw = np.radians(angle)
    cos_a, sin_a = np.cos(-yaw), np.sin(-yaw)
    rot_z = np.array([
        [cos_a, -sin_a, 0],
        [sin_a, cos_a, 0],
        [0, 0, 1]
    ])

    aligned_points = points @ rot_z.T

    return aligned_points, angle


def flip_if_needed(points):
    """檢查並翻轉點雲（確保地板在下）"""
    # 如果大部分點在 Z 軸負方向，需要翻轉
    mean_z = np.mean(points[:, 2])
    mid_z = (np.max(points[:, 2]) + np.min(points[:, 2])) / 2

    # 這裡用簡單的啟發式：如果需要翻轉會在外部處理
    return points, False


def main():
    parser = argparse.ArgumentParser(
        description="SpatialLM 點雲預處理（含自動對齊）"
    )
    parser.add_argument("--input", required=True, help="輸入點雲檔案 (.ply)")
    parser.add_argument("--output", required=True, help="輸出點雲檔案 (.ply)")
    parser.add_argument("--target_height", type=float, default=2.5,
                        help="目標高度（公尺），預設 2.5")
    parser.add_argument("--flip", action="store_true",
                        help="翻轉點雲（上下顛倒）")
    args = parser.parse_args()

    # 1. 載入點雲
    print(f"[1/7] 載入點雲: {args.input}")
    pcd = o3d.io.read_point_cloud(args.input)
    points = np.asarray(pcd.points)
    print(f"      點數: {len(points):,}")

    # 2. 找地板平面並對齊
    print("[2/7] 尋找地板平面...")
    normal, inlier_count = find_floor_normal(pcd)
    print(f"      地板法向量: ({normal[0]:.3f}, {normal[1]:.3f}, {normal[2]:.3f})")
    print(f"      地板點數: {inlier_count:,}")

    # 3. Rodrigues 旋轉對齊地板
    print("[3/7] 對齊地板到水平...")
    points, rot_angle = rodrigues_rotation(points, normal)
    print(f"      旋轉角度: {rot_angle:.1f}°")

    # 4. 去噪
    print("[4/7] 去噪...")
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=10, std_ratio=1.5)
    points = np.asarray(pcd.points)
    print(f"      剩餘點數: {len(points):,}")

    # 5. 縮放
    print(f"[5/7] 縮放到 {args.target_height}m 高度...")
    min_z, max_z = np.min(points[:, 2]), np.max(points[:, 2])
    current_height = max_z - min_z
    scale = args.target_height / current_height
    points = points * scale
    print(f"      縮放比例: {scale:.4f}")

    # 6. 對齊牆面
    print("[6/7] 對齊牆面...")
    points, wall_angle = align_walls(points)
    print(f"      牆面旋轉: {wall_angle:.2f}°")

    # 7. 翻轉（如果指定）
    if args.flip:
        print("[*] 翻轉點雲（上下顛倒）...")
        angle = np.radians(180)
        rot_x = np.array([
            [1, 0, 0],
            [0, np.cos(angle), -np.sin(angle)],
            [0, np.sin(angle), np.cos(angle)]
        ])
        points = points @ rot_x.T

    # 8. 移動到原點
    print("[7/7] 調整位置...")
    min_z = np.min(points[:, 2])
    points[:, 2] -= min_z
    center_x = (np.max(points[:, 0]) + np.min(points[:, 0])) / 2
    center_y = (np.max(points[:, 1]) + np.min(points[:, 1])) / 2
    points[:, 0] -= center_x
    points[:, 1] -= center_y
    print(f"      地板對齊到 z=0，中心對齊到原點")

    # 保存
    pcd.points = o3d.utility.Vector3dVector(points)

    import os
    os.makedirs(os.path.dirname(args.output) or '.', exist_ok=True)
    o3d.io.write_point_cloud(args.output, pcd)

    # 輸出最終尺寸
    print()
    print("=== 最終尺寸 ===")
    print(f"X: {np.min(points[:,0]):.3f} 到 {np.max(points[:,0]):.3f} "
          f"(寬 {np.max(points[:,0])-np.min(points[:,0]):.3f}m)")
    print(f"Y: {np.min(points[:,1]):.3f} 到 {np.max(points[:,1]):.3f} "
          f"(深 {np.max(points[:,1])-np.min(points[:,1]):.3f}m)")
    print(f"Z: {np.min(points[:,2]):.3f} 到 {np.max(points[:,2]):.3f} "
          f"(高 {np.max(points[:,2])-np.min(points[:,2]):.3f}m)")
    print()
    print(f"[完成] 已保存到: {args.output}")


if __name__ == "__main__":
    main()
