#!/usr/bin/env python3
"""
点云体积测算 - Python 实现

统一使用 2.5D 规则网格像元法计算体积。

流程:
  3D 散乱点云 ──栅格化──→ 2.5D 规则网格 ──像元法──→ 体积
  2.5D 规则网格 ──────────────→ 像元法 ────────────→ 体积

像元法公式: V = Σ(像元面积 × (Z - 地面高))

优势:
  - 物理意义明确，无经验系数 (0.6, ×2 等)
  - 对 grid 数据精确匹配预期值
  - 对 3D 云数据通过栅格化消除凸包过估计

用法:
  python pcd_volume.py [res_dir] [out_dir]
"""

import numpy as np
import open3d as o3d
import os
import time
import csv
import sys
from scipy.interpolate import griddata


# =====================================================================
# 坐标单位检测与缩放
# =====================================================================

def detect_scale(points):
    """
    检测坐标单位并返回 → 米的缩放因子。

    - 最大跨度 > 10000 → mm (×0.001)
    - 最大跨度 > 1000  → cm (×0.01)
    - 否则 → m (×1.0)
    """
    span_x = float(points[:, 0].max() - points[:, 0].min())
    span_y = float(points[:, 1].max() - points[:, 1].min())
    max_span = max(span_x, span_y)

    if max_span > 10000:
        return 0.001
    elif max_span > 1000:
        return 0.01
    else:
        return 1.0


# =====================================================================
# 数据类型检测
# =====================================================================

def detect_data_type(points):
    """
    检测点云是规则网格还是散乱点云。

    判断: X 坐标间距的变异系数 CV < 5% → 规则网格
    """
    if len(points) < 100:
        return "cloud"

    sample = points[np.random.choice(len(points), min(20000, len(points)), replace=False)]
    x_vals = np.sort(np.unique(sample[:, 0]))

    if len(x_vals) < 5:
        return "cloud"

    diffs = np.diff(x_vals)
    mean_d = float(np.mean(diffs))
    if mean_d < 1e-10:
        return "cloud"

    cv = float(np.std(diffs) / mean_d)
    return "grid" if cv < 0.05 else "cloud"


# =====================================================================
# 地面检测
# =====================================================================

def detect_ground(points, percentile=0.5):
    """
    取 Z 轴低百分位均值作为地面高度。
    默认 0.5% 分位，对离群点不敏感。
    """
    if len(points) == 0:
        return 0.0
    threshold = np.percentile(points[:, 2], percentile)
    candidates = points[points[:, 2] <= threshold]
    return float(np.mean(candidates[:, 2])) if len(candidates) > 0 else 0.0


# =====================================================================
# 栅格化 — 3D 散乱点云 → 2.5D 规则网格 (支持插值)
# =====================================================================

def rasterize_to_grid(points, resolution=None, interpolate_empty_cells=False):
    """
    将 3D 散乱点云栅格化为 2.5D 规则网格。

    每个 XY 网格单元取最大 Z (堆体上表面)，确保每个 XY 只有唯一 Z。

    参数:
        points:     Nx3 点云s
        resolution: 网格分辨率 (m)。None 则自动估算。
        interpolate_empty_cells: 是否对空单元格进行插值

    返回:
        grid_pts:  Mx3 网格点云 (M ≤ N，每个 XY 唯一)
        resolution: 实际使用的分辨率
    """
    if len(points) == 0:
        return np.empty((0, 3)), 0.0

    # 自动估算: 基于点云覆盖面积和点数的平均间距
    if resolution is None:
        span_x = float(points[:, 0].max() - points[:, 0].min())
        span_y = float(points[:, 1].max() - points[:, 1].min())
        area = max(span_x * span_y, 1.0)
        avg_spacing = np.sqrt(area / len(points))
        resolution = float(np.clip(avg_spacing * 1.0, 0.05, 1.0))

    # 网格范围
    min_x, min_y = float(points[:, 0].min()), float(points[:, 1].min())
    max_x, max_y = float(points[:, 0].max()), float(points[:, 1].max())

    nx = max(1, int(np.ceil((max_x - min_x) / resolution)))
    ny = max(1, int(np.ceil((max_y - min_y) / resolution)))

    # 每个点的网格索引
    ix = np.clip(((points[:, 0] - min_x) / resolution).astype(np.int64), 0, nx - 1)
    iy = np.clip(((points[:, 1] - min_y) / resolution).astype(np.int64), 0, ny - 1)

    # 网格单元 key: ix * ny + iy
    cell_key = ix * ny + iy

    # 按 (cell_key, Z) 排序 → 每个单元最后一个是最大 Z
    order = np.lexsort((points[:, 2], cell_key))
    sorted_keys = cell_key[order]
    sorted_pts = points[order]

    # 每个唯一单元取最大 Z 点
    unique_keys, start_pos = np.unique(sorted_keys, return_index=True)
    end_pos = np.append(start_pos[1:], len(sorted_pts))

    # 构建网格点云
    grid_x = min_x + (unique_keys // ny + 0.5) * resolution
    grid_y = min_y + (unique_keys % ny + 0.5) * resolution

    # 每个单元的 max Z (排序后最后一个)
    grid_z = np.array([sorted_pts[e - 1, 2] for e in end_pos])

    grid_pts = np.column_stack([grid_x, grid_y, grid_z])
    
    # 如果需要插值空单元格
    if interpolate_empty_cells and len(grid_pts) < nx * ny:
        print(f"  开始插值空单元格，原始网格点数: {len(grid_pts)}, 总网格数: {nx * ny}")
        grid_pts = interpolate_empty_cells_in_grid(grid_pts, min_x, min_y, nx, ny, resolution)
        print(f"  插值后网格点数: {len(grid_pts)}")
    
    return grid_pts, resolution


def interpolate_empty_cells_in_grid(grid_pts, min_x, min_y, nx, ny, resolution):
    """
    对规则网格中的空单元格进行插值填充
    """
    # 创建完整的网格坐标
    full_grid_x, full_grid_y = np.meshgrid(
        np.arange(nx) * resolution + min_x + resolution/2,
        np.arange(ny) * resolution + min_y + resolution/2,
        indexing='ij'
    )
    
    # 将现有网格点转换为坐标映射
    existing_coords = grid_pts[:, :2]
    existing_z = grid_pts[:, 2]
    
    # 将完整网格展平以准备插值
    flat_x = full_grid_x.flatten()
    flat_y = full_grid_y.flatten()
    grid_coords = np.column_stack([flat_x, flat_y])
    
    # 使用网格数据插值方法填充空单元格
    interpolated_z = griddata(
        existing_coords, 
        existing_z, 
        grid_coords, 
        method='linear',
        fill_value=np.nan
    )
    
    # 过滤掉插值失败的点（保留有效插值结果）
    valid_mask = ~np.isnan(interpolated_z)
    valid_coords = grid_coords[valid_mask]
    valid_z = interpolated_z[valid_mask]
    
    # 组合最终结果
    result = np.column_stack([valid_coords, valid_z])
    
    # 如果仍有缺失值，使用最近邻方法填充
    if not np.all(valid_mask):
        nan_mask = np.isnan(interpolated_z)
        if np.any(nan_mask):
            # 使用最近邻方法填充剩余的NaN值
            interpolated_z_nan = griddata(
                existing_coords, 
                existing_z, 
                grid_coords[nan_mask], 
                method='nearest'
            )
            
            # 添加最近邻填充的结果
            nan_coords = grid_coords[nan_mask]
            result = np.vstack([
                result,
                np.column_stack([nan_coords, interpolated_z_nan])
            ])
    
    return result


# =====================================================================
# 体积计算 — 2.5D 规则网格像元法
# =====================================================================

def volume_grid(points, ground_z, resolution=None):
    """
    2.5D 规则网格像元法计算体积。

    V = 像元面积 × Σ(Z - 地面高)

    参数:
        points:     Nx3 网格点云 (每个 XY 唯一)
        ground_z:   地面高度
        resolution: 网格分辨率。None 则从点云间距估算。

    返回:
        volume: 体积 (m³)
    """
    
    print(f"DEBUG - ground_z: {ground_z}")
    print(f"DEBUG - resolution: {resolution}")
    print(f"DEBUG - points[:,2] min: {np.min(points[:, 2]):.4f}, max: {np.max(points[:, 2]):.4f}, mean: {np.mean(points[:, 2]):.4f}")
    
    if len(points) == 0:
        return 0.0

    # 估算网格分辨率
    if resolution is None:
        x_vals = np.sort(np.unique(points[:, 0]))
        if len(x_vals) < 2:
            return 0.0
        resolution = float(np.median(np.diff(x_vals)))

    cell_area = resolution * resolution

    # 滤除地面点
    above = points[points[:, 2] >= ground_z]
    if len(above) == 0:
        return 0.0

    heights = above[:, 2] - ground_z
    print(f"DEBUG - cell_area: {cell_area}，total: {np.sum(heights)}")
    return float(np.sum(heights) * cell_area)


# =====================================================================
# 点云 I/O 与滤波
# =====================================================================

def read_pcd(filepath):
    """读取 PCD 文件为 Nx3 数组"""
    pcd = o3d.io.read_point_cloud(filepath)
    return np.asarray(pcd.points)


def voxel_filter(points, leaf_size=0.3):
    """体素滤波降采样"""
    if len(points) == 0:
        return points
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd = pcd.voxel_down_sample(leaf_size)
    return np.asarray(pcd.points)


def save_pcd_binary(filepath, points):
    """保存点云为二进制 PCD"""
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    o3d.io.write_point_cloud(filepath, pcd, write_ascii=False)


# =====================================================================
# 单文件处理
# =====================================================================

def process_file(filepath, out_dir, **kwargs):
    """处理单个 PCD 文件"""
    filename = os.path.basename(filepath)
    print(f"\n{'='*60}")
    print(f"处理: {filename}")
    print(f"{'='*60}")
    start = time.time()

    # ---- 读取 ----
    points = read_pcd(filepath)
    if len(points) == 0:
        print(f"  [警告] 点云为空")
        return None
    print(f"  原始点数: {len(points)}")

    # ---- 坐标单位检测与缩放 ----
    scale = detect_scale(points)
    if scale != 1.0:
        unit_name = {0.001: "mm", 0.01: "cm"}.get(scale, "?")
        print(f"  坐标单位: {unit_name} (×{scale} → m)")
        points = apply_scale(points, scale)

    # ---- 数据类型检测 ----
    data_type = detect_data_type(points)
    print(f"  数据类型: {'2.5D 规则网格' if data_type == 'grid' else '3D 散乱点云'}")

    # ---- 转为 2.5D 规则网格 ----
    grid_res = kwargs.get("grid_resolution", None)
    interpolate_empty = kwargs.get("interpolate_empty_cells", True)  # 默认启用插值

    if data_type == "cloud":
        # 3D 点云: 体素滤波 → 栅格化
        voxel_size = kwargs.get("voxel_size", 0.3)
        points_v = voxel_filter(points, voxel_size)
        # points_v = points
        print(f"  降采样后: {len(points_v)}")

        if len(points_v) < 10:
            print(f"  [警告] 点数不足")
            return None

        points_g, grid_res = rasterize_to_grid(points_v, grid_res, interpolate_empty)
        print(f"  栅格化后: {len(points_g)} 个网格单元, 分辨率 {grid_res:.3f} m")
        
        # 保存栅格化后的点云到输出目录
        base_filename = os.path.splitext(filename)[0]
        grid_pcd_path = os.path.join(out_dir, f"{base_filename}_grid.pcd")
        save_pcd_binary(grid_pcd_path, points_g)
        print(f"  栅格化点云已保存至: {grid_pcd_path}")
    else:
        # 已是网格: 可以直接使用
        points_g = points
        if grid_res is None:
            x_vals = np.sort(np.unique(points_g[:, 0]))
            grid_res = float(np.median(np.diff(x_vals))) if len(x_vals) > 1 else 1.0
        print(f"  网格分辨率: {grid_res:.3f} m")
        
        # 对于已经是网格的数据也保存一份
        base_filename = os.path.splitext(filename)[0]
        grid_pcd_path = os.path.join(out_dir, f"{base_filename}_grid.pcd")
        save_pcd_binary(grid_pcd_path, points_g)
        print(f"  栅格化点云已保存至: {grid_pcd_path}")

    # ---- 地面检测 ----
    ground_z = detect_ground(points_g, 0.5)
    print(f"  地面高度: {ground_z:.4f} m")

    # ---- 体积计算 (统一像元法) ----
    volume = volume_grid(points_g, ground_z, grid_res)
    print(f"  体积:     {volume:,.4f} m³")

    elapsed = time.time() - start
    print(f"  耗时:     {elapsed:.2f} s")

    # ---- 输出 CSV ----
    os.makedirs(out_dir, exist_ok=True)
    result_file = os.path.join(out_dir, f"{filename}Result.csv")
    with open(result_file, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(["名称", "数据类型", "网格分辨率(m)",
                         "地面高度(m)", "体积(m³)", "耗时(s)"])
        writer.writerow([filename, data_type, f"{grid_res:.3f}",
                         f"{ground_z:.4f}", f"{volume:.4f}", f"{elapsed:.2f}"])
    print(f"  结果:     {result_file}")

    return volume


# =====================================================================
# 工具函数
# =====================================================================

def apply_scale(points, scale):
    return points * scale


# =====================================================================
# 入口
# =====================================================================

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    res_dir = os.path.join(script_dir,  "res")
    out_dir = os.path.join(script_dir,  "out")

    if len(sys.argv) >= 2:
        res_dir = sys.argv[1]
    if len(sys.argv) >= 3:
        out_dir = sys.argv[2]

    if not os.path.isdir(res_dir):
        print(f"[错误] 目录不存在: {res_dir}")
        sys.exit(1)

    files = sorted(f for f in os.listdir(res_dir) if f.lower().endswith(".pcd"))
    if not files:
        print(f"[错误] 在 '{res_dir}' 中未找到 .pcd 文件")
        sys.exit(1)

    print(f"找到 {len(files)} 个点云文件\n")
    for fname in files:
        filepath = os.path.join(res_dir, fname)
        process_file(filepath, out_dir, interpolate_empty_cells=True)

    print(f"\n{'='*60}")
    print("处理完成！")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()