#!/usr/bin/env python3
"""
PCD 点云栅格化投影为 PNG 图像（增强对比度）
用法示例：
    python3 pcd_to_raster_enhanced.py --input test.pcd --output map.png --cell_size 0.1 --contrast_percentile 2
可选增强选项：
    --contrast_percentile P   : 裁剪两侧百分位（例如 2 表示使用 2%~98% 的数据进行线性拉伸）
    --equalize                : 直方图均衡化（会覆盖百分比裁剪效果）
    --gamma G                  : 伽马校正（例如 0.5 变亮，2.0 变暗）
"""

import argparse
import numpy as np
from PIL import Image, ImageOps
import open3d as o3d
from collections import defaultdict
import logging

# 配置日志记录
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('pcd_to_raster.log', encoding='utf-8'),
        logging.StreamHandler()
    ]
)

def fast_downsample(pcd, voxel_size=0.25, filter_params={'nb_neighbors':20, 'std_ratio':2.0}):
    print(f"下采样前点数: {len(pcd.points)}")
    # 体素下采样
    down_pcd = pcd.voxel_down_sample(voxel_size)
    # 滤波（可选）
    #if filter_params:
    #    nb_neighbors = filter_params.get('nb_neighbors', 20)
    #    std_ratio = filter_params.get('std_ratio', 2.0)
    #    down_pcd, _ = down_pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    #    print(f"滤波后点数: {len(pcd.points)}")
    print(f"下采样后点数: {len(down_pcd.points)}")

    return down_pcd


def pcd_to_raster(pcd, output_img, cell_size=0.05,
                  x_range=None, y_range=None,
                  radar_point=(0,0,0), use_color=False,
                  contrast_percentile=None, equalize=False, gamma=None):
    """
    将 PCD 点云栅格化并输出图像（支持对比度增强）
    """
    # 1. 读取点云
    points = np.asarray(pcd.points)
    if points.shape[0] == 0:
        raise ValueError("点云为空")

    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]

    # 2. 确定投影范围
    if x_range is None:
        x_min, x_max = x.min(), x.max()
        margin_x = (x_max - x_min) * 0.01
        x_min -= margin_x
        x_max += margin_x
    else:
        x_min, x_max = x_range
    if y_range is None:
        y_min, y_max = y.min(), y.max()
        margin_y = (y_max - y_min) * 0.01
        y_min -= margin_y
        y_max += margin_y
    else:
        y_min, y_max = y_range

    print(f"投影范围：X [{x_min:.3f}, {x_max:.3f}], Y [{y_min:.3f}, {y_max:.3f}]")

    n_cols = int(np.ceil((x_max - x_min) / cell_size))
    n_rows = int(np.ceil((y_max - y_min) / cell_size))
    print(f"栅格大小：{n_cols} 列 x {n_rows} 行")

    # 4. 计算栅格索引
    col_indices = np.floor((x - x_min) / cell_size).astype(int)
    row_indices = np.floor((y - y_min) / cell_size).astype(int)

    valid_mask = (col_indices >= 0) & (col_indices < n_cols) & \
                 (row_indices >= 0) & (row_indices < n_rows)
    col_indices = col_indices[valid_mask]
    row_indices = row_indices[valid_mask]
    z_valid = z[valid_mask]
    print(f"有效点数：{len(z_valid)} / {points.shape[0]}")

    if len(z_valid) == 0:
        raise ValueError("没有点在投影范围内")

    # 5. 聚合计算平均高度
    flat_indices = row_indices * n_cols + col_indices
    z_sum_flat = np.bincount(flat_indices, weights=z_valid, minlength=n_rows*n_cols)
    count_flat = np.bincount(flat_indices, minlength=n_rows*n_cols)

    z_sum_bottom = z_sum_flat.reshape(n_rows, n_cols)
    count_bottom = count_flat.reshape(n_rows, n_cols)

    # 翻转行，使数组索引 0 对应图像顶部
    z_sum = np.flipud(z_sum_bottom)
    count = np.flipud(count_bottom)

    avg_z = np.zeros_like(z_sum, dtype=float)
    mask = count > 0
    avg_z[mask] = z_sum[mask] / count[mask]

    # 6. 灰度映射（基础线性映射，后续可增强）
    # 仅对有点的栅格进行归一化
    z_vals = avg_z[mask]
    if len(z_vals) == 0:
        raise ValueError("没有有效栅格")

    # 基础线性映射：使用全局最小最大值
    z_min_global = z_vals.min()
    z_max_global = z_vals.max()

    # 如果启用百分比裁剪，计算裁剪后的范围
    if contrast_percentile is not None and not equalize:
        p_low = contrast_percentile
        p_high = 100 - contrast_percentile
        z_low = np.percentile(z_vals, p_low)
        z_high = np.percentile(z_vals, p_high)
        print(f"百分比裁剪：使用 [{p_low}%, {p_high}%] 范围 [{z_low:.3f}, {z_high:.3f}]")
        # 线性拉伸，并截断到 [0,255]
        gray_float = np.zeros_like(avg_z)
        gray_float[mask] = (255 * (avg_z[mask] - z_low) / (z_high - z_low))
        gray_float = np.clip(gray_float, 0, 255)
        gray = gray_float.astype(np.uint8)
    else:
        # 使用全局最小最大
        if z_max_global - z_min_global < 1e-6:
            gray = np.full_like(avg_z, 128, dtype=np.uint8)
        else:
            gray_float = np.zeros_like(avg_z)
            gray_float[mask] = (255 * (avg_z[mask] - z_min_global) / (z_max_global - z_min_global))
            gray = gray_float.astype(np.uint8)

    # 7. 直方图均衡化（如果需要）
    if equalize:
        # 注意：PIL 的 equalize 要求输入是 'L' 模式的图像
        img_gray = Image.fromarray(gray, 'L')
        img_eq = ImageOps.equalize(img_gray)
        gray = np.array(img_eq)
        print("已应用直方图均衡化")

    # 8. 伽马校正（如果需要）
    if gamma is not None:
        # 对灰度图像进行伽马校正：输出 = 255 * (输入/255)^(1/gamma)
        # 注意 gamma<1 使图像变亮，gamma>1 变暗
        gray_float = gray.astype(np.float32) / 255.0
        gray_float = np.power(gray_float, 1.0 / gamma)
        gray = (gray_float * 255).astype(np.uint8)
        print(f"已应用伽马校正，gamma={gamma}")

    # 9. 生成彩色图像（如果启用）
    if use_color:
        try:
            import matplotlib.pyplot as plt
            cmap = plt.get_cmap('jet')
            colored = cmap(gray / 255.0)
            colored = (colored[:, :, :3] * 255).astype(np.uint8)
            img = Image.fromarray(colored, 'RGB')
        except ImportError:
            print("警告：未安装 matplotlib，将输出灰度图像。")
            img = Image.fromarray(gray, 'L')
    else:
        img = Image.fromarray(gray, 'L')

    img.save(output_img)
    print(f"图像已保存至：{output_img}")

    # 10. 计算雷达点图像坐标
    radar_x, radar_y, radar_z = radar_point
    col_radar = (radar_x - x_min) / cell_size
    row_radar = (radar_y - y_min) / cell_size
    grid_x = col_radar * cell_size
    grid_y = row_radar * cell_size
   
    log_message = f"雷达点 ({radar_x}, {radar_y}, {radar_z}) 的图像坐标：\n"
    log_message += f"  列 (col) = {col_radar:.2f}  (从左到右)\n"
    log_message += f"  行 (row) = {row_radar:.2f}  (从下到上)\n"
    log_message += f"  grid_x = {grid_x:.2f}  (从左到右)\n"
    log_message += f"  grid_y = {grid_y:.2f}  (从下到上)\n"
    
    # 写入日志文件
    logging.info(log_message)

    if 0 <= col_radar < n_cols and 0 <= row_radar < n_rows:
        print("  该点位于图像内部。")
    else:
        print("  该点位于图像外部。")

    return col_radar, row_radar


def projection_outlier_removal(pcd, plane='xy', grid_size=0.1, min_points_per_cell=3):
    """
    使用投影滤波提取“密集”区域的核心点云
    注意：此函数返回的是“保留下来的点”，而不是“被移除的噪点”。
    """
    points = np.asarray(pcd.points)
    if len(points) == 0:
        return o3d.geometry.PointCloud()

    # 选择投影维度
    if plane == 'xy':
        proj = points[:, :2]
    elif plane == 'xz':
        proj = points[:, [0, 2]]
    elif plane == 'yz':
        proj = points[:, 1:]
    else:
        raise ValueError("plane must be 'xy', 'xz', or 'yz'")

    # 计算网格索引
    # 防止除以零或网格过小导致索引溢出
    if grid_size <= 0: grid_size = 0.01
    indices = np.floor(proj / grid_size).astype(int)

    # 构建字典统计每个网格中的点索引
    cell_dict = defaultdict(list)
    for i, idx in enumerate(indices):
        key = tuple(idx)
        cell_dict[key].append(i)

    # 收集保留的点索引
    keep_indices = []
    for idx_list in cell_dict.values():
        if len(idx_list) >= min_points_per_cell:
            keep_indices.extend(idx_list)

    # 关键保护：如果没有点满足条件，返回空对象而不是崩溃
    if not keep_indices:
        return o3d.geometry.PointCloud()

    # 构造新点云
    filtered_pcd = o3d.geometry.PointCloud()
    filtered_pcd.points = o3d.utility.Vector3dVector(points[keep_indices])

    # 属性继承（颜色、法线）
    if pcd.has_colors():
        filtered_pcd.colors = o3d.utility.Vector3dVector(np.asarray(pcd.colors)[keep_indices])
    if pcd.has_normals():
        filtered_pcd.normals = o3d.utility.Vector3dVector(np.asarray(pcd.normals)[keep_indices])

    return filtered_pcd

def auto_crop_pointcloud_area(filtered_xz, filtered_xy, filtered_yz, pcd):
    """
    根据三个投影面的有效范围，计算包围盒并裁剪原图
    """
    def safe_get_bounds(filtered_pcd):
        pts = np.asarray(filtered_pcd.points)
        if len(pts) == 0:
            return None
        return pts.min(axis=0), pts.max(axis=0)

    bounds_list = [
        safe_get_bounds(filtered_xz),
        safe_get_bounds(filtered_xy),
        safe_get_bounds(filtered_yz)
    ]

    # 过滤掉空的投影结果
    valid_bounds = [b for b in bounds_list if b is not None]

    if not valid_bounds:
        # 如果所有投影都是空的，说明全是噪点，返回空
        return o3d.geometry.PointCloud()

    # 合并所有有效投影的边界，取交集还是并集？
    # 这里取“最紧”的包围盒：
    # X范围取所有投影X的最小/最大值
    # Y范围取所有投影Y的最小/最大值
    # Z范围取所有投影Z的最小/最大值

    all_mins = np.array([b[0] for b in valid_bounds])
    all_maxs = np.array([b[1] for b in valid_bounds])

    # 使用最小值的最大值（最右/最前/最上）和最大值的最小值（最左/最后/最下）来取交集
    # 这样能保证裁剪框在所有投影的有效范围内
    x_min = np.max(all_mins[:, 0])
    x_max = np.min(all_maxs[:, 0])

    y_min = np.max(all_mins[:, 1])
    y_max = np.min(all_maxs[:, 1])

    z_min = np.max(all_mins[:, 2])
    z_max = np.min(all_maxs[:, 2])

    # 检查包围盒是否合法（防止出现 min > max 的情况）
    if x_min >= x_max or y_min >= y_max or z_min >= z_max:
        # 如果交集为空，退化为使用原始点云或者返回空，这里选择返回空
        return o3d.geometry.PointCloud()

    # 生成掩码
    points = np.asarray(pcd.points)
    mask = ((points[:, 0] >= x_min) & (points[:, 0] <= x_max) &
            (points[:, 1] >= y_min) & (points[:, 1] <= y_max) &
            (points[:, 2] >= z_min) & (points[:, 2] <= z_max))

    indices = np.where(mask)[0]
    if len(indices) == 0:
        return o3d.geometry.PointCloud()

    return pcd.select_by_index(indices)

def auto_crop_pointcloud(pcd, x_ratio=0.05, y_ratio=0.05, z_ratio=0.1):
    """
    按比例向内收缩裁剪
    """
    points = np.asarray(pcd.points)
    if len(points) == 0:
        return pcd

    x_min, x_max = np.min(points[:, 0]), np.max(points[:, 0])
    y_min, y_max = np.min(points[:, 1]), np.max(points[:, 1])
    z_min, z_max = np.min(points[:, 2]), np.max(points[:, 2])

    # 计算边长
    x_span = x_max - x_min
    y_span = y_max - y_min
    z_span = z_max - z_min

    # 防止除以零（如果点云是一个平面或线）
    if x_span < 1e-6: x_span = 0
    if y_span < 1e-6: y_span = 0
    if z_span < 1e-6: z_span = 0

    # 向内收缩边界
    # 左边界向右移，右边界向左移
    crop_x_min = x_min + x_ratio * x_span
    crop_x_max = x_max - x_ratio * x_span

    crop_y_min = y_min + y_ratio * y_span
    crop_y_max = y_max - y_ratio * y_span

    # Z轴：只切顶部，保留底部
    crop_z_min = z_min
    crop_z_max = z_max - z_ratio * z_span

    # 确保边界合法
    if crop_x_min >= crop_x_max or crop_y_min >= crop_y_max or crop_z_min >= crop_z_max:
        return pcd # 裁剪过度，返回原数据或空

    mask = ((points[:, 0] >= crop_x_min) & (points[:, 0] <= crop_x_max) &
            (points[:, 1] >= crop_y_min) & (points[:, 1] <= crop_y_max) &
            (points[:, 2] >= crop_z_min) & (points[:, 2] <= crop_z_max))

    return pcd.select_by_index(np.where(mask)[0])

def preprocess(pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
    # 1. 投影滤波：提取各个平面的“密集骨架”
    # 注意：这里的 min_points_per_cell 非常关键，设得太高会导致 filtered 为空
    filtered_xz = projection_outlier_removal(pcd, plane='xz', grid_size=0.1, min_points_per_cell=10)
    filtered_xy = projection_outlier_removal(pcd, plane='xy', grid_size=0.1, min_points_per_cell=10)
    filtered_yz = projection_outlier_removal(pcd, plane='yz', grid_size=0.1, min_points_per_cell=10)

    # 2. 根据骨架计算包围盒，裁剪原图
    # 这一步去除了外围的大片噪点
    cropped = auto_crop_pointcloud_area(filtered_xz, filtered_xy, filtered_yz, pcd)

    # 3. 最后微调：去除边界残留的噪点或顶部天空
    # x_ratio=0.02 表示左右各切掉 2%
    # z_ratio=0.35 表示切掉顶部 35% (去天)
    #final = auto_crop_pointcloud(cropped, x_ratio=0.02, y_ratio=0.02, z_ratio=0.35)

    return cropped

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="PCD 点云栅格化投影为 PNG 图像（增强对比度）")
    parser.add_argument("--input", "-i", required=True, help="输入 PCD 文件路径")
    parser.add_argument("--output", "-o", required=True, help="输出 PNG 图像路径")
    parser.add_argument("--cell_size", "-s", type=float, required=True,
                        help="栅格尺寸（每个像素代表的实际距离）")
    parser.add_argument("--voxel_size", "-v", type=float, default=0.25,                                                 
                        help="PCD下采样体素大小")
    parser.add_argument("--x_min", type=float, help="投影 X 最小值（可选）")
    parser.add_argument("--x_max", type=float, help="投影 X 最大值（可选）")
    parser.add_argument("--y_min", type=float, help="投影 Y 最小值（可选）")
    parser.add_argument("--y_max", type=float, help="投影 Y 最大值（可选）")
    parser.add_argument("--radar_x", type=float, default=0.0, help="雷达点 X 坐标（默认 0）")
    parser.add_argument("--radar_y", type=float, default=0.0, help="雷达点 Y 坐标（默认 0）")
    parser.add_argument("--radar_z", type=float, default=0.0, help="雷达点 Z 坐标（默认 0）")
    parser.add_argument("--color", action="store_true", help="输出彩色图像（需 matplotlib）")
    # 对比度增强选项
    parser.add_argument("--contrast_percentile", type=float, default=None,
                        help="裁剪两侧的百分位（例如 2 表示使用 2%~98% 的数据）")
    parser.add_argument("--equalize", action="store_true", help="直方图均衡化（增强全局对比度）")
    parser.add_argument("--gamma", type=float, default=None, help="伽马校正值（<1 变亮，>1 变暗）")

    args = parser.parse_args()

    x_range = (args.x_min, args.x_max) if args.x_min is not None and args.x_max is not None else None
    y_range = (args.y_min, args.y_max) if args.y_min is not None and args.y_max is not None else None

    radar_pt = (args.radar_x, args.radar_y, args.radar_z)
    print(f"读取点云：{args.input}")
    pcd = o3d.io.read_point_cloud(args.input)
    downpcd = fast_downsample(pcd, args.voxel_size, filter_params={'nb_neighbors':20, 'std_ratio':2.0})
    cropped = preprocess(downpcd)
    crop_x_y = auto_crop_pointcloud(cropped, x_ratio=0.02, y_ratio=0.02, z_ratio=0)
    o3d.io.write_point_cloud('fast_downsample.pcd', crop_x_y)
    finnal = auto_crop_pointcloud(cropped, x_ratio=0.0, y_ratio=0.0, z_ratio=0.35)	
    pcd_to_raster(finnal, args.output, args.cell_size,
                  x_range, y_range, radar_pt,
                  use_color=args.color,
                  contrast_percentile=args.contrast_percentile,
                  equalize=args.equalize,
                  gamma=args.gamma)
