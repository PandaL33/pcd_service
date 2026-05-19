import argparse
import json
import logging
import sys
from collections import defaultdict
from pathlib import Path
from typing import List, Tuple

import numpy as np
import open3d as o3d
from matplotlib.path import Path as MplPath

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


class PointCloudPreprocessor:
    def projection_outlier_removal(self, pcd, plane='xy', grid_size=0.1, min_points_per_cell=3):
        points = np.asarray(pcd.points)
        if len(points) == 0:
            return o3d.geometry.PointCloud()

        if plane == 'xy':
            proj = points[:, :2]
        elif plane == 'xz':
            proj = points[:, [0, 2]]
        elif plane == 'yz':
            proj = points[:, 1:]
        else:
            raise ValueError("plane must be 'xy', 'xz', or 'yz'")

        if grid_size <= 0: grid_size = 0.01
        indices = np.floor(proj / grid_size).astype(int)

        cell_dict = defaultdict(list)
        for i, idx in enumerate(indices):
            key = tuple(idx)
            cell_dict[key].append(i)

        keep_indices = []
        for idx_list in cell_dict.values():
            if len(idx_list) >= min_points_per_cell:
                keep_indices.extend(idx_list)

        if not keep_indices:
            return o3d.geometry.PointCloud()

        filtered_pcd = o3d.geometry.PointCloud()
        filtered_pcd.points = o3d.utility.Vector3dVector(points[keep_indices])

        if pcd.has_colors():
            filtered_pcd.colors = o3d.utility.Vector3dVector(np.asarray(pcd.colors)[keep_indices])
        if pcd.has_normals():
            filtered_pcd.normals = o3d.utility.Vector3dVector(np.asarray(pcd.normals)[keep_indices])

        return filtered_pcd

    def auto_crop_pointcloud_area(self, filtered_xz, filtered_xy, filtered_yz, pcd):
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

        valid_bounds = [b for b in bounds_list if b is not None]

        if not valid_bounds:
            return o3d.geometry.PointCloud()

        all_mins = np.array([b[0] for b in valid_bounds])
        all_maxs = np.array([b[1] for b in valid_bounds])

        x_min = np.max(all_mins[:, 0])
        x_max = np.min(all_maxs[:, 0])

        y_min = np.max(all_mins[:, 1])
        y_max = np.min(all_maxs[:, 1])

        z_min = np.max(all_mins[:, 2])
        z_max = np.min(all_maxs[:, 2])

        if x_min >= x_max or y_min >= y_max or z_min >= z_max:
             return o3d.geometry.PointCloud()

        points = np.asarray(pcd.points)
        mask = ((points[:, 0] >= x_min) & (points[:, 0] <= x_max) &
                (points[:, 1] >= y_min) & (points[:, 1] <= y_max) &
                (points[:, 2] >= z_min) & (points[:, 2] <= z_max))

        indices = np.where(mask)[0]
        if len(indices) == 0:
            return o3d.geometry.PointCloud()

        return pcd.select_by_index(indices)

    def auto_crop_pointcloud(self, pcd, x_ratio=0.05, y_ratio=0.05, z_ratio=0.1):
        points = np.asarray(pcd.points)
        if len(points) == 0:
            return pcd

        x_min, x_max = np.min(points[:, 0]), np.max(points[:, 0])
        y_min, y_max = np.min(points[:, 1]), np.max(points[:, 1])
        z_min, z_max = np.min(points[:, 2]), np.max(points[:, 2])

        x_span = x_max - x_min
        y_span = y_max - y_min
        z_span = z_max - z_min

        if x_span < 1e-6: x_span = 0
        if y_span < 1e-6: y_span = 0
        if z_span < 1e-6: z_span = 0

        crop_x_min = x_min + x_ratio * x_span
        crop_x_max = x_max - x_ratio * x_span

        crop_y_min = y_min + y_ratio * y_span
        crop_y_max = y_max - y_ratio * y_span

        crop_z_min = z_min
        crop_z_max = z_max - z_ratio * z_span

        if crop_x_min >= crop_x_max or crop_y_min >= crop_y_max or crop_z_min >= crop_z_max:
             return pcd

        mask = ((points[:, 0] >= crop_x_min) & (points[:, 0] <= crop_x_max) &
                (points[:, 1] >= crop_y_min) & (points[:, 1] <= crop_y_max) &
                (points[:, 2] >= crop_z_min) & (points[:, 2] <= crop_z_max))

        return pcd.select_by_index(np.where(mask)[0])

    def custom_round_down(self, val):
        val = val - 0.5
        decimal_part = round(val % 1, 2)
        if decimal_part >= 0.5:
            base_int = round(val) - 1.0
        else:
            base_int = round(val) - 0.5

        return base_int

    def preprocess(self, pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        filtered_xz = self.projection_outlier_removal(pcd, plane='xz', grid_size=0.1, min_points_per_cell=10)
        filtered_xy = self.projection_outlier_removal(pcd, plane='xy', grid_size=0.1, min_points_per_cell=10)
        filtered_yz = self.projection_outlier_removal(pcd, plane='yz', grid_size=0.1, min_points_per_cell=10)

        cropped = self.auto_crop_pointcloud_area(filtered_xz, filtered_xy, filtered_yz, pcd)

        points = np.asarray(cropped.points)
        if len(points) == 0: return cropped

        plane_model, inliers = cropped.segment_plane(distance_threshold=0.1,
                                                     ransac_n=3,
                                                     num_iterations=1000)

        a, b, c, d = plane_model

        is_horizontal = abs(c) > 0.9
        plane_z = -d / c
        z_max = np.max(points[:, 2])
        z_min = np.min(points[:, 2])

        is_ceiling = plane_z > (z_min + (z_max - z_min) * 0.5)

        if is_horizontal and is_ceiling:
            z_cutoff = self.custom_round_down(plane_z)
            mask = (points[:, 2] <= z_cutoff)

            logger.info(f"检测到天花板高度: {plane_z:.5f}, 裁剪上限设为: {z_cutoff:.5f}")
            return cropped.select_by_index(np.where(mask)[0])
        else:
            logger.info("未检测到明显天花板,采用裁剪到顶部固定40%")
            final = self.auto_crop_pointcloud(cropped, x_ratio=0.02, y_ratio=0.02, z_ratio=0.4)

            return final


class VolumeEstimator:
    def __init__(self, grid_size: float = 0.1):
        self.grid_size = grid_size

    def estimate_full_volume(self, points: np.ndarray) -> float:
        if points.size == 0:
            return 0.0

        if points.shape[1] < 3:
            return 0.0

        global_min_z = np.min(points[:, 2])

        with np.errstate(divide='ignore', invalid='ignore'):
            gx = np.floor(points[:, 0] / self.grid_size).astype(int)
            gy = np.floor(points[:, 1] / self.grid_size).astype(int)

        keys = [f"{x},{y}" for x, y in zip(gx, gy)]
        grid_map = {}

        for i, key in enumerate(keys):
            z = points[i, 2]
            if not np.isfinite(z):
                continue
            if key not in grid_map or z > grid_map[key]:
                grid_map[key] = z

        vol = 0.0
        cell_area = self.grid_size * self.grid_size
        for h in grid_map.values():
            dh = h - global_min_z
            if dh > 1e-6:
                vol += dh * cell_area
        return vol

    def estimate_volume_in_polygon_roi(self, points: np.ndarray, polygon: List[List[float]]) -> Tuple[float, int]:
        if len(points) == 0:
            return 0.0, 0

        if len(polygon) < 3:
            return 0.0, 0

        try:
            path = MplPath(polygon)
            inside_mask = path.contains_points(points[:, :2])

            inside_mask = np.asarray(inside_mask, dtype=bool)

            roi_points = points[inside_mask]
            count = int(np.sum(inside_mask))

            vol = self.estimate_full_volume(roi_points)
            return vol, count
        except Exception as e:
            logging.warning(f"Invalid polygon or error in calculation: {e}")
            return 0.0, 0

    def save_roi_points_as_pcd(self, points: np.ndarray, polygon: List[List[float]], output_path: str) -> bool:
        """
        将ROI区域内的点保存为PCD文件
        
        Args:
            points: 所有点的数组
            polygon: 定义ROI区域的多边形
            output_path: 输出PCD文件的路径
            
        Returns:
            是否成功保存文件
        """
        try:
            path = MplPath(polygon)
            inside_mask = path.contains_points(points[:, :2])
            inside_mask = np.asarray(inside_mask, dtype=bool)
            
            roi_points = points[inside_mask]
            
            # 创建一个新的PointCloud对象并保存
            roi_pcd = o3d.geometry.PointCloud()
            roi_pcd.points = o3d.utility.Vector3dVector(roi_points)
            
            logger.info(f"ROI区域内有 {len(roi_points)} 个点，正在保存到 {output_path}")
            success = o3d.io.write_point_cloud(output_path, roi_pcd)
            if success:
                logger.info(f"ROI点云已成功保存到 {output_path}")
            else:
                logger.error(f"保存ROI点云失败: {output_path}")
            return success
        except Exception as e:
            logger.error(f"保存ROI点云时发生错误: {e}")
            return False

    def estimate_volumes_with_rois(self, points: np.ndarray, rois: List[List[List[float]]]) -> dict:
        total_vol = self.estimate_full_volume(points)
        results = []
        part_total = 0.0

        for idx, poly in enumerate(rois):
            vol, count = self.estimate_volume_in_polygon_roi(points, poly)

            part_total += vol
            results.append({
                "roi_index": idx,
                "polygon": poly,
                "point_count": count,
                "volume_m3": round(vol, 3)
            })

        return {
            "total_volumes": round(total_vol, 3),
            "part_total_volume": round(part_total, 3),
            "grid_size_m": self.grid_size,
            "roi_volumes": results
        }


def process(input_path: str, output_path: str = None, grid_size: float = 0.1, rois: str = None):
    input_path = Path(input_path)
    if not input_path.exists():
        logger.error(f"输入文件不存在: {input_path}")
        sys.exit(1)

    if output_path is None:
        output_path = input_path.with_stem(input_path.stem + "_preprocessed")
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    logger.info(f"读取点云文件: {input_path}")
    pcd = o3d.io.read_point_cloud(str(input_path))
    if pcd.is_empty():
        logger.error("点云文件为空或读取失败")
        sys.exit(1)

    original_count = len(pcd.points)
    logger.info(f"原始点云点数: {original_count}")

    logger.info("开始预处理...")
    preprocessor = PointCloudPreprocessor()
    cleaned = preprocessor.preprocess(pcd)

    cleaned_count = len(cleaned.points)
    logger.info(f"预处理后点数: {cleaned_count} (移除 {original_count - cleaned_count} 个点)")

    logger.info(f"保存预处理后点云到: {output_path}")
    success = o3d.io.write_point_cloud(str(output_path), cleaned)
    if not success:
        logger.error("保存预处理后点云失败")
        sys.exit(1)
    logger.info("预处理点云保存成功")

    points = np.asarray(cleaned.points)
    if points.size == 0:
        logger.error("预处理后点云为空，无法计算体积")
        sys.exit(1)

    estimator = VolumeEstimator(grid_size=grid_size)

    if rois:
        try:
            roi_data = json.loads(rois)
        except json.JSONDecodeError as e:
            logger.error(f"ROI JSON 解析失败: {e}")
            sys.exit(1)

        if not isinstance(roi_data, list):
            logger.error("rois 必须是多边形列表")
            sys.exit(1)

        # 如果指定了ROI，同时保存ROI点云
        for idx, poly in enumerate(roi_data):
            roi_output_path = output_path.with_stem(f"{output_path.stem}_roi_{idx}")
            estimator.save_roi_points_as_pcd(points, poly, str(roi_output_path))
        
        result = estimator.estimate_volumes_with_rois(points, roi_data)
        logger.info(f"总体积: {result['total_volumes']} m³")
        logger.info(f"ROI 区域总体积: {result['part_total_volume']} m³")
        for item in result["roi_volumes"]:
            logger.info(f"  ROI {item['roi_index']}: 体积={item['volume_m3']} m³, 点数={item['point_count']}")
        print(json.dumps(result, ensure_ascii=False, indent=2))
    else:
        total_vol = estimator.estimate_full_volume(points)
        logger.info(f"总体积: {round(total_vol, 3)} m³")
        print(json.dumps({
            "total_volumes": round(total_vol, 3),
            "grid_size_m": grid_size,
            "original_point_count": original_count,
            "preprocessed_point_count": cleaned_count,
            "preprocessed_file": str(output_path)
        }, ensure_ascii=False, indent=2))


def main():
    parser = argparse.ArgumentParser(description="点云预处理与体积测算工具")
    parser.add_argument("input", help="输入 PCD 文件路径")
    parser.add_argument("-o", "--output", default=None, help="预处理后点云输出路径 (默认: 输入文件名_preprocessed.pcd)")
    parser.add_argument("-g", "--grid-size", type=float, default=0.1, help="体积计算栅格大小 (默认: 0.1m)")
    parser.add_argument("-r", "--rois", default=None, help="ROI 多边形区域 (JSON 字符串)")
    args = parser.parse_args()

    process(args.input, args.output, args.grid_size, args.rois)


if __name__ == "__main__":
    main()
