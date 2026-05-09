"""
PCD 点云栅格化投影为 PNG 图像（增强对比度）
用法示例：
    python3 pcd_to_raster_enhanced.py --input test.pcd --output map.png --cell_size 0.05 --voxel_size 0.25 --equalize
可选增强选项：
    --contrast_percentile P   : 裁剪两侧百分位（例如 2 表示使用 2%~98% 的数据进行线性拉伸）
    --equalize                : 直方图均衡化（会覆盖百分比裁剪效果）
    --gamma G                 : 伽马校正（例如 0.5 变亮，2.0 变暗）
    --color                   : 彩色图
    --voxel_size 0.25         : 下采样体素大小，值越大生成PCD文件越小（默认值0.25）
"""

"""
修改平台的图片和PCD文件:
先查数据库basic_service/t_system_file表的path字段，根据字段的值到服务器查询/usr/local/gofastdfs/files/portal/
"""

"""
平台体积测算裁剪服务:
# 指定输出文件
python3 preprocessing_service.py -pcd test.pcd -o output.pcd

# 自定义参数
python3 preprocessing_service.py -pcd test.pcd --grid_size 0.05 --min_points_per_cell 5
"""
