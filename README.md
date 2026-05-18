# PCD点云栅格化投影工具

## 项目概述

这是一个用于将PCD格式的点云数据栅格化投影为PNG图像的Python工具。它支持多种对比度增强功能，可以有效提升点云数据的可视化效果。该工具能够：

- 将三维点云数据转换为二维栅格图像
- 支持多种图像增强功能
- 实现体素下采样以控制文件大小
- 提供灵活的参数配置选项

## 功能特性

- **点云栅格化投影**：将PCD点云按指定单元格大小投影为PNG图像
- **对比度增强**：支持线性拉伸、直方图均衡化和伽马校正
- **体素下采样**：减少点云密度，控制输出文件大小
- **自适应裁剪**：自动去除噪点和无效区域
- **多格式输出**：支持灰度图和彩色图输出
- **雷达点定位**：可在输出图像中标记雷达点位置

## 点云体积计算工具

### 功能特点

- **点云体积测算**：使用2.5D规则网格像元法计算体积
- **物理意义明确**：无经验系数，对数据精确匹配
- **坐标单位检测**：自动检测mm/cm/m单位并转换
- **数据类型检测**：区分规则网格和散乱点云
- **地面检测**：使用Z轴低百分位数检测地面高度
- **栅格化处理**：将3D散乱点云转换为2.5D规则网格
- **空单元格插值**：支持对空单元格进行插值填充

### 使用方法

```bash
python3 pcd_volume.py [res_dir] [out_dir]
```

### 参数说明

| 参数 | 类型 | 描述 |
|------|------|------|
| res_dir | 字符串 | 输入PCD文件所在目录（可选，默认为/res） |
| out_dir | 字符串 | 输出结果目录（可选，默认为/out） |

### 高级参数

可通过代码中kwargs传递以下参数：

| 参数 | 描述 | 默认值 |
|------|------|--------|
| grid_resolution | 网格分辨率（米） | 自动估算 |
| interpolate_empty_cells | 是否对空单元格进行插值 | True |
| voxel_size | 体素滤波大小 | 0.3 |

### 处理流程

1. **读取点云**：加载PCD文件
2. **坐标单位检测**：自动检测并转换单位至米
3. **数据类型检测**：判断是规则网格还是散乱点云
4. **栅格化处理**：将3D散乱点云转换为2.5D规则网格
5. **地面检测**：使用低百分位数检测地面高度
6. **体积计算**：使用像元法计算体积
7. **结果输出**：保存CSV结果和栅格化点云

## 点云ICP精准配准工具

### 功能特点

- **ICP配准算法**：实现点云的ICP（Iterative Closest Point）配准
- **CloudCompare风格**：参考CloudCompare中的Fine Registration工具
- **点到面模式**：支持点到面距离代替点到点距离
- **自适应阈值**：根据点云包围盒自动计算距离阈值
- **随机采样**：对大点云进行随机采样以提高效率
- **收敛检测**：基于RMS差值变化检测收敛

### 使用方法

```bash
# 基本配准，使用默认参数
python3 icp_registration.py -a aligned.pcd -r reference.pcd

# 严格收敛要求
python3 icp_registration.py -a aligned.pcd -r reference.pcd --rms-diff 1e-8

# 小重叠率场景 (如部分重叠的扫描)
python3 icp_registration.py -a aligned.pcd -r reference.pcd --overlap 0.5

# 使用点到点模式 (默认是点到面)
python3 icp_registration.py -a aligned.pcd -r reference.pcd --point-to-point
```

### 参数说明

| 参数 | 缩写 | 类型 | 描述 | 默认值 |
|------|------|------|------|--------|
| `--aligned` | `-a` | 字符串 | 待配准的点云文件路径（必需） | - |
| `--reference` | `-r` | 字符串 | 参考点云文件路径（必需） | - |
| `--output` | `-o` | 字符串 | 配准结果输出路径 | 自动生成 |
| `--rms-diff` | | 浮点数 | RMS difference收敛阈值 | 1.0e-5 |
| `--max-iterations` | | 整数 | 最大迭代次数 | 20 |
| `--overlap` | | 浮点数 | 重叠率 0~1 | 1.0 |
| `--point-to-plane` | | 标志 | 使用点到面距离 | 开启 |
| `--point-to-point` | | 标志 | 使用点到点距离 | 关闭 |
| `--random-sample-limit` | | 整数 | 随机采样上限 | 50000 |
| `--correspondence-distance` | | 浮点数 | 对应点搜索最大距离 | 自动计算 |

### 处理流程

1. **加载点云**：读取待配准和参考点云文件
2. **参数设置**：根据用户输入设置ICP参数
3. **迭代配准**：
   - 寻找最近邻对应点对
   - 重叠率过滤
   - 计算变换矩阵
   - 应用变换
   - 检查RMS差值收敛
4. **结果输出**：保存配准后的点云和变换矩阵

## 环境要求

- Python 3.x
- NumPy
- Pillow (PIL)
- Open3D
- Matplotlib (可选，用于彩色输出)
- SciPy (用于插值)

### 安装依赖

```bash
pip install numpy pillow open3d matplotlib scipy
```

## 使用方法

### 基本用法

```bash
python3 pcd_to_raster_enhanced.py --input test.pcd --output map.png --cell_size 0.1
```

### 完整参数示例

```bash
python3 pcd_to_raster_enhanced.py --input test.pcd --output map.png --cell_size 0.05 --voxel_size 0.25 --equalize
```

### 参数说明

| 参数 | 缩写 | 类型 | 描述 |
|------|------|------|------|
| `--input` | `-i` | 字符串 | 输入PCD文件路径（必需） |
| `--output` | `-o` | 字符串 | 输出PNG图像路径（必需） |
| `--cell_size` | `-s` | 浮点数 | 栅格尺寸（每个像素代表的实际距离，必需） |
| `--voxel_size` | `-v` | 浮点数 | PCD下采样体素大小（默认0.25） |
| `--x_min` | | 浮点数 | 投影X轴最小值（可选） |
| `--x_max` | | 浮点数 | 投影X轴最大值（可选） |
| `--y_min` | | 浮点数 | 投影Y轴最小值（可选） |
| `--y_max` | | 浮点数 | 投影Y轴最大值（可选） |
| `--radar_x` | | 浮点数 | 雷达点X坐标（默认0.0） |
| `--radar_y` | | 浮点数 | 雷达点Y坐标（默认0.0） |
| `--radar_z` | | 浮点数 | 雷达点Z坐标（默认0.0） |
| `--color` | | 标志 | 输出彩色图像（需matplotlib） |
| `--contrast_percentile` | | 浮点数 | 裁剪两侧百分位（如2表示使用2%~98%的数据） |
| `--equalize` | | 标志 | 启用直方图均衡化（会覆盖百分比裁剪效果） |
| `--gamma` | | 浮点数 | 伽马校正值（<1变亮，>1变暗） |

### 图像增强选项

#### 1. 百分比裁剪
```bash
--contrast_percentile 2  # 使用2%到98%的数据范围进行线性拉伸
```

#### 2. 直方图均衡化
```bash
--equalize  # 启用直方图均衡化，增强全局对比度
```

#### 3. 伽马校正
```bash
--gamma 0.5  # 使图像变亮
--gamma 2.0  # 使图像变暗
```

#### 4. 彩色输出
```bash
--color  # 输出彩色图像（使用jet颜色映射）
```

## 处理流程

1. **读取点云**：加载输入的PCD文件
2. **体素下采样**：使用指定的体素大小减少点数
3. **点云预处理**：自动去除噪点和无效区域
4. **栅格化投影**：将点云投影到二维平面
5. **图像增强**：应用指定的对比度增强算法
6. **输出图像**：保存为PNG格式

## 示例

### 基础栅格化
```bash
python3 pcd_to_raster_enhanced.py --input sample.pcd --output output.png --cell_size 0.1
```

### 带对比度增强
```bash
python3 pcd_to_raster_enhanced.py --input sample.pcd --output enhanced.png --cell_size 0.1 --contrast_percentile 2
```

### 使用直方图均衡化
```bash
python3 pcd_to_raster_enhanced.py --input sample.pcd --output equalized.png --cell_size 0.1 --equalize
```

### 综合示例
```bash
python3 pcd_to_raster_enhanced.py --input test.pcd --output final_map.png --cell_size 0.05 --voxel_size 0.25 --equalize --color
```

## 输出文件

- 主要输出：指定名称的PNG图像文件
- 日志文件：[pcd_to_raster.log](file:///home/panda/Projects/pcd_service/pcd_to_raster.log)，包含处理过程和雷达点位置信息
- 中间文件：fast_downsample.pcd，经过预处理的点云文件

## 与其他系统的集成

### 修改平台图片和PCD文件
如果需要修改平台的图片和PCD文件：
1. 查询数据库[ basic_service/t_system_file ](file:///home/panda/Projects/pcd_service/basic_service/t_system_file)表的path字段
2. 根据字段的值到服务器路径`/usr/local/gofastdfs/files/portal/`查询对应文件

### 平台体积测算裁剪服务
对于预处理服务，请参考：
```bash
# 指定输出文件
python3 preprocessing_service.py -pcd test.pcd -o output.pcd

# 自定义参数
python3 preprocessing_service.py -pcd test.pcd --grid_size 0.05 --min_points_per_cell 5
```

## 注意事项

- `voxel_size`值越大，生成的PCD文件越小，但可能会影响精度
- `--equalize`选项会覆盖百分比裁剪效果
- 彩色输出需要安装matplotlib库
- 建议根据点云数据的实际大小调整`cell_size`参数
- 处理大型点云文件时，适当增加`voxel_size`以提高处理速度

## 常见问题

1. **内存不足**
   - 尝试增大`--voxel_size`参数来降低点云密度

2. **输出图像对比度过低**
   - 使用`--equalize`进行直方图均衡化
   - 或使用`--contrast_percentile`调整对比度

3. **处理速度慢**
   - 减少输入点云密度，增大`--voxel_size`值
   - 调整`--cell_size`以获得合适的输出分辨率