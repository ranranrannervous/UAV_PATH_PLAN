# 无人机高光谱遥感任务规划系统

一个完整的无人机高光谱遥感任务规划系统，支持采样点生成、太阳方位计算、路径规划优化。

## 系统架构

系统由三个核心模块组成：

- **采样点生成器** (`SamplingPointsGenerator`) - 生成网格化采样点
- **太阳方位计算器** (`SolarCalculator`) - 计算最优飞行方向
- **路径规划优化器** (`PathPlanningOptimizer`) - 优化飞行路径



## 快速开始

### 基本使用示例

```python
from sampling_points_generator import SamplingPointsGenerator
from solar_calculator import SolarCalculator  
from path_planning_optimizer import PathPlanningOptimizer

# 1. 生成采样点
point_generator = SamplingPointsGenerator()
points = point_generator.generate_sampling_points(
    corner1=(22.000, 113.000),  # 区域西南角
    corner2=(22.005, 113.005),  # 区域东北角
    spacing_x=50,               # 东西间距50米
    spacing_y=50,               # 南北间距50米
    rotation_angle=135          # 网格旋转角度
)

# 2. 计算太阳方位角
solar_calc = SolarCalculator()
sun_azimuth = solar_calc.get_sun_azimuth(22.545, 114.057, "2025-07-01 12:00:00")
flight_direction = solar_calc.get_flight_direction(sun_azimuth)

# 3. 路径规划
planner = PathPlanningOptimizer()
mission_plan = planner.plan_mission(
    points=points,
    start_point=(22.005, 113.005),
    flight_direction=flight_direction
)

# 4. 输出结果
planner.visualize_mission_plan(mission_plan)
planner.export_mission_csv(mission_plan)
```

## 模块详细使用

### 1. 采样点生成器 (SamplingPointsGenerator)

#### 基本调用

```python
from sampling_points_generator import SamplingPointsGenerator

generator = SamplingPointsGenerator()

# 生成采样点
points = generator.generate_sampling_points(
    corner1=(22.000, 113.000),    # 区域角点1 (纬度, 经度)
    corner2=(22.005, 113.005),    # 区域角点2 (纬度, 经度)
    spacing_x=30,                 # X方向间距(米)
    spacing_y=30,                 # Y方向间距(米)，可选
    rotation_angle=0,             # 旋转角度(度)，可选
    filter_boundary=True          # 是否过滤边界外的点，可选
)

# 保存可视化
generator.save_sampling_visualization(
    points=points,
    corner1=(22.000, 113.000),
    corner2=(22.005, 113.005),
    rotation_angle=0,
    spacing_x=30,
    spacing_y=30
)

# 导出CSV
generator.export_points_to_csv(points)
```

#### 参数说明

- `corner1`, `corner2`: 矩形区域的两个对角点坐标
- `spacing_x`: 东西方向采样间距（米）
- `spacing_y`: 南北方向采样间距（米），默认等于 spacing_x
- `rotation_angle`: 网格旋转角度（度），0为正北方向
- `filter_boundary`: 是否过滤区域边界外的点

### 2. 太阳方位计算器 (SolarCalculator)

#### 基本调用

```python
from solar_calculator import SolarCalculator

calculator = SolarCalculator()

# 计算太阳方位角
sun_azimuth = calculator.get_sun_azimuth(
    lat=22.545,                    # 纬度
    lon=114.057,                   # 经度
    time_str='2025-07-01 12:00:00' # 时间字符串
)

# 计算最优飞行方向（太阳方位角+135°）
flight_direction = calculator.get_flight_direction(sun_azimuth)

# 计算时间范围内的数据
data = calculator.calculate_for_time_range(
    lat=22.545,
    lon=114.057,
    date_str='2025-07-01',
    start_hour=6,                  # 开始时间
    end_hour=20                    # 结束时间
)

# 保存可视化
calculator.save_visualization(data, lat=22.545, lon=114.057, date_str='2025-07-01')

# 导出CSV
calculator.export_to_csv(data)
```

#### 参数说明

- `lat`, `lon`: 位置的纬度和经度
- `time_str`: 时间字符串，格式为 'YYYY-MM-DD HH:MM:SS'
- `date_str`: 日期字符串，格式为 'YYYY-MM-DD'
- `start_hour`, `end_hour`: 计算的时间范围

### 3. 路径规划优化器 (PathPlanningOptimizer)

#### 基本调用

```python
from path_planning_optimizer import PathPlanningOptimizer

planner = PathPlanningOptimizer()

# 基本路径规划
mission_plan = planner.plan_mission(
    points=points,                     # 采样点列表
    start_point=(22.005, 113.005),     # 起飞点
    flight_direction=135.0             # 飞行方向，可选
)

# 不同起降点规划
mission_plan = planner.plan_mission(
    points=points,
    start_point=(22.002, 113.002),     # 起飞点
    flight_direction=135.0,
    end_point=(22.008, 113.008)        # 降落点
)

# 可视化结果
image_path = planner.visualize_mission_plan(mission_plan)

# 导出任务数据
csv_path = planner.export_mission_csv(mission_plan)
```

#### 参数说明

- `points`: 采样点列表，格式为 [(lat1, lon1), (lat2, lon2), ...]
- `start_point`: 起飞点坐标 (纬度, 经度)
- `end_point`: 降落点坐标，可选，默认与起飞点相同
- `flight_direction`: 推荐飞行方向（度），可选

#### 飞行参数配置

```python
# 可以在初始化时配置飞行参数
planner = PathPlanningOptimizer()
planner.max_flight_time = 25 * 60      # 最大飞行时间（秒）
planner.sampling_time = 60             # 单点采样时间（秒）
planner.flight_speed = 10.0            # 飞行速度（米/秒）
planner.takeoff_landing_time = 90      # 起降时间（秒）
```

## 完整工作流示例

```python
from sampling_points_generator import SamplingPointsGenerator
from solar_calculator import SolarCalculator  
from path_planning_optimizer import PathPlanningOptimizer

def complete_mission_planning():
    # 1. 生成采样点
    generator = SamplingPointsGenerator()
    points = generator.generate_sampling_points(
        corner1=(22.000, 113.000),
        corner2=(22.005, 113.005),
        spacing_x=50,
        rotation_angle=135
    )
    
    # 2. 计算最优飞行方向
    solar_calc = SolarCalculator()
    sun_azimuth = solar_calc.get_sun_azimuth(22.545, 114.057, "2025-07-01 12:00:00")
    flight_direction = solar_calc.get_flight_direction(sun_azimuth)
    
    # 3. 路径规划
    planner = PathPlanningOptimizer()
    mission_plan = planner.plan_mission(
        points=points,
        start_point=(22.002, 113.002),    # 起飞点
        end_point=(22.008, 113.008),      # 降落点
        flight_direction=flight_direction
    )
    
    # 4. 输出结果
    planner.visualize_mission_plan(mission_plan)
    planner.export_mission_csv(mission_plan)
    
    print(f"任务规划完成！")
    print(f"覆盖点数: {mission_plan['planning_info']['planned_points']}")
    print(f"飞行段数: {mission_plan['summary']['num_segments']}")
    print(f"总距离: {mission_plan['summary']['total_distance']/1000:.2f} km")

if __name__ == "__main__":
    complete_mission_planning()
```

## 输出文件

系统会在以下目录生成输出文件：

- `sampling_results/` - 采样点相关文件
  - 采样点可视化图片 (PNG)
  - 采样点数据 (CSV)

- `solar_results/` - 太阳角度相关文件
  - 太阳角度变化图 (PNG)
  - 太阳角度数据 (CSV)

- `path_results/` - 路径规划相关文件
  - 任务规划可视化图 (PNG)
  - 飞行任务数据 (CSV)

## 注意事项

1. **坐标系统**: 所有坐标均为 WGS84 经纬度坐标
2. **时间格式**: 时间均使用 'Asia/Shanghai' 时区
3. **距离计算**: 使用 Haversine 公式计算地球表面距离
4. **文件编码**: 所有CSV文件均使用UTF-8编码

## 许可证

MIT License
