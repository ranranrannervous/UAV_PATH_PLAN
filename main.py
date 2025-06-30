
# main.py
from sampling_points_generator import SamplingPointsGenerator
from solar_calculator import SolarCalculator  
from path_planning_optimizer import PathPlanningOptimizer

point_generator = SamplingPointsGenerator()
points = point_generator.generate_sampling_points(
    corner1=(22.000, 113.000),
    corner2=(22.005, 113.005),  # 缩小到0.5km×0.5km
    spacing_x=50,  # 增大间距到50米
    rotation_angle=135
)

# 其余代码保持不变
solar_calc = SolarCalculator()
sun_azimuth = solar_calc.get_sun_azimuth(22.545, 114.057, "2025-07-01 12:00:00")
flight_direction = solar_calc.get_flight_direction(sun_azimuth)

# 设置不同的起飞点和降落点
takeoff_point = (22.002, 113.002)   # 起飞点
landing_point = (22.008, 113.008)   # 降落点

planner = PathPlanningOptimizer()
mission_plan = planner.plan_mission(
    points=points,
    start_point=takeoff_point,
    flight_direction=flight_direction,
    end_point=landing_point
)

planner.visualize_mission_plan(mission_plan)
planner.export_mission_csv(mission_plan)