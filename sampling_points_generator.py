"""
无人机高光谱遥感采样点生成模块
支持矩形区域网格生成、角度旋转、自定义间距、结果可视化保存
sampling_points_generator.py
"""
import numpy as np
import math
import os
from typing import List, Tuple, Optional
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from datetime import datetime


class SamplingPointsGenerator:
    """
    采样点生成器
    支持根据矩形区域边界生成旋转网格采样点，并保存可视化结果
    """
    
    def __init__(self, output_dir: str = "sampling_results"):
        """
        初始化采样点生成器
        
        Args:
            output_dir: 输出文件夹路径
        """
        # 地球半径常数 (米)
        self.EARTH_RADIUS = 6378137.0
        
        # 输出目录
        self.output_dir = output_dir
        self._ensure_output_dir()
        
    def _ensure_output_dir(self):
        """确保输出目录存在"""
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            print(f"创建输出目录: {self.output_dir}")
    
    def deg_to_rad(self, degrees: float) -> float:
        """角度转弧度"""
        return degrees * math.pi / 180.0
    
    def rad_to_deg(self, radians: float) -> float:
        """弧度转角度"""
        return radians * 180.0 / math.pi
    
    def lat_lon_to_meters(self, lat: float, lon: float, ref_lat: float, ref_lon: float) -> Tuple[float, float]:
        """
        将经纬度坐标转换为以参考点为原点的米制坐标
        使用简化的投影方法，适用于小范围区域
        
        Args:
            lat, lon: 目标点经纬度
            ref_lat, ref_lon: 参考点经纬度（作为原点）
            
        Returns:
            (x, y): 米制坐标，x为东向，y为北向
        """
        # 转换为弧度
        lat_rad = self.deg_to_rad(lat)
        lon_rad = self.deg_to_rad(lon)
        ref_lat_rad = self.deg_to_rad(ref_lat)
        ref_lon_rad = self.deg_to_rad(ref_lon)
        
        # 计算距离差
        delta_lat = lat_rad - ref_lat_rad
        delta_lon = lon_rad - ref_lon_rad
        
        # 转换为米制坐标
        # y为北向距离
        y = delta_lat * self.EARTH_RADIUS
        # x为东向距离，需要考虑纬度修正
        x = delta_lon * self.EARTH_RADIUS * math.cos(ref_lat_rad)
        
        return x, y
    
    def meters_to_lat_lon(self, x: float, y: float, ref_lat: float, ref_lon: float) -> Tuple[float, float]:
        """
        将米制坐标转换回经纬度坐标
        
        Args:
            x, y: 米制坐标（米）
            ref_lat, ref_lon: 参考点经纬度
            
        Returns:
            (lat, lon): 经纬度坐标
        """
        ref_lat_rad = self.deg_to_rad(ref_lat)
        
        # 转换回经纬度差
        delta_lat = y / self.EARTH_RADIUS
        delta_lon = x / (self.EARTH_RADIUS * math.cos(ref_lat_rad))
        
        # 计算目标点经纬度
        lat = ref_lat + self.rad_to_deg(delta_lat)
        lon = ref_lon + self.rad_to_deg(delta_lon)
        
        return lat, lon
    
    def rotate_point(self, x: float, y: float, angle_deg: float, 
                    center_x: float = 0, center_y: float = 0) -> Tuple[float, float]:
        """
        绕指定中心点旋转坐标
        
        Args:
            x, y: 原始坐标
            angle_deg: 旋转角度（度），正值为逆时针
            center_x, center_y: 旋转中心
            
        Returns:
            (new_x, new_y): 旋转后坐标
        """
        angle_rad = self.deg_to_rad(angle_deg)
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)
        
        # 平移到原点
        dx = x - center_x
        dy = y - center_y
        
        # 旋转
        new_dx = dx * cos_a - dy * sin_a
        new_dy = dx * sin_a + dy * cos_a
        
        # 平移回去
        new_x = new_dx + center_x
        new_y = new_dy + center_y
        
        return new_x, new_y
    
    def get_region_info(self, corner1: Tuple[float, float], corner2: Tuple[float, float]) -> dict:
        """
        获取区域信息
        
        Args:
            corner1, corner2: 区域对角点
            
        Returns:
            dict: 包含区域中心、尺寸等信息
        """
        lat1, lon1 = corner1
        lat2, lon2 = corner2
        
        # 区域中心
        center_lat = (lat1 + lat2) / 2
        center_lon = (lon1 + lon2) / 2
        
        # 区域尺寸（近似）
        width_m = abs(lon2 - lon1) * 111000 * math.cos(self.deg_to_rad(center_lat))
        height_m = abs(lat2 - lat1) * 111000
        
        return {
            'center': (center_lat, center_lon),
            'width_m': width_m,
            'height_m': height_m,
            'area_m2': width_m * height_m
        }
    
    def generate_grid_points(self, 
                           corner1: Tuple[float, float], 
                           corner2: Tuple[float, float],
                           spacing_x: float = 30.0, 
                           spacing_y: Optional[float] = None,
                           rotation_angle: float = 0.0) -> List[Tuple[float, float]]:
        """
        生成旋转网格采样点
        
        Args:
            corner1, corner2: 矩形区域的两个对角点经纬度 (lat, lon)
            spacing_x: X方向间距(米) - 东西方向
            spacing_y: Y方向间距(米) - 南北方向，如果为None则等于spacing_x
            rotation_angle: 旋转角度(度)，相对于正北方向，正值为逆时针
            
        Returns:
            List[Tuple[float, float]]: 采样点列表 [(lat, lon), ...]
        """
        if spacing_y is None:
            spacing_y = spacing_x
            
        lat1, lon1 = corner1
        lat2, lon2 = corner2
        
        # 确定参考点（区域中心）
        ref_lat = (lat1 + lat2) / 2
        ref_lon = (lon1 + lon2) / 2
        
        # 将区域边界转换为米制坐标
        x1, y1 = self.lat_lon_to_meters(lat1, lon1, ref_lat, ref_lon)
        x2, y2 = self.lat_lon_to_meters(lat2, lon2, ref_lat, ref_lon)
        
        # 确定边界
        min_x, max_x = min(x1, x2), max(x1, x2)
        min_y, max_y = min(y1, y2), max(y1, y2)
        
        # 扩展边界以确保覆盖完整
        margin = max(spacing_x, spacing_y)
        min_x -= margin
        max_x += margin
        min_y -= margin
        max_y += margin
        
        # 生成网格点
        grid_points = []
        
        # 计算网格起始点，使其对齐
        start_x = math.floor(min_x / spacing_x) * spacing_x
        start_y = math.floor(min_y / spacing_y) * spacing_y
        
        # 计算网格点数量
        num_x = int(math.ceil((max_x - start_x) / spacing_x)) + 1
        num_y = int(math.ceil((max_y - start_y) / spacing_y)) + 1
        
        for i in range(num_x):
            for j in range(num_y):
                x = start_x + i * spacing_x
                y = start_y + j * spacing_y
                
                # 如果需要旋转，绕区域中心旋转
                if rotation_angle != 0:
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2
                    x, y = self.rotate_point(x, y, rotation_angle, center_x, center_y)
                
                # 转换回经纬度
                lat, lon = self.meters_to_lat_lon(x, y, ref_lat, ref_lon)
                grid_points.append((lat, lon))
        
        return grid_points
    
    def filter_points_in_rotated_rectangle(self, 
                                         points: List[Tuple[float, float]],
                                         corner1: Tuple[float, float], 
                                         corner2: Tuple[float, float],
                                         rotation_angle: float = 0.0) -> List[Tuple[float, float]]:
        """
        过滤出在旋转矩形区域内的点
        
        Args:
            points: 待过滤的点列表
            corner1, corner2: 原始矩形区域对角点
            rotation_angle: 旋转角度
            
        Returns:
            List[Tuple[float, float]]: 区域内的点
        """
        lat1, lon1 = corner1
        lat2, lon2 = corner2
        
        ref_lat = (lat1 + lat2) / 2
        ref_lon = (lon1 + lon2) / 2
        
        # 原始矩形边界
        x1, y1 = self.lat_lon_to_meters(lat1, lon1, ref_lat, ref_lon)
        x2, y2 = self.lat_lon_to_meters(lat2, lon2, ref_lat, ref_lon)
        min_x, max_x = min(x1, x2), max(x1, x2)
        min_y, max_y = min(y1, y2), max(y1, y2)
        
        center_x = (min_x + max_x) / 2
        center_y = (min_y + max_y) / 2
        
        filtered_points = []
        
        for lat, lon in points:
            # 转换为米制坐标
            x, y = self.lat_lon_to_meters(lat, lon, ref_lat, ref_lon)
            
            # 反向旋转点，检查是否在原始矩形内
            if rotation_angle != 0:
                x, y = self.rotate_point(x, y, -rotation_angle, center_x, center_y)
            
            # 检查是否在原始矩形内
            if min_x <= x <= max_x and min_y <= y <= max_y:
                filtered_points.append((lat, lon))
        
        return filtered_points
    
    def generate_sampling_points(self, 
                               corner1: Tuple[float, float], 
                               corner2: Tuple[float, float],
                               spacing_x: float = 30.0, 
                               spacing_y: Optional[float] = None,
                               rotation_angle: float = 0.0,
                               filter_boundary: bool = True) -> List[Tuple[float, float]]:
        """
        生成采样点的主要接口
        
        Args:
            corner1, corner2: 矩形区域对角点 (lat, lon)
            spacing_x: X方向间距(米) - 东西方向
            spacing_y: Y方向间距(米) - 南北方向
            rotation_angle: 旋转角度(度)，正值为逆时针
            filter_boundary: 是否过滤边界外的点
            
        Returns:
            List[Tuple[float, float]]: 采样点列表
        """
        # 生成网格点
        points = self.generate_grid_points(corner1, corner2, spacing_x, spacing_y, rotation_angle)
        
        # 如果需要，过滤边界外的点
        if filter_boundary:
            points = self.filter_points_in_rotated_rectangle(points, corner1, corner2, rotation_angle)
        
        return points
    
    def save_sampling_visualization(self, 
                                  points: List[Tuple[float, float]], 
                                  corner1: Tuple[float, float], 
                                  corner2: Tuple[float, float],
                                  rotation_angle: float = 0.0,
                                  spacing_x: float = 30.0,
                                  spacing_y: float = 30.0,
                                  filename: Optional[str] = None) -> str:
        """
        保存采样点可视化图表为PNG文件
        
        Args:
            points: 采样点列表
            corner1, corner2: 区域边界
            rotation_angle: 旋转角度
            spacing_x, spacing_y: 采样间距
            filename: 文件名，如果为None则自动生成
            
        Returns:
            str: 保存的文件路径
        """
        if not points:
            print("警告：没有采样点可以可视化")
            return ""
        
        # 生成文件名
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"sampling_points_{rotation_angle}deg_{spacing_x}x{spacing_y}m_{timestamp}.png"
        
        filepath = os.path.join(self.output_dir, filename)
        
        # 创建图表
        plt.figure(figsize=(12, 10))
        
        # 获取坐标数据
        lats, lons = zip(*points)
        
        # 绘制采样点
        plt.scatter(lons, lats, c='red', s=15, alpha=0.8, label=f'Sampling Points ({len(points)})', zorder=5)
        
        # 绘制区域边界
        lat1, lon1 = corner1
        lat2, lon2 = corner2
        
        if rotation_angle == 0:
            # 简单矩形
            rect_lons = [lon1, lon2, lon2, lon1, lon1]
            rect_lats = [lat1, lat1, lat2, lat2, lat1]
            plt.plot(rect_lons, rect_lats, 'b-', linewidth=2, label='Original Area', zorder=3)
        else:
            # 绘制原始矩形（虚线）
            rect_lons = [lon1, lon2, lon2, lon1, lon1]
            rect_lats = [lat1, lat1, lat2, lat2, lat1]
            plt.plot(rect_lons, rect_lats, 'b--', linewidth=1, alpha=0.5, label='Original Area', zorder=2)
            
            # 旋转矩形的四个顶点
            ref_lat = (lat1 + lat2) / 2
            ref_lon = (lon1 + lon2) / 2
            
            corners = [(lat1, lon1), (lat1, lon2), (lat2, lon2), (lat2, lon1)]
            rotated_corners = []
            
            for lat, lon in corners:
                x, y = self.lat_lon_to_meters(lat, lon, ref_lat, ref_lon)
                center_x, center_y = 0, 0  # 参考点就是中心
                x, y = self.rotate_point(x, y, rotation_angle, center_x, center_y)
                lat, lon = self.meters_to_lat_lon(x, y, ref_lat, ref_lon)
                rotated_corners.append((lat, lon))
            
            rotated_corners.append(rotated_corners[0])  # 闭合
            rot_lats, rot_lons = zip(*rotated_corners)
            plt.plot(rot_lons, rot_lats, 'b-', linewidth=2, label='Rotated Area', zorder=3)
        
        # 添加网格和标签
        plt.xlabel('Longitude (°)', fontsize=12)
        plt.ylabel('Latitude (°)', fontsize=12)
        
        # 标题包含详细信息
        region_info = self.get_region_info(corner1, corner2)
        title = f'Sampling Points - {rotation_angle}° Rotation\n'
        title += f'Grid: {spacing_x}×{spacing_y}m | Points: {len(points)} | '
        title += f'Area: {region_info["width_m"]:.0f}×{region_info["height_m"]:.0f}m'
        plt.title(title, fontsize=14, pad=20)
        
        plt.legend(loc='upper right')
        plt.grid(True, alpha=0.3)
        
        # 设置坐标轴等比例
        plt.axis('equal')
        
        # 调整边距
        plt.tight_layout()
        
        # 保存图片
        plt.savefig(filepath, dpi=300, bbox_inches='tight', facecolor='white')
        plt.close()  # 关闭图形，释放内存
        
        print(f"可视化结果已保存: {filepath}")
        return filepath
    
    def export_points_to_csv(self, points: List[Tuple[float, float]], 
                           filename: Optional[str] = None) -> str:
        """
        导出采样点到CSV文件
        
        Args:
            points: 采样点列表
            filename: 文件名，如果为None则自动生成
            
        Returns:
            str: 保存的文件路径
        """
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"sampling_points_{timestamp}.csv"
        
        filepath = os.path.join(self.output_dir, filename)
        
        with open(filepath, 'w', encoding='utf-8') as f:
            f.write("index,latitude,longitude\n")
            for i, (lat, lon) in enumerate(points, 1):
                f.write(f"{i},{lat:.8f},{lon:.8f}\n")
        
        print(f"采样点数据已导出: {filepath} (共{len(points)}个点)")
        return filepath


def test_sampling_generator():
    """测试函数 - 生成多种场景的采样点并保存可视化结果"""
    print("=" * 60)
    print("无人机高光谱遥感采样点生成模块 - 完整测试")
    print("=" * 60)
    
    # 创建生成器实例
    generator = SamplingPointsGenerator(output_dir="sampling_results")
    
    # 测试区域：基于实际应用场景
    corner1 = (22.000, 113.000)  # 左下角
    corner2 = (22.010, 113.010)  # 右上角
    
    region_info = generator.get_region_info(corner1, corner2)
    print(f"\n测试区域信息:")
    print(f"  坐标范围: {corner1} 到 {corner2}")
    print(f"  区域中心: ({region_info['center'][0]:.6f}, {region_info['center'][1]:.6f})")
    print(f"  区域尺寸: {region_info['width_m']:.0f}m × {region_info['height_m']:.0f}m")
    print(f"  总面积: {region_info['area_m2']/1e6:.2f} km²")
    
    test_cases = [
        {
            "name": "标准网格 - 无旋转",
            "params": {"spacing_x": 30, "spacing_y": 30, "rotation_angle": 0},
            "description": "基础30m等间距网格"
        },
        {
            "name": "太阳角优化 - 135度旋转",
            "params": {"spacing_x": 30, "spacing_y": 30, "rotation_angle": 135},
            "description": "匹配太阳方位的135度旋转网格"
        },
        {
            "name": "高密度采样 - 20m间距",
            "params": {"spacing_x": 20, "spacing_y": 20, "rotation_angle": 45},
            "description": "高分辨率监测用的密集网格"
        },
        {
            "name": "矩形网格 - 不等间距",
            "params": {"spacing_x": 25, "spacing_y": 40, "rotation_angle": 90},
            "description": "适应地形特征的非等间距网格"
        },
        {
            "name": "粗网格 - 快速覆盖",
            "params": {"spacing_x": 50, "spacing_y": 50, "rotation_angle": 30},
            "description": "应急响应用的快速覆盖网格"
        }
    ]
    
    results = []
    
    print(f"\n开始生成 {len(test_cases)} 种测试场景...")
    print("-" * 60)
    
    for i, test_case in enumerate(test_cases, 1):
        print(f"\n[{i}/{len(test_cases)}] {test_case['name']}")
        print(f"参数: {test_case['params']}")
        print(f"说明: {test_case['description']}")
        
        # 生成采样点
        points = generator.generate_sampling_points(
            corner1, corner2, 
            **test_case['params']
        )
        
        # 保存可视化
        image_path = generator.save_sampling_visualization(
            points, corner1, corner2, 
            **test_case['params']
        )
        
        # 导出CSV（仅为第一个测试用例）
        csv_path = ""
        if i == 2:  # 为135度旋转的案例导出CSV
            csv_path = generator.export_points_to_csv(points)
        
        # 记录结果
        result = {
            "case": test_case['name'],
            "points_count": len(points),
            "image_path": image_path,
            "csv_path": csv_path,
            **test_case['params']
        }
        results.append(result)
        
        print(f"✓ 生成采样点: {len(points)} 个")
        if image_path:
            print(f"✓ 保存图片: {os.path.basename(image_path)}")
        if csv_path:
            print(f"✓ 导出CSV: {os.path.basename(csv_path)}")
    
    # 测试坐标转换精度
    print(f"\n" + "-" * 60)
    print("坐标转换精度验证:")
    test_lat, test_lon = 22.005, 113.005
    ref_lat, ref_lon = 22.000, 113.000
    
    x, y = generator.lat_lon_to_meters(test_lat, test_lon, ref_lat, ref_lon)
    back_lat, back_lon = generator.meters_to_lat_lon(x, y, ref_lat, ref_lon)
    
    lat_error = abs(test_lat - back_lat) * 1e6  # 微度
    lon_error = abs(test_lon - back_lon) * 1e6  # 微度
    
    print(f"  原始坐标: ({test_lat:.6f}, {test_lon:.6f})")
    print(f"  米制坐标: ({x:.2f}, {y:.2f}) 米")
    print(f"  转换回坐标: ({back_lat:.6f}, {back_lon:.6f})")
    print(f"  转换误差: 纬度 {lat_error:.2f} 微度, 经度 {lon_error:.2f} 微度")
    print(f"  距离误差: ~{lat_error * 0.11:.3f} 米")
    
    # 输出总结
    print(f"\n" + "=" * 60)
    print("测试完成 - 结果总结:")
    print("=" * 60)
    
    for result in results:
        spacing_info = f"{result['spacing_x']}×{result['spacing_y']}m"
        print(f"  {result['case']:25s} | {result['points_count']:4d} 点 | {spacing_info:8s} | {result['rotation_angle']:3.0f}°")
    
    total_points = sum(r['points_count'] for r in results)
    print(f"\n总计生成采样点: {total_points} 个")
    print(f"输出目录: {generator.output_dir}")
    print(f"生成文件数: {len([r for r in results if r['image_path']])} 张图片")
    
    return results


if __name__ == "__main__":
    # 运行完整测试
    test_results = test_sampling_generator()