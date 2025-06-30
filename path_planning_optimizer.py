"""
无人机路径规划优化模块
支持TSP路径优化、多段飞行规划、飞行时间估算
path_planning_optimizer.py
"""

import numpy as np
import matplotlib.pyplot as plt
import os
import math
import time
from typing import List, Tuple, Optional, Dict
from datetime import datetime
import random


class PathPlanningOptimizer:
    """
    路径规划优化器
    支持TSP求解、多段路径规划、飞行约束处理
    """
    
    def __init__(self, output_dir: str = "path_results"):
        self.output_dir = output_dir
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
        
        # 飞行参数
        self.max_flight_time = 30 * 60  # 30分钟（秒）
        self.sampling_time = 45  # 每个采样点的采样时间（秒）
        self.flight_speed = 8.0  # 飞行速度（米/秒）
        self.takeoff_landing_time = 60  # 起降时间（秒）
    
    def calculate_distance(self, point1: Tuple[float, float], point2: Tuple[float, float]) -> float:
        """
        计算两点间的距离（米）
        使用简化的经纬度距离计算
        
        Args:
            point1, point2: 经纬度坐标 (lat, lon)
            
        Returns:
            float: 距离（米）
        """
        lat1, lon1 = point1
        lat2, lon2 = point2
        
        # 转换为弧度
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lat = math.radians(lat2 - lat1)
        delta_lon = math.radians(lon2 - lon1)
        
        # Haversine公式
        a = (math.sin(delta_lat / 2) ** 2 + 
             math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lon / 2) ** 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = 6371000 * c  # 地球半径6371km
        
        return distance
    
    def calculate_flight_time(self, points: List[Tuple[float, float]], 
                            start_point: Tuple[float, float],
                            end_point: Optional[Tuple[float, float]] = None) -> Dict:
        """
        计算飞行时间信息
        
        Args:
            points: 采样点列表
            start_point: 起飞点
            end_point: 降落点，如果为None则与start_point相同
            
        Returns:
            Dict: 包含各种时间信息
        """
        if not points:
            return {}
        
        if end_point is None:
            end_point = start_point
        
        total_distance = 0
        
        # 从起点到第一个采样点
        total_distance += self.calculate_distance(start_point, points[0])
        
        # 采样点之间的距离
        for i in range(len(points) - 1):
            total_distance += self.calculate_distance(points[i], points[i + 1])
        
        # 从最后一个采样点到终点
        total_distance += self.calculate_distance(points[-1], end_point)
        
        # 计算时间
        flight_time = total_distance / self.flight_speed  # 飞行时间
        sampling_time = len(points) * self.sampling_time  # 采样时间
        total_time = flight_time + sampling_time + self.takeoff_landing_time
        
        return {
            'total_distance': total_distance,
            'flight_time': flight_time,
            'sampling_time': sampling_time,
            'takeoff_landing_time': self.takeoff_landing_time,
            'total_time': total_time,
            'num_points': len(points),
            'avg_speed': self.flight_speed
        }
    
    def tsp_nearest_neighbor(self, points: List[Tuple[float, float]], 
                           start_point: Tuple[float, float]) -> List[int]:
        """
        最近邻TSP算法（快速但非最优）
        
        Args:
            points: 采样点列表
            start_point: 起点
            
        Returns:
            List[int]: 访问顺序的索引列表
        """
        if not points:
            return []
        
        n = len(points)
        visited = [False] * n
        path = []
        
        # 找到距离起点最近的点作为第一个访问点
        current_pos = start_point
        
        for _ in range(n):
            min_dist = float('inf')
            next_idx = -1
            
            for i in range(n):
                if not visited[i]:
                    dist = self.calculate_distance(current_pos, points[i])
                    if dist < min_dist:
                        min_dist = dist
                        next_idx = i
            
            if next_idx != -1:
                visited[next_idx] = True
                path.append(next_idx)
                current_pos = points[next_idx]
        
        return path
    
    def tsp_2opt(self, points: List[Tuple[float, float]], 
                initial_path: List[int], max_iterations: int = 100) -> List[int]:
        """
        2-opt优化算法（性能优化版）
        
        Args:
            points: 采样点列表
            initial_path: 初始路径
            max_iterations: 最大迭代次数
            
        Returns:
            List[int]: 优化后的路径
        """
        if len(points) < 4:
            return initial_path
        
        def calculate_path_distance(path):
            total_dist = 0
            for i in range(len(path)):
                j = (i + 1) % len(path)
                total_dist += self.calculate_distance(points[path[i]], points[path[j]])
            return total_dist
        
        current_path = initial_path.copy()
        best_distance = calculate_path_distance(current_path)
        
        no_improvement_count = 0
        max_no_improvement = min(10, max_iterations // 5)  # 早期停止
        
        for iteration in range(max_iterations):
            improved = False
            
            for i in range(len(current_path) - 1):
                for j in range(i + 2, len(current_path)):
                    # 避免相邻边
                    if j == len(current_path) - 1 and i == 0:
                        continue
                    
                    # 2-opt交换
                    new_path = current_path.copy()
                    new_path[i+1:j+1] = reversed(new_path[i+1:j+1])
                    
                    new_distance = calculate_path_distance(new_path)
                    
                    if new_distance < best_distance:
                        current_path = new_path
                        best_distance = new_distance
                        improved = True
                        no_improvement_count = 0
                        break
                
                if improved:
                    break
            
            if not improved:
                no_improvement_count += 1
                if no_improvement_count >= max_no_improvement:
                    print(f"  2-opt在第{iteration+1}次迭代后收敛")
                    break
        
        return current_path
    
    def optimize_path(self, points: List[Tuple[float, float]], 
                     start_point: Tuple[float, float],
                     flight_direction: Optional[float] = None) -> List[int]:
        """
        路径优化主函数
        
        Args:
            points: 采样点列表
            start_point: 起降点
            flight_direction: 推荐飞行方向（度）
            
        Returns:
            List[int]: 优化后的访问顺序
        """
        if not points:
            return []
        
        print(f"正在优化 {len(points)} 个采样点的路径...")
        
        # 对于大量点，使用更简单的算法
        if len(points) > 100:
            print("点数较多，使用方向排序算法...")
            if flight_direction is not None:
                return self.sort_by_direction(points, start_point, flight_direction)
            else:
                return self.tsp_nearest_neighbor(points, start_point)
        
        # 如果有推荐飞行方向，尝试按方向排序
        if flight_direction is not None:
            sorted_path = self.sort_by_direction(points, start_point, flight_direction)
        else:
            # 使用最近邻算法获取初始路径
            sorted_path = self.tsp_nearest_neighbor(points, start_point)
        
        # 使用2-opt算法优化（仅对小规模问题）
        print("正在进行2-opt优化...")
        max_iter = min(50, len(points))  # 动态调整迭代次数
        optimized_path = self.tsp_2opt(points, sorted_path, max_iterations=max_iter)
        
        return optimized_path
    
    def sort_by_direction(self, points: List[Tuple[float, float]], 
                         start_point: Tuple[float, float], 
                         direction: float) -> List[int]:
        """
        按指定方向排序采样点
        
        Args:
            points: 采样点列表
            start_point: 起点
            direction: 方向角度（度）
            
        Returns:
            List[int]: 排序后的索引列表
        """
        if not points:
            return []
        
        # 计算每个点相对于起点在指定方向上的投影
        direction_rad = math.radians(direction)
        projections = []
        
        for i, point in enumerate(points):
            # 计算相对位置
            delta_lat = point[0] - start_point[0]
            delta_lon = point[1] - start_point[1]
            
            # 转换为米制坐标（简化）
            x = delta_lon * 111000 * math.cos(math.radians(start_point[0]))
            y = delta_lat * 111000
            
            # 在指定方向上的投影
            projection = x * math.sin(direction_rad) + y * math.cos(direction_rad)
            projections.append((projection, i))
        
        # 按投影距离排序
        projections.sort()
        return [idx for _, idx in projections]
    
    def split_into_flights(self, points: List[Tuple[float, float]], 
                          start_point: Tuple[float, float],
                          optimized_path: List[int],
                          end_point: Optional[Tuple[float, float]] = None) -> List[List[int]]:
        """
        将路径分割为多个飞行段（考虑续航限制）
        
        Args:
            points: 采样点列表
            start_point: 起飞点
            optimized_path: 优化后的路径
            end_point: 降落点，如果为None则与start_point相同
            
        Returns:
            List[List[int]]: 多个飞行段的路径列表
        """
        if not optimized_path:
            return []
        
        if end_point is None:
            end_point = start_point
        
        flights = []
        current_flight = []
        current_time = self.takeoff_landing_time
        current_pos = start_point
        
        for i, point_idx in enumerate(optimized_path):
            point = points[point_idx]
            
            # 计算到这个点的飞行时间
            flight_time_to_point = self.calculate_distance(current_pos, point) / self.flight_speed
            
            # 计算从这个点返回的时间
            is_last_point = (i == len(optimized_path) - 1)
            if is_last_point:
                return_time = self.calculate_distance(point, end_point) / self.flight_speed
            else:
                return_time = self.calculate_distance(point, start_point) / self.flight_speed
            
            # 总时间 = 当前时间 + 飞行到点 + 采样 + 返回
            total_time_needed = current_time + flight_time_to_point + self.sampling_time + return_time
            
            if total_time_needed <= self.max_flight_time:
                # 可以访问这个点
                current_flight.append(point_idx)
                current_time += flight_time_to_point + self.sampling_time
                current_pos = point
            else:
                # 需要开始新的飞行段
                if current_flight:
                    flights.append(current_flight)
                
                current_flight = [point_idx]
                current_time = self.takeoff_landing_time + self.sampling_time
                current_pos = point
        
        # 添加最后一个飞行段
        if current_flight:
            flights.append(current_flight)
        
        return flights
    
    def plan_mission(self, points: List[Tuple[float, float]], 
                    start_point: Tuple[float, float],
                    flight_direction: Optional[float] = None,
                    end_point: Optional[Tuple[float, float]] = None) -> Dict:
        """
        完整任务规划
        
        Args:
            points: 采样点列表
            start_point: 起降点
            flight_direction: 推荐飞行方向
            end_point: 降落点，如果为None则与start_point相同
            
        Returns:
            Dict: 完整的任务规划信息
        """
        if end_point is None:
            end_point = start_point
            
        print(f"开始任务规划...")
        print(f"  采样点数量: {len(points)}")
        print(f"  起降点: {start_point}")
        print(f"  飞行方向: {flight_direction:.1f}°" if flight_direction else "  飞行方向: 未指定")
        
        start_time = time.time()
        
        # 路径优化
        optimized_path = self.optimize_path(points, start_point, flight_direction)
        optimization_time = time.time() - start_time
        print(f"路径优化完成，耗时: {optimization_time:.2f} 秒")
        
        # 分割飞行段
        print("正在分割飞行段...")
        flight_segments = self.split_into_flights(points, start_point, optimized_path, end_point)
        
        # 计算每个飞行段的信息
        segment_info = []
        for i, segment in enumerate(flight_segments):
            segment_points = [points[idx] for idx in segment]
            is_last_segment = (i == len(flight_segments) - 1)
            segment_end = end_point if is_last_segment else start_point
            time_info = self.calculate_flight_time(segment_points, start_point, segment_end)
            
            segment_info.append({
                'segment_id': i + 1,
                'point_indices': segment,
                'point_count': len(segment),
                'time_info': time_info
            })
        
        # 总体统计
        total_points = sum(len(seg) for seg in flight_segments)
        total_distance = sum(info['time_info']['total_distance'] for info in segment_info)
        total_time = sum(info['time_info']['total_time'] for info in segment_info)
        
        planning_time = time.time() - start_time
        
        print(f"任务规划完成!")
        print(f"  规划总耗时: {planning_time:.2f} 秒")
        print(f"  飞行段数: {len(flight_segments)}")
        print(f"  覆盖点数: {total_points}/{len(points)} ({100*total_points/len(points):.1f}%)")
        print(f"  总飞行距离: {total_distance/1000:.2f} km")
        print(f"  总任务时间: {total_time/3600:.2f} 小时")
        
        mission_plan = {
            'planning_info': {
                'total_points': len(points),
                'planned_points': total_points,
                'coverage_rate': total_points / len(points) if points else 0,
                'planning_time': planning_time,
                'flight_direction': flight_direction
            },
            'flight_segments': segment_info,
            'summary': {
                'num_segments': len(flight_segments),
                'total_distance': total_distance,
                'total_time': total_time,
                'total_flight_hours': total_time / 3600
            },
            'raw_data': {
                'points': points,
                'start_point': start_point,
                'end_point': end_point,
                'optimized_path': optimized_path,
                'flight_segments': flight_segments
            }
        }
        
        return mission_plan
    
    def visualize_mission_plan(self, mission_plan: Dict, filename: Optional[str] = None) -> str:
        """
        可视化任务规划结果
        
        Args:
            mission_plan: 任务规划数据
            filename: 保存文件名
            
        Returns:
            str: 保存的文件路径
        """
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"mission_plan_{timestamp}.png"
        
        filepath = os.path.join(self.output_dir, filename)
        
        points = mission_plan['raw_data']['points']
        start_point = mission_plan['raw_data']['start_point']
        end_point = mission_plan['raw_data']['end_point']
        flight_segments = mission_plan['raw_data']['flight_segments']
        
        if not points:
            print("警告: 没有采样点可以可视化")
            return ""
        
        plt.figure(figsize=(14, 10))
        
        # 提取坐标
        lats = [p[0] for p in points]
        lons = [p[1] for p in points]
        
        # 绘制所有采样点
        plt.scatter(lons, lats, c='lightblue', s=30, alpha=0.6, label='Sampling Points')
        
        # 绘制起降点
        plt.scatter(start_point[1], start_point[0], c='green', s=100, 
                   marker='^', label='Start/Landing Point', zorder=5)
        
        # 如果降落点不同，单独绘制
        if end_point != start_point:
            plt.scatter(end_point[1], end_point[0], c='red', s=100, 
                       marker='v', label='Landing Point', zorder=5)
        
        # 为每个飞行段绘制不同颜色的路径
        colors = ['red', 'blue', 'orange', 'purple', 'brown', 'pink', 'gray', 'olive']
        
        for i, segment in enumerate(flight_segments):
            if not segment:
                continue
                
            color = colors[i % len(colors)]
            
            # 构建这个段的完整路径
            is_last_segment = (i == len(flight_segments) - 1)
            segment_end = end_point if is_last_segment else start_point
            segment_path = [start_point] + [points[idx] for idx in segment] + [segment_end]
            
            path_lats = [p[0] for p in segment_path]
            path_lons = [p[1] for p in segment_path]
            
            plt.plot(path_lons, path_lats, color=color, linewidth=2, 
                    alpha=0.8, label=f'Flight {i+1} ({len(segment)} points)')
            
            # 标记这个段的采样点
            segment_lons = [points[idx][1] for idx in segment]
            segment_lats = [points[idx][0] for idx in segment]
            plt.scatter(segment_lons, segment_lats, c=color, s=50, alpha=0.8, zorder=4)
        
        plt.xlabel('Longitude (°)')
        plt.ylabel('Latitude (°)')
        
        # 标题包含统计信息
        summary = mission_plan['summary']
        planning_info = mission_plan['planning_info']
        
        title = f"Mission Plan - {summary['num_segments']} Flight Segments\n"
        title += f"Total: {planning_info['total_points']} points, "
        title += f"{summary['total_distance']/1000:.1f}km, "
        title += f"{summary['total_flight_hours']:.1f}h"
        
        if planning_info['flight_direction']:
            title += f" | Flight Direction: {planning_info['flight_direction']:.0f}°"
        
        plt.title(title, fontsize=12, pad=20)
        plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        
        plt.tight_layout()
        plt.savefig(filepath, dpi=300, bbox_inches='tight')
        plt.close()
        
        print(f"任务规划可视化已保存: {filepath}")
        return filepath
    
    def export_mission_csv(self, mission_plan: Dict, filename: Optional[str] = None) -> str:
        """
        导出任务规划到CSV文件
        
        Args:
            mission_plan: 任务规划数据
            filename: 文件名
            
        Returns:
            str: 保存的文件路径
        """
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"mission_plan_{timestamp}.csv"
        
        filepath = os.path.join(self.output_dir, filename)
        
        points = mission_plan['raw_data']['points']
        flight_segments = mission_plan['raw_data']['flight_segments']
        
        with open(filepath, 'w', encoding='utf-8') as f:
            f.write("segment_id,point_id,latitude,longitude,order_in_segment\n")
            
            for seg_id, segment in enumerate(flight_segments, 1):
                for order, point_idx in enumerate(segment, 1):
                    lat, lon = points[point_idx]
                    f.write(f"{seg_id},{point_idx},{lat:.8f},{lon:.8f},{order}\n")
        
        print(f"任务规划数据已导出: {filepath}")
        return filepath


def test_path_planner():
    """测试函数"""
    print("=" * 60)
    print("无人机路径规划优化模块测试")
    print("=" * 60)
    
    # 创建路径规划器
    planner = PathPlanningOptimizer()
    
    # 生成测试数据（模拟采样点）
    print("生成测试采样点...")
    
    # 起降点
    start_point = (22.005, 113.005)
    
    # 模拟一个小区域的采样点网格
    test_points = []
    base_lat, base_lon = 22.000, 113.000
    
    for i in range(8):
        for j in range(6):
            lat = base_lat + i * 0.001  # 约100米间距
            lon = base_lon + j * 0.001
            test_points.append((lat, lon))
    
    print(f"生成了 {len(test_points)} 个测试采样点")
    print(f"起降点: {start_point}")
    
    # 测试1: 基础路径规划
    print(f"\n" + "-" * 40)
    print("测试1: 基础路径规划")
    print("-" * 40)
    
    mission_plan = planner.plan_mission(test_points, start_point)
    
    planning_info = mission_plan['planning_info']
    summary = mission_plan['summary']
    
    print(f"规划结果:")
    print(f"  总采样点: {planning_info['total_points']} 个")
    print(f"  规划覆盖: {planning_info['planned_points']} 个 ({planning_info['coverage_rate']*100:.1f}%)")
    print(f"  飞行段数: {summary['num_segments']} 段")
    print(f"  总距离: {summary['total_distance']/1000:.2f} km")
    print(f"  总时间: {summary['total_flight_hours']:.2f} 小时")
    print(f"  规划耗时: {planning_info['planning_time']:.3f} 秒")
    
    # 显示各飞行段信息
    print(f"\n各飞行段详情:")
    for segment in mission_plan['flight_segments']:
        time_info = segment['time_info']
        print(f"  段 {segment['segment_id']}: {segment['point_count']} 点 | "
              f"{time_info['total_distance']/1000:.2f}km | "
              f"{time_info['total_time']/60:.1f}min")
    
    # 测试2: 带飞行方向的规划
    print(f"\n" + "-" * 40)
    print("测试2: 带飞行方向约束的规划")
    print("-" * 40)
    
    flight_direction = 135.0  # 模拟太阳方位角+135度
    mission_plan_directed = planner.plan_mission(test_points, start_point, flight_direction)
    
    directed_summary = mission_plan_directed['summary']
    print(f"飞行方向约束 {flight_direction}° 的规划结果:")
    print(f"  飞行段数: {directed_summary['num_segments']} 段")
    print(f"  总距离: {directed_summary['total_distance']/1000:.2f} km")
    print(f"  总时间: {directed_summary['total_flight_hours']:.2f} 小时")
    
    # 对比两种规划方法
    print(f"\n规划方法对比:")
    print(f"  无约束: {summary['total_distance']/1000:.2f}km, {summary['num_segments']}段")
    print(f"  有方向约束: {directed_summary['total_distance']/1000:.2f}km, {directed_summary['num_segments']}段")
    
    # 测试3: 可视化和导出
    print(f"\n" + "-" * 40)
    print("测试3: 可视化和导出")
    print("-" * 40)
    
    # 可视化
    image_path = planner.visualize_mission_plan(mission_plan_directed)
    
    # 导出CSV
    csv_path = planner.export_mission_csv(mission_plan_directed)
    
    # 测试4: 性能测试
    print(f"\n" + "-" * 40)
    print("测试4: 算法性能测试")
    print("-" * 40)
    
    # 测试不同规模的点集
    test_sizes = [10, 25, 50]
    
    for size in test_sizes:
        test_subset = test_points[:size] if size <= len(test_points) else test_points
        
        start_time = time.time()
        temp_plan = planner.plan_mission(test_subset, start_point)
        end_time = time.time()
        
        print(f"  {size:2d} 点规划: {end_time-start_time:.3f} 秒")
    
    print(f"\n" + "=" * 60)
    print("测试完成 - 文件输出:")
    print("=" * 60)
    print(f"  可视化图: {os.path.basename(image_path) if image_path else '无'}")
    print(f"  任务数据: {os.path.basename(csv_path) if csv_path else '无'}")
    print(f"  输出目录: {planner.output_dir}")
    
    return mission_plan_directed


if __name__ == "__main__":
    test_mission = test_path_planner()