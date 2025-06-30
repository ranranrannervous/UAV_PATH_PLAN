#solar_calculator.py
import pandas as pd
import zhplot
import matplotlib.pyplot as plt
import os
from datetime import datetime
from typing import List, Tuple, Optional

try:
    from pvlib import solarposition
    PVLIB_AVAILABLE = True
except ImportError:
    PVLIB_AVAILABLE = False
    print("警告: pvlib库未安装，请使用 'pip install pvlib' 安装")


class SolarCalculator:
    """
    太阳方位角计算器 - 简洁版
    主要功能：获取太阳方位角，计算135°偏移的最优飞行方向
    """
    
    def __init__(self, output_dir: str = "solar_results"):
        if not PVLIB_AVAILABLE:
            raise ImportError("需要安装pvlib: pip install pvlib")
        
        self.output_dir = output_dir
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
    
    def get_sun_azimuth(self, lat: float, lon: float, time_str: str) -> float:
        """
        获取太阳方位角（基于用户提供的代码）
        
        Args:
            lat: 纬度
            lon: 经度
            time_str: 时间字符串，如 '2025-07-01 12:00:00'
            
        Returns:
            float: 太阳方位角（度）
        """
        times = pd.date_range(time_str, periods=1, freq='1min', tz='Asia/Shanghai')
        solpos = solarposition.get_solarposition(times, latitude=lat, longitude=lon)
        return solpos['azimuth'].iloc[0]
    
    def get_flight_direction(self, sun_azimuth: float) -> float:
        """
        计算最优飞行方向（与太阳方位角相差135度）
        
        Args:
            sun_azimuth: 太阳方位角
            
        Returns:
            float: 推荐飞行方向（度）
        """
        return (sun_azimuth + 135) % 360
    
    def calculate_for_time_range(self, lat: float, lon: float, date_str: str,
                               start_hour: int = 6, end_hour: int = 20) -> pd.DataFrame:
        """
        计算指定时间范围内的太阳方位角和飞行方向
        
        Args:
            lat: 纬度
            lon: 经度
            date_str: 日期，如 '2025-07-01'
            start_hour: 开始小时
            end_hour: 结束小时
            
        Returns:
            pd.DataFrame: 包含时间、太阳方位角、飞行方向的数据
        """
        results = []
        
        for hour in range(start_hour, end_hour + 1):
            time_str = f"{date_str} {hour:02d}:00:00"
            
            try:
                sun_azimuth = self.get_sun_azimuth(lat, lon, time_str)
                flight_direction = self.get_flight_direction(sun_azimuth)
                
                results.append({
                    'time': f"{hour:02d}:00",
                    'sun_azimuth': round(sun_azimuth, 1),
                    'flight_direction': round(flight_direction, 1)
                })
            except Exception as e:
                print(f"计算时间 {time_str} 时出错: {e}")
        
        return pd.DataFrame(results)
    
    def save_visualization(self, data: pd.DataFrame, lat: float, lon: float, 
                         date_str: str, filename: Optional[str] = None) -> str:
        """
        保存太阳方位角和飞行方向的可视化图表
        
        Args:
            data: 计算数据
            lat, lon: 位置
            date_str: 日期
            filename: 文件名
            
        Returns:
            str: 保存的文件路径
        """
        if filename is None:
            filename = f"solar_flight_direction_{date_str}.png"
        
        filepath = os.path.join(self.output_dir, filename)
        
        plt.figure(figsize=(12, 6))
        
        # 转换时间为数值用于绘图
        hours = [int(t.split(':')[0]) for t in data['time']]
        
        plt.plot(hours, data['sun_azimuth'], 'b-o', linewidth=2, markersize=4, label='太阳方位角')
        plt.plot(hours, data['flight_direction'], 'r--s', linewidth=2, markersize=4, label='飞行方向(+135°)')
        
        plt.xlabel('时间 (小时)')
        plt.ylabel('方位角 (度)')
        plt.title(f'太阳方位角与飞行方向 - {date_str}\n位置: {lat:.3f}°N, {lon:.3f}°E')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.xlim(6, 20)
        plt.ylim(0, 360)
        
        # 设置y轴刻度
        plt.yticks(range(0, 361, 45))
        plt.xticks(range(6, 21, 2))
        
        plt.tight_layout()
        plt.savefig(filepath, dpi=300, bbox_inches='tight')
        plt.close()
        
        print(f"可视化图表已保存: {filepath}")
        return filepath
    
    def export_to_csv(self, data: pd.DataFrame, filename: Optional[str] = None) -> str:
        """
        导出数据到CSV文件
        
        Args:
            data: 数据
            filename: 文件名
            
        Returns:
            str: 保存的文件路径
        """
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"solar_data_{timestamp}.csv"
        
        filepath = os.path.join(self.output_dir, filename)
        data.to_csv(filepath, index=False, encoding='utf-8')
        
        print(f"数据已导出: {filepath}")
        return filepath


def test_solar_calculator():
    """测试函数"""
    print("=" * 50)
    print("太阳方位角计算模块测试")
    print("=" * 50)
    
    # 创建计算器
    calculator = SolarCalculator()
    
    # 测试参数
    lat, lon = 22.545, 114.057  # 深圳
    date_str = "2025-07-01"
    
    print(f"测试地点: {lat:.3f}°N, {lon:.3f}°E")
    print(f"测试日期: {date_str}")
    
    # 测试单点计算
    print(f"\n单点计算测试:")
    test_times = ["09:00:00", "12:00:00", "15:00:00"]
    
    for time in test_times:
        full_time = f"{date_str} {time}"
        sun_azimuth = calculator.get_sun_azimuth(lat, lon, full_time)
        flight_direction = calculator.get_flight_direction(sun_azimuth)
        
        print(f"  {time} | 太阳方位: {sun_azimuth:6.1f}° | 飞行方向: {flight_direction:6.1f}°")
    
    # 时间范围计算
    print(f"\n时间范围计算 (6:00-20:00):")
    data = calculator.calculate_for_time_range(lat, lon, date_str)
    
    print(f"  计算完成，共 {len(data)} 个时间点")
    print(f"  太阳方位角范围: {data['sun_azimuth'].min():.1f}° - {data['sun_azimuth'].max():.1f}°")
    print(f"  飞行方向范围: {data['flight_direction'].min():.1f}° - {data['flight_direction'].max():.1f}°")
    
    # 显示部分数据
    print(f"\n前5个时间点:")
    for _, row in data.head().iterrows():
        print(f"  {row['time']} | 太阳: {row['sun_azimuth']:6.1f}° | 飞行: {row['flight_direction']:6.1f}°")
    
    # 保存结果
    print(f"\n保存结果:")
    image_path = calculator.save_visualization(data, lat, lon, date_str)
    csv_path = calculator.export_to_csv(data)
    
    print(f"\n测试完成!")
    print(f"输出目录: {calculator.output_dir}")
    
    return data


if __name__ == "__main__":
    test_data = test_solar_calculator()