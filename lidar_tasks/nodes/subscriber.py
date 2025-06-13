#!/usr/bin/env python3
# encoding: utf-8

import numpy as np
import math
import statistics
from typing import List, Optional, Tuple
from scipy.signal import medfilt

import rospy
from sensor_msgs.msg import LaserScan
from typing import Final


ROS_SCAN_TOPIC: Final[str] = "/scan"
ROS_FILTERED_TOPIC: Final[str] = "/scan_filtered"


class ObstacleMovementDetector:
  def __init__(self):
    rospy.init_node('obstacle_movement_detector')

    # Параметры
    self.prev_distance: Optional[float] = None
    self.threshold = 0.02  # Порог в метрах
    self.fov_degrees = 30  # Сектор анализа (±30°)
    self.pub = rospy.Publisher(ROS_FILTERED_TOPIC, LaserScan, queue_size=10)
    self.sub = rospy.Subscriber(ROS_SCAN_TOPIC, LaserScan, self.scan_callback,queue_size=None)
    self.last_msg = LaserScan()
    self.moving_average_window = rospy.get_param('~moving_average_window', 5)
    self.median_filter_window = rospy.get_param('~median_filter_window', 3)

    # Здесь подпишитесь на измерения LiDAR ...

  def scan_callback(self, msg: LaserScan) -> None:
    self.last_msg = msg
    # Получение данных из сектора
    frontal_ranges = self.get_frontal_sector(msg.ranges, msg.angle_min, msg.angle_increment)

    if len(frontal_ranges) == 0:
      rospy.logwarn("Не удалось получить сканы фронального сектора!")
      return
    
    min_distance = np.min(frontal_ranges)
    print(min_distance)


  def get_frontal_sector(self, ranges: List[float], angle_min: float, angle_increment: float) -> List[float]:
    """
    Возвращает список расстояний в заданном секторе.
    Исключает значения inf (бесконечности) и nan.
    """
    frontal_ranges = []

    right_indexes = ranges[0:self.fov_degrees]
    left_indexes = ranges[-self.fov_degrees:]

    for scan_index in left_indexes:
       if  not scan_index == math.isnan and not scan_index == math.isinf:
          frontal_ranges.append(scan_index)
    
    for scan_index in right_indexes:
       if  not scan_index == math.isnan and not scan_index == math.isinf:
          frontal_ranges.append(scan_index)

    moved_average = self.moving_average(front_ranges=frontal_ranges) 
    median_filtred = self.median_filter(front_ranges=moved_average)
    pub_msg = self.last_msg
    # print(median_filtred)
    pub_msg.ranges = median_filtred
    self.pub.publish(pub_msg)

    return median_filtred

  def median_filter(self, front_ranges, window_size =3):
    array = np.array(front_ranges)
    filtered_array = medfilt(array, kernel_size=window_size)
    filtered_array.tolist()
    return filtered_array


  def moving_average(self, front_ranges, window_size=5):
    i = 0
    moving_averages = []
    while i < len(front_ranges) - window_size + 1:
        window_average = round(np.sum(front_ranges[i:i+window_size]) / window_size, 2)
      
        moving_averages.append(window_average)
      
        i += 1
    return moving_averages
    
  def log_movement(self, movement: str, distance: float, delta: float) -> None:
    """
    Логирует результат с цветовой подсветкой для терминала.
    Возможно работающий код...
    """
    colors = {
      "approaching": "\033[91m",  # Красный
      "receding": "\033[92m",     # Зелёный
      "stable": "\033[94m"        # Синий
    }
    reset = "\033[0m"

    # Обновляем счётчики
    if movement == "approaching":
      self.approach_count += 1
    elif movement == "receding":
      self.recede_count += 1
    else:
      self.stable_count += 1

    rospy.loginfo(
      f"{colors[movement]}{movement.upper()}{reset} | "
      f"Distance: {distance:.2f}m | "
      f"Change: {delta:.3f}m"
    )

def main() -> None:
  detector = ObstacleMovementDetector()
  rospy.spin()

if __name__ == '__main__':
  main()
