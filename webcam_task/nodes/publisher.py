#!/usr/bin/env python3
# encoding: utf-8

import rospy
from sensor_msgs.msg import Image
import numpy as np

from typing import Final

# constants
ROS_NODE_NAME: Final[str] = "publisher"

ROS_PARAM_PUB_RATE: Final[int] = 5
ROS_IMAGE_TOPIC: Final[str] = "/usb_cam/image_raw"


def main() -> None:
  def generate_image(width=320, height=240):
    return [np.random.randint(0,255) for _ in range(width*height)] 
  

  rospy.init_node(ROS_NODE_NAME)

  pub_frequency: int = rospy.get_param("~rate", ROS_PARAM_PUB_RATE)

  # Q: Почему здесь не нужно писать rospy.resolve_name(ROS_IMAGE_TOPIC)?
  # A: rospy сам автоматически решает проблему с названиями, 
  # поэтому нам не надо вызывать rospy.resolve_name() 
  # в случае, например, дебага, можно использовать эту функцию
  publisher = rospy.Publisher(ROS_IMAGE_TOPIC, Image, queue_size=10)

  # Обратите внимание: топик "image" может переименоваться при запуске ROS-узла.
  # rosrun project_template publisher.py image:=image_raw
  # Более подробно об этом можно узнать по ссылке: http://wiki.ros.org/Names
  rospy.loginfo(f"Publishing to '{ROS_IMAGE_TOPIC}' at {pub_frequency} Hz ...")

  rate = rospy.Rate(pub_frequency)

  while not rospy.is_shutdown():
    # Задание 1: сгенерируйте случайное изображение.
    # Разрешение: 320 x 240 (ширина x высота).
    # Формат пикселей: монохром, 8-бит.
    # Создайте функцию для генерации изображения "generate_image(width = 320, height = 240)".
    generated = generate_image()
    publisher.publish(Image(width=320, height=240, encoding='mono8', data=generated))
  
    rate.sleep()


    

if __name__ == '__main__':
    main()
