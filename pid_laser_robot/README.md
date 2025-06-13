# Домашнее задание №5 - ПИД-регулятор (LiDAR)

## Задание 1 - Симуляция в Gazebo

Запуск **gazebo** 

```
roslaunch pid_laser_robot gazebo.launch
```


## Задание 2 - Фильтрация данных лидара

Запуск **gazebo** 

```
roslaunch lidar_tasks gazebo.launch
```

Запуск **узла для фильтрации** 

```
rosrun pid_laser_robot subscriber.py
```


## Задание 3 - ПИД-регулятор

Запуск **gazebo** 

```
roslaunch pid_laser_robot gazebo.launch
```

Запуск **узла для фильтрации** 

```
rosrun pid_laser_robot subscriber.py
```

Запуск **узла PID регулятора** 

```
rosrun pid_laser_robot pid_robot.py
```

