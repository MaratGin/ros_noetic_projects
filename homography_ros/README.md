# Домашнее задание по гомографии

## Структура пакета

### fake_publisher.py 
Публикует фотографию под углом в топик /image_send.
```bash
rosrun homography_ros fake_publisher.py
```
## send_image_service.py
Является клиентом ROS-сервиса, берёт изображение из топика и посылает на сервер.  
```bash
rosrun homography_ros send_image_service.py
```
## homography.py
Является сервером ROS-сервиса, получает картинку и выполняет операцию гомографии над изображением.
```bash
rosrun homography_ros homography.py
```