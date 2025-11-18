# py_bt_ros

### ros2 HW3 

# webot 실행 
```
cd webots_ros2_ws
source install/local_setup.bash
ros2 launch webots_ros2_turtlebot robot_launch.py nav:=true
```

# 2D goal pose 버튼 생성 
```
rviz에서 + 버튼 , set goal , tool properties , topic name '/bt/goal_pose' 수정 
```

# 카메라 서버 실행 
```
cd final code 
cd py_bt_ros
python3 camera_server.py

```
# main.py 실행 

```
python3 main.py

```
# Rviz 인터페이스에서 2D Goal Pose를 사용한 목적지 설정
```
2D Goal Pose로 목적지 지정
```
### 수행완료 !!



