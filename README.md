# py_bt_ros

### 과제 수행 3단계 미션

# **Step 1: 목적지 이동 (Move To Goal)**

# **Step 2: 사진 촬영 (Capture Image)**

# **Step 3: 복귀 (Return to Home)**


### 이 프로젝트는 ROS 2와 Webots 환경에서 Behavior Tree를 사용하여 로봇을 제어하는 과제(HW3)입니다. 실행을 위해서는 총 3개의 터미널이 필요합니다.

# webot 실행 (Terminal 1)
```
cd webots_ros2_ws
source install/local_setup.bash
ros2 launch webots_ros2_turtlebot robot_launch.py nav:=true
```

# 2D goal pose 버튼 생성 
```
RViz 툴바의 [+] 버튼 클릭

[2D Goal Pose] 선택하여 추가

추가된 툴의 Tool Properties 패널 열기

Topic Name을 /bt/goal_pose로 수정
```

# 카메라 서버 실행 (Terminal 2)

```
cd py_bt_ros
python3 camera_server.py

```
# 메인 노드 실행 (Terminal 3)

```
cd py_bt_ros
python3 main.py

```

# 실행 방법
```
RViz 상단 툴바에서 위에서 설정한 2D Goal Pose 버튼을 클릭합니다.

맵 상의 원하는 위치를 클릭하고 드래그하여 목표 지점과 방향을 설정합니다.

로봇이 이동을 시작하고 3단계 미션을 수행합니다.

```




