# 실행 방법

## 실행방법( 차선인식 테스트)

### 1. SSH로 Turtlebot3에 달린 라즈베리파이에 접속 후 uvc\_camera와 robot을 함께&#x20;

```
ssh heromin@192.168.0.60
roslaunch turtlebot3_autorace_construction_camera camera_with_robot.launch
```

### 2. 노트북의 터미널에서 lane\_detection 런치를 켬

```
roslaunch turtlebot3_autorace_construction_camera lane_detection_test.launch
```

## 실행방법 전체 테스트

### 1. SSH로 Turtlebot3에 달린 라즈베리파이에 접속 후 uvc\_camera와 robot을 함께&#x20;

```
ssh heromin@192.168.0.60
roslaunch turtlebot3_autorace_construction_camera camera_with_robot.launch
```

### 2. 노트북의 터미널에서 turtlebot3\_autorace\_icsl.launch 런치를 켬

```
roslaunch turtlebot3_autorace_construction_control turtlebot3_autorace_icsl.launch
```

### 3. 노트북의 터미널에서 turtlebot3\_autorace\_icsl.launch 런치를 켬

```
roslaunch turtlebot3_autorace_construction_control turtlebot3_autorace_control_lane.launch
```
