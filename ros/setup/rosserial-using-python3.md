---
description: python3 환경에서 rosserial 을 사용하기 위한 방법
---

# rosserial using python3

## 1. rosserial 제거

기존에 설치된 rosserial을 제거

```
sudo apt remove ros-melodic-rosserial*
```

### 2. python3 rosserial 설치

[여기](https://github.com/ros-drivers/rosserial)로 이동하여 git repository 주소를 복사하고 catkin\_ws에 복사

```
cd ~/catkin_ws/src
git clone -b noetic-devel https://github.com/ros-drivers/rosserial.git
```

