---
description: ubnutu 18.04 ros-melodic 환경에서 python3로 node를 만들기 위한 환경설정
---

# Melodic python3 setup

## Setup Step

### 1. Env check

ubuntu 18.04 bionic

ros-melodic

### 2. Install packages

```shell
sudo apt-get install python3-pip python3-yaml
sudo pip3 install rospkg catkin_pkg
```

### 3. Test rospy in terminal

터미널에서 다음의 명령어를 통해 python3 실행

```shell
python3
```

다음 두 패키지를 import 하여 정상 동작 여부 확인

```python
import rospy
import cv_bridge
```

위 코드 입력시 에러가 발생하는 경우 제대로 설정 되지 않은 것

에러가 없을 경우 여기서 Setup 완료

### 4. import cv\_bridge Error 발생시 해결 방

```shell
mkdir ~/catkin_build_ws && cd ~/catkin_build_ws
catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/bin/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
catkin config --install
mkdir src && cd src
git clone -b melodic https://github.com/ros-perception/vision_opencv.git
cd ..
catkin_make
```

여기까지 했다면 문제가 안생길&#x20;
