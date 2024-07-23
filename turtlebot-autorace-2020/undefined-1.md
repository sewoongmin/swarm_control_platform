---
description: 로보티즈에서 제공하는 터틀봇3 오토레이스 2020 레퍼런스 코드
---

# 레퍼런스 코드

## 설치 방법

### 노트북 설치

```
cd ~/catkin_ws/src/
git clone https://github.com/ROBOTIS-GIT/turtlebot3_autorace_2020.git
cd ~/catkin_ws && catkin_make
```

{% hint style="info" %}
사용 된 기본 모델은 터틀봇3 버거다

터틀봇에 장착된 Single Board Computer(SBC) 와 통신하여 동작하는 ROS 패키지 이다.
{% endhint %}

### 테스트 환경

* 우분투 18.04
* ROS melodic
* Python2

### 의존성

#### Hardware

* 90도 이상의 광각 카메라 ( 사용을 안 할시 도로가 너무 좁게 보여 도로 추적이 힘듬)

#### Software

```
sudo apt-get install ros-melodic-image-transport ros-melodic-cv-bridge ros-melodic-vision-opencv python-opencv libopencv-dev ros-melodic-image-proc
```

{% hint style="danger" %}
패키지의 대부분의 노드가 Python으로 작성 되어있다. 테스트 환경인 우분투 18.04 까지는 기본적으로 ROS가 Python2를 지원하기 때문에 **Python2의 사용을 권장**한다.&#x20;
{% endhint %}

{% hint style="danger" %}
위 패키지의 의존성 패키지중의 하나인 raspicam\_node 는 [여기](https://github.com/UbiquityRobotics/raspicam\_node)서 확인 가능하나 **개발은 멈춰** 있는 것으로 보이고, **우분투 공식 패키지로 지원되지 않는다**. 또한 **ROS melodic 환경에서 git clone 하여 빌드를 시도하였으나 제대로 빌드가 되지 않는 문제**가 있다.&#x20;

가장 최근의 커밋에서 우분투 18.04를 위한 커밋을 한 것 같으나 실제로 빌드되진 않았다.
{% endhint %}
