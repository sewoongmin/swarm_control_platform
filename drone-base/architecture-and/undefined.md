# 사용법

## 메인 문에서 인스턴스 생성(예시)

```cpp
#include <ros/ros.h>
#include <vehicle.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "drone_base_node");
	ros::NodeHandle nh("~");
	ros::Rate rate(10);

	double num_drone;
	nh.getParam("num_drone", num_drone);
	std::unique_ptr<SwarmVehicle> camila(new SwarmVehicle(nh, "camila", num_drone));

	while (ros::ok())
	{
		camila->run();

		ros::spinOnce();
		rate.sleep();
	}

	camila.release();
	return 0;
}
```

## 축약어 설정

### Terminal

```
gedit ~/.bashrc
```

```bash
alias eb='gedit ~/.bashrc'
alias sb='source ~/.bashrc'
alias d1='ssh -X icsl-pi@cmila1'
alias d2='ssh -X icsl-pi@cmila2'
alias d3='ssh -X icsl-pi@cmila3'
alias gs='git status'
alias gp='git pull'
alias cw='cd ~/catkin_ws'
alias cs='cd ~/catkin_ws/src'
alias cm='cd ~/catkin_ws && catkin_make'
alias connmav='rosrun mavros mavros_node _fcu_url:=/dev/ttyUSB0:57600'
alias qgc='~/Downloads/QGroundControl.AppImage'

alias mode='rostopic pub /multi/set_mode std_msgs/String '
alias rtl='rostopic pub /multi/set_mode std_msgs/String "auto.rtl"'
alias offboard='rostopic pub /multi/set_mode std_msgs/String "offboard"'
alias takeoff='rostopic pub /multi/set_mode std_msgs/String "auto.takeoff"'
alias land='rostopic pub /multi/set_mode std_msgs/String "auto.land"'
alias arm='rostopic pub /multi/arming std_msgs/Bool 1'
alias disarm='rostopic pub /multi/set_mode std_msgs/Bool 0'
alias goto='rosservice call /multi_setpoint_local -- POINT'
alias gotov='rosservice call /swarm_node/goto_vehicle -- '
alias idle='rosservice call /swarm_node/multi_setpoint_local -- IDLE 0 0 5'
alias sp_on='rosparam set /swarm_node/setpoint/separate true'
alias sp_off='rosparam set /swarm_node/setpoint/separate false'
alias scen='rosparam set /swarm_node/scen '
alias scen4='rosservice call /swarm_node/multi_setpoint_local -- SCEN4 0 0 10'
```

## 실행

### Terminal 1

```bash
roslaunch drone_base drone_base.launch
```

### Terminal 2

```
goto 0 0 2
```

```
arm
```

```
offboard
```

