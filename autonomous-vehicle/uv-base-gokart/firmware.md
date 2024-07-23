---
description: 실제 GoKart 레포지토리와는 관계 없지만 차량 제어를 위해 Px4 펌웨어를 수정한 부분에 관해 서술함
---

# Firmware 수정

## Rover\_pos\_control

File 경로 :  Firmware/src/modules/rover\_pos\_control/RoverPositionControl.cpp

### 코드 변경 사유

기존 펌웨어 코드는 Rover 펌웨어 개발자들이 고정익 드론 코드를 변경하여 개발한 까닭으로 속도제어 부분이 **Local Frame** 기준 좌표계의 Linear x, y 속도를 가지고 제어하도록 구현되어 있다.  이는 자동차나 모바일 로봇등을 일반적으로 **Body Frame** 기준의 Linear x, Angular z 속도를 통해 제어하는 것과 너무 다르기 때문에 **Body Frame** 을 기준으로 제어하기 위해 펌웨어를 수정하였다.

### 변경된 코드&#x20;

{% tabs %}
{% tab title="변경 후" %}
```cpp
void
RoverPositionControl::control_velocity(const matrix::Vector3f &current_velocity,
				       const position_setpoint_triplet_s &pos_sp_triplet)
{
	float dt = 0.01; // Using non zero value to a avoid division by zero

	const float mission_throttle = _param_throttle_cruise.get();
	const float desired_speed = pos_sp_triplet.current.vx;

	if (desired_speed > 0.01f || desired_speed < -0.01f) {
		const Dcmf R_to_body(Quatf(_vehicle_att.q).inversed());
		// current_velocity는 body Frame이 아닌 local Frame 이니 body Frame으로 변경해주는 작업 필요!
		const Vector3f vel = R_to_body * Vector3f(current_velocity(0), current_velocity(1), current_velocity(2));

		const float x_vel = vel(0);
		const float x_acc = _vehicle_acceleration_sub.get().xyz[0];
		const float control_throttle = pid_calculate(&_speed_ctrl, desired_speed, x_vel, x_acc, dt);

		//Constrain maximum throttle to mission throttle
		_act_controls.control[actuator_controls_s::INDEX_THROTTLE] = math::constrain(control_throttle, -mission_throttle, mission_throttle);

		const float angular_z_speed = pos_sp_triplet.current.yawspeed; //angular z

		if (angular_z_speed > 0.01f || angular_z_speed < -0.01f)
		{ //조향이 있을때
			const float wheel_base = _param_wheel_base.get();
			float radius = abs(desired_speed) / angular_z_speed;
			const float desired_theta = atanf(wheel_base / radius);
			float control_effort = desired_theta / _param_max_turn_angle.get();
			control_effort = math::constrain(control_effort, -1.0f, 1.0f);

			_act_controls.control[actuator_controls_s::INDEX_YAW] = control_effort;
		}
		else {
			_act_controls.control[actuator_controls_s::INDEX_YAW] = 0.0f;
		}
	}
	else {
		_act_controls.control[actuator_controls_s::INDEX_THROTTLE] = 0.0f;

	}
}
```
{% endtab %}
{% endtabs %}

### 코드 분석

```cpp
void
RoverPositionControl::control_velocity(const matrix::Vector3f &current_velocity,
				       const position_setpoint_triplet_s &pos_sp_triplet)
```

이 부분을 통해 `control_velocity`라는 함수가 `current_velocity` 와 `pos_sp_triplet`을 매개변수로 실행한다는 것을 알 수있다.&#x20;

여기서 `current_velocity`는 Local Frame 기준의 속도값으로 **NED 좌표** 기준 x, y, z 속도 값이다. `pos_sp_triplet`은 입력으로 들어온 목표속도를 의미하며 Linerar x, y, z 와 Angular x, y, z를 포함하고, 기준 좌표계는 `pos_sp_triplet.current.velocity_frame` 의 값을 통해 알 수 있다. 모바일 로봇이나 자동차 제어를 위해서는 일반적으로 **Body Frame** 기준의 Linear x 와 Angular z를 이용한 제어가 사용된다.&#x20;

```cpp
float dt = 0.01; // Using non zero value to a avoid division by zero

const float mission_throttle = _param_throttle_cruise.get();
const float desired_speed = pos_sp_triplet.current.vx;
```

dt 는 밑에 나오는 `pid_calculate` 함수에서 사용된다. `mission_throttle`은 throttle\_cruise 파라미터 값을 받아오며 이는 QGC를 통해 변경 가능하다. `desired_speed`는 입력된 속도 값 중 linear x 속도 값을 받아온다. 이 코드는 **Body Frame** 기준의 Linear x 와 Angular z로 만 제어되기 때문에 vy, vz는 무시 되는 것을 알 수 있다.

```cpp
if (desired_speed > 0.01f || desired_speed < -0.01f) {
	const Dcmf R_to_body(Quatf(_vehicle_att.q).inversed());
	// current_velocity는 body Frame이 아닌 local Frame 이니 body Frame으로 변경해주는 작업 필요!
	const Vector3f vel = R_to_body * Vector3f(current_velocity(0), current_velocity(1), current_velocity(2));
```

이 함수는 속도로 throttle 과 steering을 제어하기 때문에 Linear x 속도가 0이라면 throttle과 steering 모두 어떠한 제어도 되지 않는다. `R_to_body` 변수를 **NED 좌표**계의 벡터에 곱해주면 **Body Frame** 으로 회전한다. 이를 이용해 위의 코드 처럼 매개변수로 입력된 `current_velocity`를 **NED 좌표에서 Body Frame으로 변경**해 준다.&#x20;

{% hint style="danger" %}
Angular z만 입력 되더라도 조향제어는 되어야 할 것 같지만 차량에서는 Linear 속도 없이 Angular 속도 또한 있을수 없다.
{% endhint %}

```cpp
const float x_vel = vel(0);
const float x_acc = _vehicle_acceleration_sub.get().xyz[0];
const float control_throttle = pid_calculate(&_speed_ctrl, desired_speed, x_vel, x_acc, dt);

//Constrain maximum throttle to mission throttle
_act_controls.control[actuator_controls_s::INDEX_THROTTLE] = math::constrain(control_throttle, -mission_throttle, mission_throttle);
```

`x_vel` 은 **Body Frame** 기준 현재 속도중 Linear x 값을 사용하고, `x_acc`는 펌웨어 어딘가에서 가속도 값을 가져오는 것 같은데 알수 없다. 원래 코드에서 사용되는 것을 그대로 사용했다. `control_throttle` 변수는 모터의 제어량인데 펌웨어 내부에서 만들어진 `pid_calculate` 함수가 사용되었다. 함수에서 입력된 매개변수를 보면 목표속도, 현재속도, 가속도, dt 등이 필요 한 것을 알 수 있다. 이 함수를 이용해야지 QGC를 통해 rover pid 파라미터를 변경하였을 때 제어량이 변경된다는 것을명심하자. 마지막 `_act_controls.control[]` 에 값을 주는 것을 통해 실제 모터를 제어한다. 여기서 constrain 함수의 제한 값에 `-mission_throttle` 과 `mission_throttle`이 들어가서 전진 후진 모두 가능하게 되었다.

```cpp
const float angular_z_speed = pos_sp_triplet.current.yawspeed; //angular z

if (angular_z_speed > 0.01f || angular_z_speed < -0.01f)
{ //조향이 있을때
	const float wheel_base = _param_wheel_base.get();
	float radius = abs(desired_speed) / angular_z_speed;
	const float desired_theta = atanf(wheel_base / radius);
	float control_effort = desired_theta / _param_max_turn_angle.get();
	control_effort = math::constrain(control_effort, -1.0f, 1.0f);

	_act_controls.control[actuator_controls_s::INDEX_YAW] = control_effort;
}
```

입력 속도에 Angular z 가 없을 경우 Steering이 필요 없으므로 `_act_controls.control[actuator_controls_s::INDEX_YAW]` 를 0으로 고정하면 된다. 다만 있을 경우 조향각도 (`desired_theta`) 를 구해야 하는데 이 코드는 [여기](http://wiki.ros.org/teb\_local\_planner/Tutorials/Planning%20for%20car-like%20robots)를 참고하여 구현 되었다.  참고한 코드에서 사용된 radius = v/w 수식은 [여기](https://courses.lumenlearning.com/physics/chapter/6-1-rotation-angle-and-angular-velocity/)를 통해 검증하였다. 여기서 v에 abs를 취해준 이유는 전진과 후진이 바뀔경우 조향각이 반대로 바뀌는 것을 해결해주기 위해 적용되었다. 이를 통해 구해진 `desired_theta`는 `max_turn_angle` 파라미터로 Normalize되어 모터에 입력된다. `max_turn_angle` 파라미터는 QGC를 통해 수정 가능하며 실제 하드웨어에 맞게 변경되어야 바람직하다.

## Mavlink streaming rate 조절

px4 펌웨어에서는 다양한 데이터가 필요에의해 일반적으로 꽤 빠른속도로 발행된다. 그 데이터들은 GPS 위치, Local position, setpoint position 등 다양하다. 시뮬레이션이나 통신환경의 문제로 주고 받는 데이터의 밴드위스를 조절하기 위해 최적화가 필요 할 경우 아래와 같은 작업이 요구된다.

file path : Firmware/ROMFS/px4fmu\_common/init.d-posix/rcS

```bash
# Configure vehicle type specific parameters.
# Note: rc.vehicle_setup is the entry point for rc.interface,
#       rc.fw_apps, rc.mc_apps, rc.rover_apps, and rc.vtol_apps.
#
sh etc/init.d/rc.vehicle_setup

# GCS link
mavlink start -x -u $udp_gcs_port_local -r 4000000
mavlink stream -r 50 -s POSITION_TARGET_LOCAL_NED -u $udp_gcs_port_local
mavlink stream -r 50 -s LOCAL_POSITION_NED -u $udp_gcs_port_local
mavlink stream -r 50 -s GLOBAL_POSITION_INT -u $udp_gcs_port_local
mavlink stream -r 0 -s ATTITUDE -u $udp_gcs_port_local
mavlink stream -r 0 -s ATTITUDE_QUATERNION -u $udp_gcs_port_local
mavlink stream -r 0 -s ATTITUDE_TARGET -u $udp_gcs_port_local
mavlink stream -r 0 -s SERVO_OUTPUT_RAW_0 -u $udp_gcs_port_local
mavlink stream -r 0 -s RC_CHANNELS -u $udp_gcs_port_local
mavlink stream -r 0 -s OPTICAL_FLOW_RAD -u $udp_gcs_port_local

# API/Offboard link
mavlink start -x -u $udp_offboard_port_local -r 4000000 -m onboard -o $udp_offboard_port_remote

# execute autostart post script if any
[ -e "$autostart_file".post ] && sh "$autostart_file".post

# Run script to start logging
sh etc/init.d/rc.logging

mavlink boot_complete
replay trystart
```

위 코드는 rcS 파일의 가장 하단부의 일부분이다. 저기 8째줄 부터 17번째 줄에서 볼수 있는 GCS link 라고 되어 있는 mavlink 의 stream rate를 조절 하면 원하는 대로 최적화를 할 수 있다. 이파일을 변경하는 것은 실제 pixhawk에 올리는 것이 아니라면 시뮬레이션 상에서는 펌웨어를 빌드하지 않고도 가능하다.
