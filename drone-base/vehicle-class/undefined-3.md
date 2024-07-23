---
description: 별도의 기능을 수행하는 인터페이스들
---

# 기타 인터페이스

## void setVehicleInfo(const VehicleInfo &)

```cpp
void Vehicle::setVehicleInfo(const VehicleInfo &new_vehicle_info)
{
	vehicle_info_.vehicle_id_ = new_vehicle_info.vehicle_id_;
	vehicle_info_.vehicle_name_ = new_vehicle_info.vehicle_name_;

	release();

	nh_ = ros::NodeHandle(vehicle_info_.vehicle_name_);

	vehicleInit();
}
```

차량의 아이디와 이름정보를 `VehicleInfo` 타입으로 받아서 업데이트 하는 함수.

{% hint style="info" %}
지금은 노드핸들의 생성자를 불러와서 직접 초기화를 했지만 nh\_.resolveName() 함수를 사용하는 것이 바람직해 보인다. \
테스트가 필요하지만 위 함수를 사용시 vehicleInit() 함수를 실행할 필요가 없을 것으로 예상된다.  [여기 참](http://wiki.ros.org/roscpp/Overview/Names%20and%20Node%20Information)
{% endhint %}

## VehicleInfo getInfo() const

```cpp
VehicleInfo Vehicle::getInfo() const
{
	return vehicle_info_;
}
```

차량의 정보를 리턴해주는 인터페이

## mavros\_msgs::State getState() const

```cpp
mavros_msgs::State Vehicle::getState() const
{
	return cur_state_;
}
```

`mavros::msgs::State` 타입인 `cur_state_` 변수를 리턴해주는 인터페이

## sensor\_msgs::BatteryState getBattery() const

```cpp
sensor_msgs::BatteryState Vehicle::getBattery() const
{
	return cur_battery_;
}
```

배터리 정보를 리턴해주는 인터페이스

## double getArmingYaw() const

```cpp
double getArmingYaw() const
{
    return arming_yaw_;
}
```

시동을 걸 때(arming) 기록된 yaw 값을 리턴해주는 인터페이스. 여기서 `arming_yaw_`는 double 타입.

## bool isPublish() const

```cpp
bool Vehicle::isPublish() const
{
	return setpoint_publish_flag_;
}
```

`setLocalTarget()` 함수나, `gotoGlobal()`을 통해 변경되는 `setpoint_publish_flag_`의 값을 리턴해주는 인터페이스.
