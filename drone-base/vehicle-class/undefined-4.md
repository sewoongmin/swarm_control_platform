---
description: 드론의 다양한 위치정보를 얻거나 설정하기 위한 인터페이스들
---

# 위치정보를 위한 인터페이스

## bool setHomeGlobal()

```cpp
bool Vehicle::setHomeGlobal()
{
	mavros_msgs::CommandHome msg;

	msg.request.current_gps = false;
	msg.request.latitude = cur_global_.latitude;
	msg.request.longitude = cur_global_.longitude;
	msg.request.altitude = cur_global_.altitude;

	if (set_home_client_.call(msg) && msg.response.success)
	{
		home_global_ = cur_global_;

		ROS_INFO_STREAM(vehicle_info_.vehicle_name_ << 
			" Global home position is set. " << msg.response.result);
		ROS_INFO("%s new global home is (long : %lf, lang : %lf, alti : %lf)", 
			vehicle_info_.vehicle_name_.c_str(), home_global_.longitude, 
			home_global_.latitude, home_global_.altitude);
	}
	else
		ROS_ERROR_STREAM(vehicle_info_.vehicle_name_ << 
			" Failed to set global home position. " << msg.response.result);

	return msg.response.success;
}
```

mavros/cmd/set\_home 서비스를 통해 드론의 글로벌 홈 좌표 현재 위치로 설정하는 함수.

## sensor\_msgs::NavSatFix getHomeGlobal() const

```cpp
sensor_msgs::NavSatFix Vehicle::getHomeGlobal() const
{
	return home_global_;
}
```

현재의 글로벌 홈 좌표를 리턴한다.

`home_global_`은 `sensor_msgs::NavSatFix` 타입으로 정의 되어 있다.&#x20;

## sensor\_msgs::NavSatFix getGlobalPosition() const

```cpp
sensor_msgs::NavSatFix Vehicle::getGlobalPosition() const
{
	return cur_global_;
}
```

현재 드론의 GPS 위치를 리턴한다.&#x20;

`cur_global_`은 `sensor_msgs::NavSatFix` 타입으로 정의 되어 있다.&#x20;

## sensor\_msgs::NavSatFix getTargetGlobal() const

```cpp
sensor_msgs::NavSatFix Vehicle::getTargetGlobal() const
{
	return tar_global_;
}
```

GPS 기준의 목표지점을 리턴한다.

`tar_global_`은 `sensor_msgs::NavSatFix` 타입으로 정의 되어 있다. &#x20;

## void setHomeLocal()

```cpp
void Vehicle::setHomeLocal()
{
	home_local_ = cur_local_;

	ROS_INFO_STREAM(vehicle_info_.vehicle_name_ << " Local home position is set.");
	ROS_INFO("%s new local home is (x : %lf, y : %lf, z : %lf)", 
		vehicle_info_.vehicle_name_.c_str(), home_local_.pose.position.x, 
		home_local_.pose.position.y, home_local_.pose.position.z);
}
```

로컬 홈 좌표를 현재의 로컬 좌표로 지정한다.&#x20;

{% hint style="info" %}
로컬 홈 설정을 펌웨어에 직접적으로 설할 수 없기 때문에 내부적으로만 저장하고 현재 따로 사용되고 있지 않다.
{% endhint %}

{% hint style="warning" %}
펌웨어의 로컬 홈을 직접 설정할 수 없으므로 토픽으로 받아오는 값으로 계속 업데이트 되기 때문에 setHomeLocal() 함수 자체를 없애도 된다.
{% endhint %}

## geometry\_msgs::PoseStamped getHomeLocal() const

```cpp
geometry_msgs::PoseStamped Vehicle::getHomeLocal() const
{
	return home_local_;
}
```

드론의 로컬 홈을 리턴한다.

`home_local_` 은 `geometry_msgs::PoseStamped` 타입으로 정의 되어 있다.

## geometry\_msgs::PoseStamped getLocalPosition() const

```cpp
geometry_msgs::PoseStamped Vehicle::getLocalPosition() const
{
	return cur_local_;
}
```

드론의 현재 로컬 위치를 리턴한다.

`cur_local_` 은 `geometry_msgs::PoseStamped` 타입으로 정의 되어 있다.

## geometry\_msgs::PoseStamped getTargetLocal() const

```cpp
geometry_msgs::PoseStamped Vehicle::getTargetLocal() const
{
	return tar_local_;
}
```

드론의 로컬 목표지점을 리턴한다.

`tar_local_` 은 `geometry_msgs::PoseStamped` 타입으로 정의 되어 있다.
