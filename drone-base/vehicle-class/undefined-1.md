---
description: 한 대의 드론을 제어하기 위한 메인 제어 함수들
---

# 메인 제어 인터페이스

## bool arming(const bool &)

```cpp
bool Vehicle::arming(const bool &arm_state)
{
	double roll, pitch;
	mavros_msgs::CommandBool msg;
	msg.request.value = arm_state;
	tf::Quaternion q(
		cur_local_.pose.orientation.x,
		cur_local_.pose.orientation.y,
		cur_local_.pose.orientation.z,
		cur_local_.pose.orientation.w
	);
	tf::Matrix3x3 m(q);
	m.getRPY( roll, pitch, arming_yaw_ );

	std::cout << "set arming yaw: "<<arming_yaw_<<std::endl;

	if (arming_client_.call(msg) && msg.response.success)
		ROS_INFO_STREAM(msg.response.result);
	else
		ROS_ERROR_STREAM(vehicle_info_.vehicle_name_ << 
		    " failed to call arming service. " << msg.response.result);
	return msg.response.success;
}
```

Vehicle 객체와 연결된 하나의 드론의 시동(Arm)을 걸거나 풀기 위해서 사용되는 함수. Arming  요청이 있을때 내부 변수인 arming\_yaw\_에 현재 자세인 orientation으로 부터 추출 된 yaw 값을 저장한다.

Service 로 정의 된 arming\_client\_ 를 통해 Arm을 요청하고 그에 따른 response를 리턴한다. 즉, 부득이하게 arming 요청이 거절 된 경우 함수의 리턴 값도 false이다.

## bool setMode(const std::string &)

```cpp
bool Vehicle::setMode(const std::string &current_mode)
{
	mavros_msgs::SetMode mode;
	mode.request.custom_mode = current_mode;
	if (((current_mode == "offboard") || (current_mode == "OFFBOARD")) 
	    && (!setpoint_publish_flag_))
	{
		ROS_WARN("Please publish setpoint first");
		return false;
	}
	else
	{
		if (set_mode_client_.call(mode) && mode.response.mode_sent)
			;
		else
			ROS_ERROR_STREAM(vehicle_info_.vehicle_name_ << 
			    " failed to call set_mode service. " << mode.response.mode_sent);
		return mode.response.mode_sent;
	}
}
```

드론의 모드를 변경하기 위해 사용되는 인터페이스. 입력인자로 들어온 문자열을 기반으로 드론한테 모드변경을 요청한다. 사용 가능한 모드들은 아래 콜백함수 페이지에서 stateCB()에 기록되어 있다. offboard 모드는 자주 사용되기 때문에 대소문자를 가리지 않기 위해 처리되어 있지만, **타겟지점이 설정되지 않은 상태에서 offboard 모드로 변경되는 것을 방지하기  위해 setpoint\_pulish\_flag\_변수의 값을 확인하는 것을 알 수 있다.** &#x20;

{% content-ref url="callback.md" %}
[callback.md](callback.md)
{% endcontent-ref %}

## bool takeoff(const double &)

```cpp
bool Vehicle::takeoff(const double &takeoff_alt)
{
	mavros_msgs::CommandTOL msg;
	msg.request.min_pitch = 0;
	msg.request.yaw = 0;
	msg.request.latitude = cur_global_.latitude;
	msg.request.longitude = cur_global_.longitude;
	msg.request.altitude = home_global_.altitude + takeoff_alt;

	if (takeoff_client_.call(msg) && msg.response.success)
		ROS_INFO_STREAM(msg.response.result);
	else
		ROS_ERROR_STREAM(vehicle_info_.vehicle_name_ << 
		    " failed to call takeoff service. " << msg.response.result);
	return msg.response.success;
}
```

일반적으로 드론이라 하면 고정익과 회전익 모두를 지원한다. 따라서 고정익의 이착륙에 필요한 고도 때문에  min\_pitch에 대한 변수값을 요구하는 것을 알 수있다. 쿼드로터는 수직이착륙을 하기 때문에 피치값은 0으로 고정되고 고도값이 주어지면 현재 GPS의 위치에서 이륙한다.&#x20;

{% hint style="info" %}
GPS 값을 통해 현재 위치에서 이륙하기 때문에 GPS가 없을 때 이 내부 이륙 함수를 사용할 수 없다.
{% endhint %}

## bool land()

```cpp
bool Vehicle::land()
{
	mavros_msgs::CommandTOL msg;
	msg.request.min_pitch = 0;
	msg.request.yaw = 0;
	msg.request.latitude = cur_global_.latitude;
	msg.request.longitude = cur_global_.longitude;
	msg.request.altitude = 0;

	if (land_client_.call(msg) && msg.response.success)
		ROS_INFO_STREAM(msg.response.result);
	else
		ROS_ERROR_STREAM(vehicle_info_.vehicle_name_ << " failed to call land service. " << msg.response.result);
	return msg.response.success;
}
```

이륙과 마찬가지로 min\_pitch 값을 요구하는 것을 알 수 있다. GPS 값을 통해 들어온 현재 좌표를 기준으로 착륙한다.

{% hint style="info" %}
이륙과 마찬가지로 현재 GPS 값을 사용하므로 GPS 값이 없을 때 사용 할 수 없다.&#x20;
{% endhint %}

## void gotoGlobal(const sensor\_msgs::NavSatFix &)

```cpp
void Vehicle::gotoGlobal(const sensor_msgs::NavSatFix &tar_global)
{
	setpoint_publish_flag_ = true;

	if ((tar_global_.latitude != tar_global.latitude) ||
		(tar_global_.longitude != tar_global.longitude) ||
		(tar_global_.altitude != tar_global.altitude))
		ROS_INFO("%s set target_global_pos(long : %lf, lati : %lf, alti : %lf)", vehicle_info_.vehicle_name_.c_str(),
				 tar_global_.longitude, tar_global_.latitude, tar_global_.altitude);

	tar_global_ = tar_global;
	tar_global_.header.seq += 1;
	tar_global_.header.stamp = ros::Time::now();
	tar_global_.header.frame_id = vehicle_info_.vehicle_name_;
다
	mavros_msgs::GlobalPositionTarget msg;
	msg.latitude = tar_global_.latitude;
	msg.longitude = tar_global_.longitude;
	msg.altitude = tar_global_.altitude;
	msg.header.seq += 1;
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = vehicle_info_.vehicle_name_;
	msg.coordinate_frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_INT;
	msg.type_mask = mavros_msgs::GlobalPositionTarget::IGNORE_AFX |
					mavros_msgs::GlobalPositionTarget::IGNORE_AFY |
					mavros_msgs::GlobalPositionTarget::IGNORE_AFZ |
					mavros_msgs::GlobalPositionTarget::IGNORE_VX |
					mavros_msgs::GlobalPositionTarget::IGNORE_VY |
					mavros_msgs::GlobalPositionTarget::IGNORE_VZ |
					mavros_msgs::GlobalPositionTarget::IGNORE_YAW |
					mavros_msgs::GlobalPositionTarget::IGNORE_YAW_RATE;

	setpoint_global_pub_.publish(msg);
}
```

GPS 좌표 입력을 통해 드론을 이동시키는 함수이다. 5번째 줄은 입력좌표점이 다를 때 목적지를 출력해주는 것이다. 24번째 줄의 타입마스크는 setpoint\_raw에서 사용하는 것과 동일한데, GPS 좌표로 이동시에는 가속도, 속도, 위치, yaw 제어 등을 지원한다. 그 중에서 여기서는 가속도, 속도, yaw 제어를 무시하도록 구현했다.

{% hint style="info" %}
현재는 사용하고 있지 않은 기능이다. GPS 좌표의 경우 로컬좌표와 달리 원하는 좌표를 직접적으로 주기 불편하기 때문이다. 또한 실제 펌웨어에 구현 또한 GPS 좌표 입력시 내부 로컬 좌표로 변환하여 이동한다.
{% endhint %}

{% hint style="success" %}
이 함수의 경우 추후 GUI 환경과 연계시 구글맵 등의 지도정보에서 GPS값을 추출하여 드론을 그곳으로 이동시킬 때 사용 될 수 있다.
{% endhint %}

## void setLocalTarget(const geometry\_msgs::PoseStamped &)

```cpp
void Vehicle::setLocalTarget(const geometry_msgs::PoseStamped &tar_local)
{
	setpoint_publish_flag_ = true;

	if ((tar_local_.pose.position.x != tar_local.pose.position.x) ||
		(tar_local_.pose.position.y != tar_local.pose.position.y) ||
		(tar_local_.pose.position.z != tar_local.pose.position.z))
		tar_local_ = tar_local;
}
```

gotoGlobal과 달리 타겟지점을 지정하는 것과 제어하는 부분이 나뉘어 구현되어 있다. 이유는 제어하는 부분이 현재 브랜치에서는 위치제어 밖에 없지만 다른 브랜치에서 속도제어를 하기 때문이다.&#x20;

{% hint style="info" %}
이 함수는 gotoLocal()과 다른브랜치의 gotoVel()과 함께 사용된다.&#x20;
{% endhint %}

## void gotoLocal()

```cpp
void Vehicle::gotoLocal()
{
	double yaw;
	geometry_msgs::PoseStamped setpoint;
	setpoint.header.seq += 1;
	setpoint.header.stamp = ros::Time::now();
	setpoint.header.frame_id = vehicle_info_.vehicle_name_;
	//현재 로컬포지션값의 쿼터니언을 받아와서 RPY로 변환
	tf::Quaternion cur_q(
		cur_local_.pose.orientation.x,
		cur_local_.pose.orientation.y,
		cur_local_.pose.orientation.z,
		cur_local_.pose.orientation.w
	);
	tf::Matrix3x3 cur_m(cur_q);
	cur_m.getRPY( roll_, pitch_, yaw );
	
	//yaw값만 arming걸때 값으로 돌려주면 됨.
	tf::Matrix3x3 tar_m;
	tar_m.setEulerYPR( arming_yaw_ + turned_yaw_, pitch_, roll_ );
	//  std::cout << "arming yaw: "<<arming_yaw_*180/M_PI<<std::endl;
	//  std::cout << "cur yaw: "<<yaw_*180/M_PI<<std::endl;
	tf::Quaternion tar_q;
	tar_m.getRotation(tar_q);
	setpoint.pose.orientation.w = tar_q.getW();
	setpoint.pose.orientation.x = tar_q.getX();
	setpoint.pose.orientation.y = tar_q.getY();
	setpoint.pose.orientation.z = tar_q.getZ();
	
	setpoint.pose.position.x = setpoint_pos_.getX();
	setpoint.pose.position.y = setpoint_pos_.getY();
	setpoint.pose.position.z = setpoint_pos_.getZ();

	setpoint_local_pub_.publish(setpoint);
}
```

드론의 제어를 위해 사용되는 함수이다. 9번째 줄부터 28번째 줄까지는 현재의 roll, pitch, yaw를 구하고 초기 yaw값으로 대체하여 다시 쿼터니언값을 구해 넣어주는 부분이다. 드론은 일반적으로 자세를 표현하는데에 roll, pitch, yaw를 사용하는데 ROS에서는 쿼터니언을 사용한다. 때문에 쿼터니언을 RPY로 변환하고 다시 쿼터니언으로 변환하는 복잡한 과정을 거친다.

내부 변수에 기록된 `setpoint_pos_`는 `tf2::Vector3` 타입의 변수이므로 값을 불러와서 `geometry_msgs::PoseStamped` 타입의 `setpoint`로 변환하여 전송한다.&#x20;

{% hint style="success" %}
여기서 사용되는 setpoint\_pos\_는 바로 위에서 사용되는 target\_local\_과 다르다. Target\_local은 드론의 최종목표지점이고, setpoint\_pos는 타겟지점까지 가기위한 경로계획을 위해 목표지점까지의 경로를 잘게 쪼개서 드론에서 가장 가까운 위치가 setpoint\_pos가 된다.
{% endhint %}

{% hint style="info" %}
추후에는 쿼터니언에서 RPY 로 변환하고 다시 쿼터니언으로 변환하는 것을 쿼터니언을 리턴값으로 가지는 함수로 변환하는 것이 좋을 것 같다.&#x20;
{% endhint %}
