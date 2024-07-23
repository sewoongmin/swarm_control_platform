---
description: 한대 이상의 드론을 제어하기 위한 메인 제어 인터페이스
---

# 메인 제어 인터페이스

## Private 제어 인터페이스

### void updateOffset()

```cpp
void SwarmVehicle::updateOffset()
{
	sensor_msgs::NavSatFix leader = camila_.front().getHomeGlobal();

	int i = 0;
	if (offset_.size() == 0)
	{
		if (num_of_vehicle_ != 1)
			angle_ = 2.0 * M_PI / (num_of_vehicle_ - 1);
		for (auto &vehicle : camila_)
		{
			sensor_msgs::NavSatFix follower = vehicle.getGlobalPosition();
			tf2::Vector3 _offset = convertGeoToENU(leader, follower);
			offset_.push_back(_offset);
			ROS_INFO_STREAM("offset_[" << i << "] = " << offset_[i].getX() 
				<< ", " << offset_[i].getY() << ", " << offset_[i].getZ());
			i++;
		}
	}
}
```

### void limit(tf2::Vector3 &, const double &)

```cpp
void SwarmVehicle::limit(tf2::Vector3 &v, const double &limit)
{
	if (v.length() > limit)
		v = v.normalize() * limit;
}
```

### void getVehiclePos()

```cpp
void SwarmVehicle::getVehiclePos()
{
	sensor_msgs::NavSatFix origin = camila_.front().getHomeGlobal();
	for (auto &vehicle : camila_)
	{
		tf2::Vector3 vehicle_pos = 
			convertGeoToENU(vehicle.getGlobalPosition(), origin);
		vehicle.setPos(vehicle_pos);
	}
}
```

### void separate(Vehicle &)

```cpp
void SwarmVehicle::separate(Vehicle &vehicle)
{
	tf2::Vector3 sum(0, 0, 0);
	unsigned int cnt = 0;

	for (auto &another_vehicle : camila_)
	{
		if (&vehicle != &another_vehicle)
		{
			tf2::Vector3 diff = vehicle.getPos() - another_vehicle.getPos();
			double dist = diff.length();
			if (dist < range_sp_)
			{
				if (diff.length() != 0)
					diff = diff.normalize();
				diff *= (range_sp_ / dist);
				sum += diff;
				cnt++;
			}
		}
	}
	if (cnt > 0)
	{
		sum /= cnt;
		limit(sum, vector_speed_limit_);
		vehicle.setSumOfSp(sum);
	}
	else
	{
		sum.setZero();
		vehicle.setSumOfSp(sum);
	}
}
```

### void seek(Vehicle &)

```cpp
void SwarmVehicle::seek(Vehicle &vehicle)
{
	tf2::Vector3 err(0, 0, 0);
	geometry_msgs::PoseStamped target_pos = vehicle.getTargetLocal();
	geometry_msgs::PoseStamped current_pos = vehicle.getLocalPosition();
	err.setX(target_pos.pose.position.x - current_pos.pose.position.x);
	err.setY(target_pos.pose.position.y - current_pos.pose.position.y);
	err.setZ(target_pos.pose.position.z - current_pos.pose.position.z);

	limit(err, vector_speed_limit_);
	vehicle.setErr(err);
}
```

### void formationGenerator()

```cpp
void SwarmVehicle::formationGenerator()
{
	geometry_msgs::PoseStamped msg, msg_f;
	double m_sec = ros::Time::now().toNSec() / 1000000;
	double x = 0.00006 * m_sec; // 0.2 rad/s
	double angle, spacing;
	nh_global_.getParamCached("spacing", spacing);

	int i = 0;
	for (auto &vehicle : camila_)
	{
		msg.pose.position.x = swarm_target_local_.getX()*cos(vehicle.getArmingYaw()) 
			- swarm_target_local_.getY()*sin(vehicle.getArmingYaw());
		msg.pose.position.y = swarm_target_local_.getX()*sin(vehicle.getArmingYaw()) 
			+ swarm_target_local_.getY()*cos(vehicle.getArmingYaw());
		msg.pose.position.z = swarm_target_local_.getZ();
		i++;
	}

	if (formation_ == "POINT")
	{
		int i = 0;
		for (auto &vehicle : camila_)
		{
			if (vehicle.getInfo().vehicle_id_ != 1)
			{
				msg_f.pose.position.x = msg.pose.position.x + offset_[i].getX();
				msg_f.pose.position.y = msg.pose.position.y + offset_[i].getY();
				msg_f.pose.position.z = msg.pose.position.z;
				vehicle.setLocalTarget(msg_f);
			}
			else
				vehicle.setLocalTarget(msg);
			i++;
		}
	}
	else if (formation_ == "IDLE")
	{
		int i = 0;
		int x = 0 , y = -3;
		for (auto &vehicle : camila_)
		{
			if(y < 16)
				y += 3;
			else{
				x += 3;
				y = 0;
			}
			msg_f.pose.position.x = msg.pose.position.x + offset_[i].getX() + x;
			msg_f.pose.position.y = msg.pose.position.y + offset_[i].getY() + y;
			msg_f.pose.position.z = msg.pose.position.z;
			vehicle.setLocalTarget(msg_f);
			i++;
		}
	}
	else if (formation_ == "SCEN1")
	{
		int i = 0;
		for (auto &vehicle : camila_)
		{
			if (vehicle.getInfo().vehicle_id_ != 1)
			{
				angle = i * angle_;

				msg_f.pose.position.x = msg.pose.position.x + offset_[i].getX() 
					+ spacing * cos(angle + x);
				msg_f.pose.position.y = msg.pose.position.y + offset_[i].getY() 
					+ spacing * sin(angle + x);
				msg_f.pose.position.z = msg.pose.position.z;
				vehicle.setLocalTarget(msg_f);
			}
			else
				vehicle.setLocalTarget(msg);
			i++;
		}
	}
}
```

## Public 제어 인터페이스

### void run()

```cpp
void SwarmVehicle::run()
{
	if (isPublish())
	{
		bool control_method, sp;
		double final_speed_limit;
		nh_global_.getParamCached("use_vel", control_method);
		nh_global_.getParamCached("setpoint/kp_seek", kp_seek_);
		nh_global_.getParamCached("setpoint/kp_sp", kp_sp_);
		nh_global_.getParamCached("setpoint/range_sp", range_sp_);
		nh_global_.getParamCached("setpoint/vector_speed_limit", vector_speed_limit_);
		nh_global_.getParamCached("setpoint/final_speed_limit", final_speed_limit);
		nh_global_.getParamCached("setpoint/separate", sp);
		
		getVehiclePos();
		for (auto &vehicle : camila_)
		{
			tf2::Vector3 setpoint;
			seek(vehicle);
			if (sp)
			{
				// separate(vehicle);
				setpoint = vehicle.getSumOfSp() * kp_sp_ 
					+ vehicle.getErr() * kp_seek_;
			}
			else
				setpoint = vehicle.getErr() * kp_seek_;
			limit(setpoint, final_speed_limit);
			vehicle.setSetpointPos(setpoint);
			if (control_method)
				vehicle.gotoVel();
			else
				vehicle.gotoLocal();
		}
	}
	formationGenerator();
}
```
