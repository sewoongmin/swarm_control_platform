# Service Callback 함수

## bool multiSetpointLocal(swarm\_ctrl\_pkg::srvMultiSetpointLocal::Request \&req, swarm\_ctrl\_pkg::srvMultiSetpointLocal::Response \&res)

```cpp
bool SwarmVehicle::multiSetpointLocal(
	swarm_ctrl_pkg::srvMultiSetpointLocal::Request &req,
	swarm_ctrl_pkg::srvMultiSetpointLocal::Response &res)
{
	updateOffset();

	formation_ = req.formation;
	swarm_target_local_.setX(req.x);
	swarm_target_local_.setY(req.y);
	swarm_target_local_.setZ(req.z);

	target_changed_flag_ = true;

	res.success = true;
	return res.success;
}
```

## bool multiSetpointGlobal(swarm\_ctrl\_pkg::srvMultiSetpointGlobal::Request \&req, swarm\_ctrl\_pkg::srvMultiSetpointGlobal::Response \&res)

```cpp
bool SwarmVehicle::multiSetpointGlobal(
	swarm_ctrl_pkg::srvMultiSetpointGlobal::Request &req,
	swarm_ctrl_pkg::srvMultiSetpointGlobal::Response &res)
{
	res.success = true;
	return res.success;
}
```

## bool gotoVehicle(swarm\_ctrl\_pkg::srvGoToVehicle::Request \&req, swarm\_ctrl\_pkg::srvGoToVehicle::Response \&res)

```cpp
bool SwarmVehicle::gotoVehicle(swarm_ctrl_pkg::srvGoToVehicle::Request &req,
							   swarm_ctrl_pkg::srvGoToVehicle::Response &res)
{
	geometry_msgs::PoseStamped msg;
	iter_ = camila_.begin() + req.num_drone - 1;

	updateOffset();

	if (req.num_drone > 0 && req.num_drone <= num_of_vehicle_)
	{
		msg.header.stamp = ros::Time::now();
		msg.pose.position.x = req.x + offset_[req.num_drone - 1].getX();
		msg.pose.position.y = req.y + offset_[req.num_drone - 1].getY();
		msg.pose.position.z = req.z;
	}

	iter_->setLocalTarget(msg);

	return true;
}
```

## void triggerCB(const std\_msgs::Empty::ConstPtr &)

```cpp
void SwarmVehicle::triggerCB(const std_msgs::Empty::ConstPtr &trigger)
{
	std_msgs::Bool arm;
	std_msgs::String mode;

	trigger_arm_ = nh_.advertise<std_msgs::Bool>("/multi/arming", 10);
	trigger_mode_ = nh_.advertise<std_msgs::String>("/multi/set_mode", 10);

	arm.data = true;
	// mode.data = "auto.takeoff";
	mode.data = "offboard";

	trigger_arm_.publish(arm);
	trigger_mode_.publish(mode);
}
```
