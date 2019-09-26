//#include <algorithm>

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include<tf/tf.h>
#include <trajectory_msgs/JointTrajectory.h>

//TODO Make these parametrizable ROS args.
#define SLEEP_INTERVAL 1.0

#define ARM_TILT_MIN_DRONE -1.5
#define ARM_TILT_MAX_DRONE 0.0
#define ARM_TILT_MIN_POLARIS -2.0
#define ARM_TILT_MAX_POLARIS 0.0

#define OMEGA_MIN_DRONE -1.5
#define OMEGA_MAX_DRONE 1.5
#define OMEGA_MIN_POLARIS -0.6
#define OMEGA_MAX_POLARIS 0.2

#define V_MIN_DRONE -0.1
#define V_MAX_DRONE 0.1
#define V_MIN_POLARIS 0.0
#define V_MAX_POLARIS -0.15

ros::Time stop_since_time;
ros::Publisher arm_tilt_publisher;

void polaris_CB(const geometry_msgs::PoseArray::ConstPtr& msg){
	//If we lose the arm, wait a little while and then bring it back.
	if(isnan(msg->poses[0].position.x)){
		if(ros::Time::now() > stop_since_time){
			ROS_WARN("Lost arm target.");
		}
		stop_since_time = ros::Time::now() + ros::Duration(SLEEP_INTERVAL);
		return;
	}
	if(ros::Time::now() < stop_since_time){
		return;
	}
	
	//Convert from quaternion to RPY
	double roll, pitch, yaw;
	tf::Quaternion q;
	quaternionMsgToTF(msg->poses[0].orientation, q);
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	
	//Tilt the arm.
	double arm_tilt_raw = (
		(std::min(
			std::max(pitch,
			ARM_TILT_MIN_POLARIS),
			ARM_TILT_MAX_POLARIS
		) - ARM_TILT_MIN_POLARIS) / (ARM_TILT_MAX_POLARIS - ARM_TILT_MIN_POLARIS)
	) * (ARM_TILT_MAX_DRONE - ARM_TILT_MIN_DRONE) + ARM_TILT_MIN_DRONE;
	
	trajectory_msgs::JointTrajectory arm_trajectory_out;
	trajectory_msgs::JointTrajectoryPoint arm_point;
	arm_point.positions = {arm_tilt_raw};
	arm_point.time_from_start = ros::Duration(0.5);
	arm_trajectory_out.points = {arm_point};
	arm_trajectory_out.joint_names = {"arm_joint"};
	
	arm_tilt_publisher.publish(arm_trajectory_out);
	ROS_INFO("%f, %f, %f -> %f", roll, pitch, yaw, arm_tilt_raw);
}

int main(int argc, char ** argv){
	
	//ROS initialization
	ros::init(argc, argv, "lyra_aion");
	ros::NodeHandle nh;
	
	//Set up motion transmission (to drone) publishers
	arm_tilt_publisher = nh.advertise<trajectory_msgs::JointTrajectory>("/arm/joint_traj_controller/command", 1);
	
	//Set up sensory reception (from ROS) subscriber
	ros::Subscriber sub = nh.subscribe("/polaris_sensor/targets", 10, polaris_CB);
	
	stop_since_time = ros::Time::now();
	
	ros::spin();
	return 0;
}
