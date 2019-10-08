//#include <algorithm>

#include <ros/ros.h>
#include <tf/tf.h>
#include <leap_motion/Human.h>
#include <trajectory_msgs/JointTrajectory.h>

//TODO Make these parametrizable ROS args.
#define SLEEP_INTERVAL 1.0

#define ARM_TILT_MIN_DRONE -1.5
#define ARM_TILT_MAX_DRONE 0.0
#define ARM_TILT_MIN_LEAP -0.3
#define ARM_TILT_MAX_LEAP 1.3

#define OMEGA_MIN_DRONE -1.5
#define OMEGA_MAX_DRONE 1.5
#define OMEGA_MIN_LEAP -0.5
#define OMEGA_MAX_LEAP 1.0

#define V_MIN_DRONE -0.1
#define V_MAX_DRONE 0.1
#define V_MIN_LEAP -0.15
#define V_MAX_LEAP 0.2
#define V_DRONE_DEADZONE 0.02

ros::Time stop_since_time;
ros::Publisher arm_tilt_publisher;
ros::Publisher drone_publisher;

void leap_CB(const leap_motion::Human::ConstPtr& msg){
	//If we lose the arm, wait a little while and then bring it back.
	if(!(msg->left_hand.is_present)){
		if(ros::Time::now() > stop_since_time){
			ROS_WARN("Lost arm target.");
		}
		stop_since_time = ros::Time::now() + ros::Duration(SLEEP_INTERVAL);
		return;
	}
	if(ros::Time::now() < stop_since_time){
		return;
	}
	
	/*//Convert from quaternion to RPY
	double roll, pitch, yaw;
	tf::Quaternion q;
	quaternionMsgToTF(msg->poses[0].orientation, q);
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);*/
	
	//Tilt the arm.
	double arm_tilt_raw = (
		(std::min(
			std::max((double)(msg->left_hand.pitch),
			ARM_TILT_MIN_LEAP),
			ARM_TILT_MAX_LEAP
		) - ARM_TILT_MIN_LEAP) / (ARM_TILT_MAX_LEAP - ARM_TILT_MIN_LEAP)
	) * (ARM_TILT_MAX_DRONE - ARM_TILT_MIN_DRONE) + ARM_TILT_MIN_DRONE;
	
	trajectory_msgs::JointTrajectory arm_trajectory_out;
	trajectory_msgs::JointTrajectoryPoint arm_point;
	arm_point.positions = {arm_tilt_raw};
	arm_point.time_from_start = ros::Duration(0.5);
	arm_trajectory_out.points = {arm_point};
	arm_trajectory_out.joint_names = {"arm_joint"};
	
	arm_tilt_publisher.publish(arm_trajectory_out);
	
	geometry_msgs::Twist drone_twist;
	
	//Spin the drone
	double drone_spin_raw = (
		(std::min(
			std::max((double)(msg->left_hand.yaw),
			OMEGA_MIN_LEAP),
			OMEGA_MAX_LEAP
		) - OMEGA_MIN_LEAP) / (OMEGA_MAX_LEAP - OMEGA_MIN_LEAP)
	) * (OMEGA_MAX_DRONE - OMEGA_MIN_DRONE) + OMEGA_MIN_DRONE;
	
	drone_twist.angular.z = drone_spin_raw;
	
	
	double drone_vel_raw = (
		(std::min(
			std::max((double)(msg->left_hand.palm_center.z),
			V_MIN_LEAP),
			V_MAX_LEAP
		) - V_MIN_LEAP) / (V_MAX_LEAP - V_MIN_LEAP)
	) * (V_MAX_DRONE - V_MIN_DRONE) + V_MIN_DRONE;
	
	if(abs(drone_vel_raw) > V_DRONE_DEADZONE){
		drone_twist.linear.x = drone_spin_raw;
	} else{
		drone_twist.linear.x = 0.0;
	}
	
	drone_publisher.publish(drone_twist);
	
	//ROS_INFO("%f, %f, %f -> %f", msg->left_hand.roll, msg->left_hand.pitch, msg->left_hand.yaw, arm_tilt_raw);
	ROS_INFO("%f, %f, %f -> %f", msg->left_hand.palm_center.x, msg->left_hand.palm_center.y, msg->left_hand.palm_center.z, arm_tilt_raw);
}

int main(int argc, char ** argv){
	
	//ROS initialization
	ros::init(argc, argv, "lyra_aion");
	ros::NodeHandle nh;
	
	//Set up motion transmission (to drone) publishers
	arm_tilt_publisher = nh.advertise<trajectory_msgs::JointTrajectory>("/arm/joint_traj_controller/command", 1);
	drone_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
	
	//Set up sensory reception (from ROS) subscriber
	ros::Subscriber sub = nh.subscribe("/leap_motion/leap_device", 10, leap_CB);
	
	stop_since_time = ros::Time::now();
	
	ros::spin();
	return 0;
}
