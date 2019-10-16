//#include <algorithm>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <tf/tf.h>
#include <leap_motion/Human.h>
#include <trajectory_msgs/JointTrajectory.h>

//TODO Make these parametrizable ROS args.
#define SLEEP_INTERVAL 1.0

/*#define ARM_TILT_MIN_DRONE -1.5
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
#define V_DRONE_DEADZONE 0.02*/

ros::Time stop_since_time;
ros::Publisher arm_tilt_publisher;
ros::Publisher drone_publisher;

bool are_we_recieving;

void checker_CB(const leap_motion::Human::ConstPtr& msg){
	if(are_we_recieving && !(msg->left_hand.is_present)){
		printf("\b\b\b\b\b\b\b");
		printf("\e[31m██████ ");
		fflush(stdout); 
		are_we_recieving = false;
		return;
	}
	if(!are_we_recieving && msg->left_hand.is_present){
		printf("\b\b\b\b\b\b\b");
		printf("\e[32m██████ ");
		fflush(stdout); 
		are_we_recieving = true;
		return;
	}
}

void get_calib_point(ros::NodeHandle & nh, ros::AsyncSpinner & spinny, const char * location, double * output){
	printf("\tMove hand to the %s center of your comfortable reaching range and press enter: \e[32m██████ ", location);
	fflush(stdout); 
	
	spinny.start();
	while(std::getchar() && ros::ok()){
		if(are_we_recieving){
			leap_motion::Human tmp = *(
				ros::topic::waitForMessage<leap_motion::Human>(
					"/leap_motion/leap_device", nh
				)
			);
			output[0] = tmp.left_hand.palm_center.x;
			output[1] = tmp.left_hand.palm_center.y;
			output[2] = tmp.left_hand.palm_center.z;
			break;
		} else{
			printf("\b\b\b\b\b\b\b");
		}
	}
	spinny.stop();
	printf("\e[39m\t%s location acquired.\n", location);
}

int lr_ind, ud_ind, fb_ind;

double l_bound, r_bound, u_bound, d_bound, f_bound, b_bound;

void calibration_procedure(ros::NodeHandle & nh){
	//double zero_position [3];
	double front	[3];
	double back	[3];
	double left	[3];
	double right	[3];
	double up	[3];
	double down	[3];
	
	ros::Subscriber calsub = nh.subscribe("/leap_motion/leap_device", 10, checker_CB);
	
	ros::AsyncSpinner spinny(1);
	
	printf("Calibration in progress...\n");
	get_calib_point(nh, spinny, "TOP", up);
	get_calib_point(nh, spinny, "BOTTOM", down);
	
	get_calib_point(nh, spinny, "FRONT", front);
	get_calib_point(nh, spinny, "BACK", back);
	
	get_calib_point(nh, spinny, "LEFT", left);
	get_calib_point(nh, spinny, "RIGHT", right);
	
	double max_diff = 0.0;
	for(int i = 0; i < 3; i++){
		if(abs(front[i] - back[i]) > max_diff){
			max_diff = abs(front[i] - back[i]);
			fb_ind = i;
		}
	}
	
	max_diff = 0.0;
	for(int i = 0; i < 3; i++){
		if(abs(up[i] - down[i]) > max_diff){
			max_diff = abs(up[i] - down[i]);
			ud_ind = i;
		}
	}
	
	max_diff = 0.0;
	for(int i = 0; i < 3; i++){
		if(abs(left[i] - right[i]) > max_diff){
			max_diff = abs(left[i] - right[i]);
			lr_ind = i;
		}
	}
	
	/*ROS_ERROR(
		"Front-back index is %d,\
		 top-bottom index is %d,\
		  left-right index is %d",
		fb_ind,
		ud_ind,
		lr_ind
	);*/
	assert(lr_ind != ud_ind);
	assert(lr_ind != fb_ind);
	assert(ud_ind != fb_ind);
	
	l_bound = left	[lr_ind];
	r_bound = right	[lr_ind];
	u_bound = up	[ud_ind];
	d_bound = down	[ud_ind];
	f_bound = front	[fb_ind];
	b_bound = back	[fb_ind];
	
	printf("Calibration complete!\n");
}
	
#define A_MAX -1.5
#define A_MIN 0.0
#define A_DED 0.0

#define V_MAX 0.5
#define V_MIN -0.5
#define V_DED 0.2

#define W_MAX 1.0
#define W_MIN -1.0
#define W_DED 0.1

double flatten(
	const double input,
	const double hi_bound,
	const double lo_bound,
	const double hi_output,
	const double lo_output,
	const double deadzone
){
	double normalized = (input - lo_bound) / (hi_bound - lo_bound);
	if(normalized < 0.0){
		normalized = 0.0;
	} else if(normalized > 1.0){
		normalized = 1.0;
	}
	
	if(abs(2.0 * (normalized - 0.5)) < deadzone){
		return NAN;
	}
	
	return (normalized * (hi_output - lo_output)) + lo_output;
}

void leap_CB(const leap_motion::Human::ConstPtr& msg){
	//If we lose the arm, wait a little while and then bring it back.
	if(!(msg->left_hand.is_present)){
		if(ros::Time::now() > stop_since_time){
			ROS_WARN("Lost arm target.");
		}
		stop_since_time = ros::Time::now() + ros::Duration(SLEEP_INTERVAL);
		//When we lose the hand we should stop moving.
		geometry_msgs::Twist drone_twist;
		drone_twist.linear.x = 0.0;
		drone_twist.angular.z = 0.0;
		drone_publisher.publish(drone_twist);
		return;
	}
	if(ros::Time::now() < stop_since_time){
		return;
	}
	
	double incoming [3] = {
		msg->left_hand.palm_center.x,
		msg->left_hand.palm_center.y,
		msg->left_hand.palm_center.z
	};
	
	double a = flatten(
		incoming[ud_ind],
		u_bound, d_bound,
		A_MAX, A_MIN, A_DED
	);
	
	double v = flatten(
		incoming[fb_ind],
		f_bound, b_bound,
		V_MAX, V_MIN, V_DED
	);
	
	double w = flatten(
		incoming[lr_ind],
		l_bound, r_bound,
		W_MAX, W_MIN, W_DED
	);
	
	ROS_INFO("A = %f, V = %f, W = %f", a, v, w);
	
	if(!isnan(a)){
		trajectory_msgs::JointTrajectory arm_trajectory_out;
		trajectory_msgs::JointTrajectoryPoint arm_point;
		arm_point.positions = {a};
		arm_point.time_from_start = ros::Duration(0.1);
		arm_trajectory_out.points = {arm_point};
		arm_trajectory_out.joint_names = {"arm_joint"};
	
		arm_tilt_publisher.publish(arm_trajectory_out);
	}
	
	geometry_msgs::Twist drone_twist;
	drone_twist.linear.x = 0.0;
	drone_twist.angular.z = 0.0;
	if(!isnan(v)){
		drone_twist.linear.x = v;
	}
	if(!isnan(w)){
		drone_twist.angular.z = w;
	}
	drone_publisher.publish(drone_twist);
	
	//Tilt the arm.
	/*double arm_tilt_raw = (
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
	
	drone_publisher.publish(drone_twist);*/
	
	//ROS_INFO("%f, %f, %f -> %f", msg->left_hand.roll, msg->left_hand.pitch, msg->left_hand.yaw, arm_tilt_raw);
	//ROS_INFO("%f, %f, %f -> %f", msg->left_hand.palm_center.x, msg->left_hand.palm_center.y, msg->left_hand.palm_center.z, arm_tilt_raw);
}

int main(int argc, char ** argv){
	
	//ROS initialization
	ros::init(argc, argv, "lyra_aion");
	ros::NodeHandle nh;
	
	are_we_recieving = true;
	
	calibration_procedure(nh);
	
	//Set up motion transmission (to drone) publishers
	arm_tilt_publisher = nh.advertise<trajectory_msgs::JointTrajectory>("/arm/joint_traj_controller/command", 1);
	drone_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
	
	//Set up sensory reception (from ROS) subscriber
	ros::Subscriber sub = nh.subscribe("/leap_motion/leap_device", 10, leap_CB);
	
	stop_since_time = ros::Time::now();
	
	ros::spin();
	return 0;
}
