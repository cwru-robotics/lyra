#include <ros/ros.h>
//#include <lyra_msg/msg/hand_motion.hpp>
//#include <lyra_msg/msg/hand_contact.hpp>
#include <open_loop_control/MoveAction.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <gazebo_msgs/ApplyBodyWrench.h>

#include <thread>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

//TODO Make this command-line parametrizable.
#define TARGET_IP "192.168.42.129"
#define POSITION_PORT 5025
#define SENSATION_PORT 5226

ros::Time last_time;
double accum_a;
double accum_b;
double accum_c;

void thread_for_recieving(
	const int socket,
	const struct sockaddr_in & add_in,
	const ros::NodeHandle & node,
	const ros::Publisher & pub,
	const bool v
){
	char buffer [100];
	while(ros::ok()){
		unsigned int len;
		recvfrom(
			socket,
			(char *)buffer,
			100 * sizeof(char),  
                	MSG_WAITALL,
                	(struct sockaddr *) (& add_in),
                	&len
                );
                
                if(last_time.sec == -1){
                	last_time = ros::Time::now();
                }
                ros::Duration dt = ros::Time::now() - last_time;
                
                //RCLCPP_INFO(node->get_logger(), "Got raw result %s", buffer);
                double result[7];
                sscanf(
                	buffer,
                	"[%lf, %lf, %lf, %lf, %lf, %lf, %lf]",
                	&result[0], &result[1], &result[2],
                	&result[3], &result[4], &result[5], &result[6]
                );
                
                if(v){
                	ROS_INFO("Publishing hand motion %f %f %f %f %f %f",
                		result[0], result[1], result[2],
                		result[3], result[4], result[5]
                	);
                }
                
                double dt_factor = 0.00157 * ((double)dt.sec + 1e-9 * (double)dt.nsec);
                
                /*accum_a += dt_factor * (result[0]);
                accum_b += dt_factor * (result[1]);
                accum_c += dt_factor * (result[2]);
                
                //TODO figure out how to populate the time in the headers.
                geometry_msgs::Pose poser;
                
                //m_out.name = {"wristx", "wristz"};
                poser.position.y = accum_a;
                poser.position.z = accum_b;
                
                poser.orientation.w = accum_c;
                
                open_loop_control::MoveGoal mover;
                mover.pose = poser;
		
		pub.publish(mover);
                */
                
                accum_a = 5.0 * result[0];
                accum_b = 40.0 * result[1];
                accum_c = 40.0 * abs(result[2]);
                
                gazebo_msgs::ApplyBodyWrench srv;
                srv.request.body_name="changeling::base_link";
                srv.request.reference_frame = "changeling::base_link";
                
                srv.request.duration = ros::Duration(0.2);
                
                srv.request.wrench.torque.z = accum_a;
                srv.request.wrench.force.y = accum_b;
                srv.request.wrench.force.z = accum_c;
                
                ros::service::call("/gazebo/apply_body_wrench", srv);
	}
}


int publisher_socket;
struct sockaddr_in * servaddr;
ros::NodeHandle * n_extern;
bool verbose;


void CB_sense_msg(const std_msgs::String::ConstPtr& msg){
	float serialized [18];
	
	for(int i = 0; i < 18; i++){
		serialized[i] = std::stod(msg->data.substr(i * 3, i * 3 + 3));
	}
	
	if(verbose){
                ROS_INFO("Forwarding sensory motion %f %f %f\n%f %f %f\n%f %f %f\n%f %f %f\n%f %f %f\n%f %f %f",
                	serialized[0], serialized[1], serialized[2],
                	serialized[3], serialized[4], serialized[5],
                	serialized[6], serialized[7], serialized[8],
                	serialized[9], serialized[10], serialized[11],
                	serialized[12], serialized[13], serialized[14],
                	serialized[15], serialized[16], serialized[17]
		);
        }
	
	sendto(
		publisher_socket,
		(const char *)serialized,
		18 * sizeof(float),
		MSG_CONFIRM,
		(const struct sockaddr *) servaddr,  
		sizeof(&servaddr)
	); 
}

int main(int argc, char ** argv){
	verbose = false;
	
	if(argc == 1){
		printf("Hand-sensor ROS adapter. \n\t-v: Print messages as they are sent and recieved.\n");
	}

	for(int i = 0; i < argc; i++){
		if(!(strcmp(argv[i], "-v") && strcmp(argv[i], "-V"))){
			verbose = true;
		}
	}
	
	//ROS initialization
	ros::init(argc, argv, "lyra_driver");
	ros::NodeHandle nh;
	
	
	//Set up motion transmission (to ROS) publisher
	ros::Publisher motion_ros_publisher;// =  nh.advertise<open_loop_control::MoveGoal>("/control_server/goal", 1);
	
	
	//Set up motion reception (from UDP) subscriber
	int motion_udp_reciever = socket(AF_INET, SOCK_DGRAM, 0);
	if(motion_udp_reciever == 0){
		ROS_ERROR("Could not create motion reception socket.");
		return 0;
	}
	int opt = 1;
	int errorcode = setsockopt(motion_udp_reciever, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));
	if (errorcode){ 
		ROS_ERROR("Socket configuration error: %d", errorcode);
		return 0;
	}
	struct sockaddr_in motion_udp_reciever_address; 
	motion_udp_reciever_address.sin_family = AF_INET; 
	motion_udp_reciever_address.sin_addr.s_addr = INADDR_ANY; 
	motion_udp_reciever_address.sin_port = htons(POSITION_PORT);
	errorcode = bind(
		motion_udp_reciever,
		(struct sockaddr *) & motion_udp_reciever_address,
		sizeof(motion_udp_reciever_address)
	);
	if (errorcode < 0){ 
		ROS_ERROR("Socket binding error: %d", errorcode);
		return 0;
	}
	std::thread recthread(
		thread_for_recieving,
		motion_udp_reciever,
		motion_udp_reciever_address,
		nh,
		motion_ros_publisher,
		verbose
	);
	recthread.detach();
	
	
	//Set up sensory transmission (to udp) socket
	int sens_transmit_socket = socket(AF_INET, SOCK_DGRAM, 0);
	if(sens_transmit_socket < 0){
		ROS_ERROR("Error creating position transmitter socket: %d", sens_transmit_socket);
		return 0;
	}
	struct sockaddr_in sens_transmit_addr;
	sens_transmit_addr.sin_family = AF_INET;
	sens_transmit_addr.sin_port = SENSATION_PORT;
	int ecode = inet_pton(AF_INET, TARGET_IP, &sens_transmit_addr.sin_addr);
	if(ecode <=0){ 
		ROS_ERROR("Invalid address/ Address not supported: %s, %d", TARGET_IP, ecode);
		return 0;
	} 
	//ecode = connect(sens_transmit_socket, (struct sockaddr *)&sens_transmit_addr, sizeof(sens_transmit_addr));
	if (ecode < 0){ 
		ROS_ERROR("Connection to sensory computer failed: %d", ecode);
		return 0;
	}
	
	last_time.sec = -1;
	accum_a = 0.0;
	accum_b = 0.0;
	accum_c = 0.0;
	
	//Set up sensory reception (from ROS) subscriber
	ros::Subscriber sub = nh.subscribe("/hand_feedback", 10, CB_sense_msg);
	
	ros::spin();
	
	//RCLCPP_ERROR(nh->get_logger(), "Ran successfully.");

	return 0;
}
