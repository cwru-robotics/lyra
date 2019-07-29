#include <rclcpp/rclcpp.hpp>
#include <lyra_msg/msg/hand_motion.hpp>
#include <lyra_msg/msg/hand_contact.hpp>

#include <thread>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

//TODO Make this command-line parametrizable.
#define TARGET_IP "192.168.42.129"
#define POSITION_PORT 5225
#define SENSATION_PORT 5226

void thread_for_recieving(
	const int socket,
	const struct sockaddr_in & add_in,
	const std::shared_ptr<rclcpp::Node> & node,
	const std::shared_ptr<rclcpp::Publisher<lyra_msg::msg::HandMotion> > & pub,
	const bool v
){
	float buffer [6];
	while(rclcpp::ok()){
		unsigned int len;
		recvfrom(
			socket,
			(char *)buffer,
			6 * sizeof(float),  
                	MSG_WAITALL,
                	(struct sockaddr *) (& add_in),
                	&len
                );
                
                if(v){
                	RCLCPP_INFO(node->get_logger(), "Publishing hand motion %f %f %f %f %f %f",
                		buffer[0], buffer[1], buffer[2],
                		buffer[3], buffer[4], buffer[5]
                	);
                }
                
                //TODO figure out how to populate the time in the headers.
                lyra_msg::msg::HandMotion m_out;
                
                m_out.wrist_rot = buffer[0];
		m_out.wrist_flex = buffer[1];
		m_out.thumb_a = buffer[2];
		m_out.thumb_b = buffer[3];
		m_out.index = buffer[4];
		m_out.mrl = buffer[5];
		
		pub->publish(m_out);
	}
}


int publisher_socket;
struct sockaddr_in * servaddr;
std::shared_ptr<rclcpp::Node> n_extern;
bool verbose;


void CB_sense_msg(const lyra_msg::msg::HandContact::SharedPtr msg){
	float serialized [11];
	serialized[0] = msg->palm_front;
	serialized[1] = msg->palm_back;
	serialized[2] = msg->palm_inside;
	serialized[3] = msg->palm_outside;
	serialized[4] = msg->thumb;
	serialized[5] = msg->thumb_side;
	serialized[6] = msg->index;
	serialized[7] = msg->index_side;
	serialized[8] = msg->middle;
	serialized[9] = msg->ring;
	serialized[10] = msg->little;
	
	if(verbose){
                RCLCPP_INFO(n_extern->get_logger(), "Forwarding sensory motion %f %f %f\n%f %f %f\n%f %f %f\n%f",
                	serialized[0], serialized[1], serialized[2],
                	serialized[3], serialized[4], serialized[5],
                	serialized[6], serialized[7], serialized[8],
                	serialized[9], serialized[10]
		);
        }
	
	sendto(
		publisher_socket,
		(const char *)serialized,
		11 * sizeof(float),
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
	rclcpp::init(argc, argv);
	auto nh = rclcpp::Node::make_shared("lyra");
	
	
	//Set up motion transmission (to ROS) publisher
	auto motion_ros_publisher =  nh->create_publisher<lyra_msg::msg::HandMotion>("/hand_motion", 1);
	
	
	//Set up motion reception (from UDP) subscriber
	int motion_udp_reciever = socket(AF_INET, SOCK_DGRAM, 0);
	if(motion_udp_reciever == 0){
		RCLCPP_ERROR(nh->get_logger(), "Could not create motion reception socket.");
		return 0;
	}
	int opt = 1;
	int errorcode = setsockopt(motion_udp_reciever, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));
	if (errorcode){ 
		RCLCPP_ERROR(nh->get_logger(), "Socket configuration error: %d", errorcode);
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
		RCLCPP_ERROR(nh->get_logger(), "Socket binding error: %d", errorcode);
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
		RCLCPP_ERROR(nh->get_logger(), "Error creating position transmitter socket: %d", sens_transmit_socket);
		return 0;
	}
	struct sockaddr_in sens_transmit_addr;
	sens_transmit_addr.sin_family = AF_INET;
	sens_transmit_addr.sin_port = SENSATION_PORT;
	int ecode = inet_pton(AF_INET, TARGET_IP, &sens_transmit_addr.sin_addr);
	if(ecode <=0){ 
		RCLCPP_ERROR(nh->get_logger(), "Invalid address/ Address not supported: %s, %d", TARGET_IP, ecode);
		return 0;
	} 
	//ecode = connect(sens_transmit_socket, (struct sockaddr *)&sens_transmit_addr, sizeof(sens_transmit_addr));
	if (ecode < 0){ 
		RCLCPP_ERROR(nh->get_logger(), "Connection to sensory computer failed: %d", ecode);
		return 0;
	}
	
	
	//Set up sensory reception (from ROS) subscriber
	auto subscription = nh->create_subscription<lyra_msg::msg::HandContact>("/hand_contact", 1, CB_sense_msg);
	
	rclcpp::spin(nh);
	
	//RCLCPP_ERROR(nh->get_logger(), "Ran successfully.");

	return 0;
}
