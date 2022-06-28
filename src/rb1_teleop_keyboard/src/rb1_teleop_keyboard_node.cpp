#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/thread.hpp>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_i 0x69
#define KEYCODE_I 0x49

#define KEYCODE_j 0x6a
#define KEYCODE_J 0x4a

#define KEYCODE_k 0x6b
#define KEYCODE_K 0x4b

#define KEYCODE_l 0x6c
#define KEYCODE_L 0x4c

#define KEYCODE_comma 0x2c
#define KEYCODE_COMMA 0x3c

#define KEYCODE_z 0x7A
#define KEYCODE_Z 0x5A

#define KEYCODE_q 0x71
#define KEYCODE_Q 0x51

#define KEYCODE_u 0x75
#define KEYCODE_U 0x55

#define KEYCODE_o 0x6F
#define KEYCODE_O 0x4F


#define KEYCODE_SPACE 0x20

#define ANGLE_VEL_INC 5.0 //1.0
#define TR_VEL_INC    0.05 //0.01


boost::recursive_mutex m_guard;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class TeleopTurtle {
public:
	TeleopTurtle();
	void keyLoop();

	ros::NodeHandle nh_;
	double linear_, angular_, l_scale_, a_scale_;
        bool pub_cmd_vel_;

	bool use_action_client_; 
	ros::Publisher twist_pub_;

};

TeleopTurtle::TeleopTurtle() :
		 nh_("~"), linear_(0), angular_(0), l_scale_(1.0), a_scale_(1.0), pub_cmd_vel_(false), use_action_client_(false) {
	nh_.param("scale_angular", a_scale_, a_scale_);
	nh_.param("scale_linear", l_scale_, l_scale_);
	
	twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig) {
	(void) sig;
	tcsetattr(kfd, TCSANOW, &cooked);
	ros::shutdown();
	exit(0);
}

void controlThread(ros::Rate rate, TeleopTurtle* robot)
{
  geometry_msgs::Twist twist;
  while (1)
  {
	{
 		boost::lock_guard<boost::recursive_mutex> lock(m_guard);       	
		 		
		if(robot->pub_cmd_vel_ == true)
		{		
			twist.angular.z = (M_PI / 180.0) * robot->a_scale_ * robot->angular_;
			twist.linear.x  = robot->l_scale_ * robot->linear_;
			robot->twist_pub_.publish(twist);
		}
        }
	rate.sleep();
  }
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "teleop_rb1");
	TeleopTurtle teleop_turtle;

	signal(SIGINT, quit);

	
	
	boost::thread(boost::bind(controlThread, ros::Rate(50), &teleop_turtle));
	teleop_turtle.keyLoop();

	return (0);
}

void TeleopTurtle::keyLoop() {
	char c;
	bool dirty = false;

	// get the console in raw mode
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &= ~(ICANON | ECHO);
	// Setting a new line, then end of file
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);

	puts("Reading from keyboard");
	puts("---------------------------");
	puts("Use arrow keys to move the rb1.");

	linear_ = angular_ = 0;
        pub_cmd_vel_ = false;

	MoveBaseClient ac("move_base", true);
	
	nh_.param("use_action", use_action_client_, use_action_client_);
	if(use_action_client_ == true)
	{
		ROS_INFO("Use action client lib. to cancel goal");
	}
	
	if(use_action_client_ == true)
	{
		//wait for the action server to come up
		while(!ac.waitForServer(ros::Duration(5.0))){
			ROS_INFO("Waiting for the move_base action server to come up");
		}
	}

	ros::Duration d(0.1, 0);
	for (;;) {
		// get the next event from the keyboard
		if (read(kfd, &c, 1) < 0) {
			perror("read():");
			exit(-1);
		}

		ROS_INFO("value: %c, 0x%02X\n", c, c);
		{
			boost::lock_guard<boost::recursive_mutex> lock(m_guard);       	
 					
			switch (c) {
			case KEYCODE_q: //q
				ROS_INFO("Stop Current Goal");
				ac.cancelAllGoals(); 
				break;
			case KEYCODE_Q: //Q
				ROS_INFO("Stop Current Goal");
				if(use_action_client_ == true)
				{
					ac.cancelAllGoals();
				}  
				break;
			case KEYCODE_z: //Z
				ROS_INFO("publish cmd_vel : %s", (pub_cmd_vel_ == true) ? "false" : "true");
				pub_cmd_vel_ = (pub_cmd_vel_ == true) ? false : true;
				angular_ = 0.0;
				linear_  = 0.0;
				dirty = true;
				break;
			case KEYCODE_Z: //Z
				ROS_INFO("publish cmd_vel : %s", (pub_cmd_vel_ == true) ? "false" : "true");
				pub_cmd_vel_ = (pub_cmd_vel_ == true) ? false : true;
				angular_ = 0.0;
				linear_  = 0.0;
				dirty = true;
				break;
			case KEYCODE_SPACE: //space
				ROS_DEBUG("SPACE");
				angular_ = 0.0;
				linear_  = 0.0;
				dirty = true;
				break;
			case KEYCODE_k: //space
				ROS_DEBUG("STOP");
				angular_ = 0.0;
				linear_  = 0.0;
				dirty = true;
				break;
			case KEYCODE_K: //space
				ROS_DEBUG("STOP");
				angular_ = 0.0;
				linear_  = 0.0;
				dirty = true;
				break;
			case KEYCODE_u: //u
				ROS_DEBUG("Trans. Vel. STOP");
				linear_  = 0.0;
				dirty = true;
				break;
			case KEYCODE_U: //U
				ROS_DEBUG("Trans. Vel. STOP");
				linear_  = 0.0;
				dirty = true;
				break;
			case KEYCODE_o: //o
				ROS_DEBUG("Rot. Vel. STOP");
				angular_ = 0.0;
				dirty = true;
				break;
			case KEYCODE_O: //O
				ROS_DEBUG("Rot. Vel. STOP");
				angular_ = 0.0;
				dirty = true;
				break;
			case KEYCODE_j:
				ROS_DEBUG("LEFT");
				angular_ += ANGLE_VEL_INC;
				dirty = true;
				break;
			case KEYCODE_J:
				ROS_DEBUG("LEFT");
				angular_ += ANGLE_VEL_INC;
				dirty = true;
				break;
			case KEYCODE_l:
				ROS_DEBUG("RIGHT");
				angular_ -= ANGLE_VEL_INC;
				dirty = true;
				break;
			case KEYCODE_L:
				ROS_DEBUG("RIGHT");
				angular_ -= ANGLE_VEL_INC;
				dirty = true;
				break;
			case KEYCODE_i:
				ROS_DEBUG("UP");
				linear_ += TR_VEL_INC;
				dirty = true;
				break;
			case KEYCODE_I:
				ROS_DEBUG("UP");
				linear_ += TR_VEL_INC;
				dirty = true;
				break;
			case KEYCODE_comma:
				ROS_DEBUG("DOWN");
				linear_ -= TR_VEL_INC;
				dirty = true;
				break;
			case KEYCODE_COMMA:
				ROS_DEBUG("DOWN");
				linear_ -= TR_VEL_INC;
				dirty = true;
				break;
			}
		}
		ROS_INFO("velocity (%.2lf, %.1lf)", linear_, angular_);

	}

	return;
}

