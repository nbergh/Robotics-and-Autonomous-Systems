#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <math.h>
#include <iostream>
#include <sstream>

#include "phidgets/motor_encoder.h"

#define CONTROLLER_UPDATE_RATE 10.0 // [Hz]
#define ALPHA 0.5
#define BETA 1.0 // 1.0
#define LEFT_MOTOR_SERIAL 465084
#define RIGHT_MOTOR_SERIAL 465093
#define WHEEL_RADIUS 0.0352
#define WHEEL_BASE 0.2
#define TICKS_PER_REV 850.0
#define MAX_PWM 40
#define MAX_ANGULAR_SPEED 0.5 // [rad/s]

class Callback_class {
private:
	double encoder_ticks_left, encoder_ticks_right, encoder_ticks_left_last_requested, encoder_ticks_right_last_requested;
	double cmd_vel_linear, cmd_vel_angular; // Velocity command state [m/s] [rad/s]
	
public:
	Callback_class() :
		encoder_ticks_left(0),
		encoder_ticks_right(0),
		encoder_ticks_left_last_requested(0),
		encoder_ticks_right_last_requested(0),
		cmd_vel_linear(0),
		cmd_vel_angular(0) {}

	void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg) {
		cmd_vel_linear = (*msg).linear.x;
		cmd_vel_angular = (*msg).angular.z;
	}

	void encoder_left_motor_callback(const phidgets::motor_encoder::ConstPtr& msg) {	
		encoder_ticks_left = (*msg).count;
	}	

	void encoder_right_motor_callback(const phidgets::motor_encoder::ConstPtr& msg) {
		encoder_ticks_right = (*msg).count;
	}

	void get_encoder_ticks(double& encoder_ticks_left_arg, double& encoder_ticks_right_arg) {
		encoder_ticks_left_arg = encoder_ticks_left - encoder_ticks_left_last_requested;
		encoder_ticks_right_arg = encoder_ticks_right - encoder_ticks_right_last_requested;
		encoder_ticks_left_last_requested = encoder_ticks_left;
		encoder_ticks_right_last_requested = encoder_ticks_right;
	}
		
	void get_cmd_vel(double& cmd_vel_linear_arg, double& cmd_vel_angular_arg) {
		cmd_vel_linear_arg = cmd_vel_linear;
		cmd_vel_angular_arg = cmd_vel_angular;
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "ras_g9_motor_controller_node");

	ros::NodeHandle n;

	Callback_class callback_object;

	// Create subscribers and publishers:	

	std::stringstream pwm_left_motor_topic, pwm_right_motor_topic, encoder_left_motor_topic, encoder_right_motor_topic; 

	pwm_left_motor_topic << "/motorcontrol/cmd_vel/" << LEFT_MOTOR_SERIAL;
	pwm_right_motor_topic << "/motorcontrol/cmd_vel/" << RIGHT_MOTOR_SERIAL;
	encoder_left_motor_topic << "/motorcontrol/encoder/" << LEFT_MOTOR_SERIAL;
	encoder_right_motor_topic << "/motorcontrol/encoder/" << RIGHT_MOTOR_SERIAL;

	ros::Subscriber cmd_vel_sub = n.subscribe("/cmd_vel",1,&Callback_class::cmd_vel_callback,&callback_object);
	ros::Subscriber encoder_left_motor_sub = n.subscribe(encoder_left_motor_topic.str(),1,&Callback_class::encoder_left_motor_callback,&callback_object);
	ros::Subscriber encoder_right_motor_sub = n.subscribe(encoder_right_motor_topic.str(),1,&Callback_class::encoder_right_motor_callback,&callback_object);
	
	ros::Publisher pwm_left_motor_pub = n.advertise<std_msgs::Float32>(pwm_left_motor_topic.str(),1);
	ros::Publisher pwm_right_motor_pub = n.advertise<std_msgs::Float32>(pwm_right_motor_topic.str(),1);
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

	tf::TransformBroadcaster odom_broadcaster;

	double pos_x = 0, pos_y = 0, orientation_z = 0, vel_x = 0, vel_y = 0, linear_vel = 0, angular_vel = 0; // [m] [m] [rad] [m/s] [m/s] [m/s] [rad/s]. Robot pos and vel state
    double w_error_left_accu = 0, w_error_right_accu = 0;

	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	//int debug_ticks_left=0, debug_ticks_right=0;

	ros::Rate r(CONTROLLER_UPDATE_RATE);
	while(n.ok()) {
		ros::spinOnce(); // Check for incoming messages

		current_time = ros::Time::now();

		double dt = (current_time - last_time).toSec();	

		double encoder_ticks_left = 0,  encoder_ticks_right = 0;
		callback_object.get_encoder_ticks(encoder_ticks_left,encoder_ticks_right);
		
		//debug_ticks_left += encoder_ticks_left;
		//debug_ticks_right += encoder_ticks_right;
		//std::cout << "Debug ticks: " << debug_ticks_left << " " << debug_ticks_right << std::endl;		

		double w_left = (encoder_ticks_left*2*M_PI/TICKS_PER_REV) / dt; // [rad/s]
		double w_right = -(encoder_ticks_right*2*M_PI/TICKS_PER_REV) / dt; // [rad/s]

		linear_vel = (w_left + w_right) * WHEEL_RADIUS / 2.0;
    	angular_vel = (w_right - w_left) * WHEEL_RADIUS / WHEEL_BASE;

		orientation_z += angular_vel * dt;	

		vel_x = linear_vel*std::cos(orientation_z);
		vel_y = linear_vel*std::sin(orientation_z);

		pos_x += vel_x * dt;
		pos_y += vel_y * dt;

	//Publish odometry (see: http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom)
		tf::Quaternion odom_quat(0,0,0,0);
		odom_quat.setEuler(0,0,orientation_z);

		tf::Vector3 odom_vector(pos_x,pos_y,0.0);		
		
		/*
		// Send TF transform:
		odom_broadcaster.sendTransform(
			tf::StampedTransform(
				tf::Transform(odom_quat,odom_vector),
				current_time,
				"map",
				"base_link"
			)		  
		);
		*/

		// Publish odometry:
		nav_msgs::Odometry odom_msg;
		odom_msg.header.stamp = current_time;
		odom_msg.header.frame_id = "map";

		odom_msg.pose.pose.position.x = pos_x;
		odom_msg.pose.pose.position.y = pos_y;
		odom_msg.pose.pose.position.z = 0.0;
		odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(orientation_z);

		odom_msg.child_frame_id = "base_link";
		odom_msg.twist.twist.linear.x = vel_x;
		odom_msg.twist.twist.linear.y = vel_y;
		odom_msg.twist.twist.angular.z = angular_vel;

		odom_pub.publish(odom_msg);

	// Update PWM control:

		double linear_vel_desired = 0, angular_vel_desired = 0;
		callback_object.get_cmd_vel(linear_vel_desired,angular_vel_desired);

		if (angular_vel_desired > MAX_ANGULAR_SPEED) {angular_vel_desired = MAX_ANGULAR_SPEED;}
		if (angular_vel_desired < -MAX_ANGULAR_SPEED) {angular_vel_desired = -MAX_ANGULAR_SPEED;}

		double w_left_desired = (linear_vel_desired - (WHEEL_BASE/2) * angular_vel_desired) / WHEEL_RADIUS; // [rad/s]
    	double w_right_desired = (linear_vel_desired + (WHEEL_BASE/2) * angular_vel_desired) / WHEEL_RADIUS; // [rad/s]

		double w_error_left = w_left_desired - w_left;
		double w_error_right = w_right_desired - w_right;

        w_error_left_accu += w_error_left;
        w_error_right_accu += w_error_right;

		std_msgs::Float32 msg_pwm_left,msg_pwm_right;
        msg_pwm_left.data = (int) (w_error_left * ALPHA + w_error_left_accu * BETA);
        msg_pwm_right.data = (int) -(w_error_right * ALPHA + w_error_right_accu * BETA);
		
		if (msg_pwm_left.data > MAX_PWM ) {msg_pwm_left.data = MAX_PWM;}
		if (msg_pwm_left.data < -MAX_PWM ) {msg_pwm_left.data = -MAX_PWM;}
		if (msg_pwm_right.data > MAX_PWM ) {msg_pwm_right.data = MAX_PWM;}
		if (msg_pwm_right.data < -MAX_PWM ) {msg_pwm_right.data = -MAX_PWM;}
		
		if (w_left_desired == 0) {msg_pwm_left.data = 0;}		
		if (w_right_desired == 0) {msg_pwm_right.data = 0;}

		pwm_left_motor_pub.publish(msg_pwm_left);
		pwm_right_motor_pub.publish(msg_pwm_right);

	// Print data:
		std::cout << std::endl
		<< "Delta time: " << dt << std::endl
		<< "Encoder ticks left: "<< encoder_ticks_left << std::endl
		<< "Encoder ticks right: " << encoder_ticks_right << std::endl
		<< "W left: " << w_left << std::endl
		<< "W right: " << w_right << std::endl
		<< "Linear velocity: " << linear_vel << std::endl
		<< "Angular velocity: " << angular_vel << std::endl
		<< "Orientation z: " << orientation_z << std::endl
		<< "Position x: " << pos_x << std::endl
		<< "Position y: " << pos_y << std::endl
		<< "Linear velocity desired: " << linear_vel_desired << std::endl 
		<< "Angular velocity desired: " << angular_vel_desired << std::endl
		<< "W left desired: " << w_left_desired << std::endl
		<< "W right desired: " << w_right_desired << std::endl		
		<< "W error left: " << w_error_left << std::endl
		<< "W error right: " << w_error_right << std::endl
		<< "W error left accu: " << w_error_left_accu << std::endl
		<< "W error right accu: " << w_error_right_accu << std::endl
		<< "PWM left: " << msg_pwm_left.data << std::endl
		<< "PWM right: " << msg_pwm_right.data << std::endl;


		last_time = current_time;
		r.sleep();
	}
}

