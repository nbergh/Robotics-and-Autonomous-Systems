#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <math.h>
#include <iostream>

#define CONTROLLER_UPDATE_RATE 10

class Callback_class {
private:
	sensor_msgs::LaserScan laser_scan_msg;
    nav_msgs::Path current_path;
	double pos_x_robot, pos_y_robot, orientation_z_robot;
    bool new_path_set,arm_is_in_use;
	
public:
	Callback_class() :
		pos_x_robot(0),
		pos_y_robot(0),
		orientation_z_robot(0),
        new_path_set(0),
        arm_is_in_use(0)
			{}

    void path_callback(const nav_msgs::Path::ConstPtr& msg) {
        current_path = (*msg);
        new_path_set = true;
	}

	void laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
		laser_scan_msg = (*msg);
	}

    void position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        double siny = + 2.0 * ((*msg).pose.orientation.w * (*msg).pose.orientation.z + (*msg).pose.orientation.x * (*msg).pose.orientation.y);
        double cosy = + 1.0 - 2.0 * ((*msg).pose.orientation.y * (*msg).pose.orientation.y + (*msg).pose.orientation.z * (*msg).pose.orientation.z);
		orientation_z_robot = std::atan2(siny, cosy);

        pos_x_robot = (*msg).pose.position.x;
        pos_y_robot = (*msg).pose.position.y;
	}

    void is_arm_in_use_callback(const std_msgs::Int8::ConstPtr& msg){
        if (msg->data > 0){arm_is_in_use = false;}
        else {arm_is_in_use = true;}
    }

	sensor_msgs::LaserScan& get_laser_scan() {
		return laser_scan_msg;
	}

    nav_msgs::Path& get_path() {
        return current_path;
    }

    bool is_new_path_set() {
        return new_path_set;
    }

    void reset_new_path_flag() {
        new_path_set = false;
	}

	void get_position(double& pos_x_robot_arg, double& pos_y_robot_arg, double& orientation_z_robot_arg) {
		orientation_z_robot_arg = orientation_z_robot;
		pos_x_robot_arg = pos_x_robot;
		pos_y_robot_arg = pos_y_robot;
	}

    bool is_arm_in_use() {
        return arm_is_in_use;
    }

};

bool stop_for_obstacle(sensor_msgs::LaserScan laser_scan) {
	bool stop_for_obstacle_bool = false;
	
	int nr_of_points = (int) ((laser_scan.angle_max - laser_scan.angle_min) / laser_scan.angle_increment);
	double obstacle_angle_min = -M_PI/15, obstacle_angle_max = M_PI/15, obstacle_distance = 0.2;

	obstacle_angle_max += M_PI/2;
	obstacle_angle_min += M_PI/2;

	// Print laser scan data
	/*
	std::cout << std::endl
			<< "Laser scan data: " << std::endl
			<< "Min range: " << laser_scan.range_min << std::endl
			<< "Max range: " << laser_scan.range_max << std::endl
			<< "Min angle: " << laser_scan.angle_min << std::endl
			<< "Max angle: " << laser_scan.angle_max << std::endl
			<< "Angle increment: " << laser_scan.angle_increment << std::endl
			<< "Number of points: " << nr_of_points << std::endl
			<< "Obstacle angle min: " << obstacle_angle_min << std::endl
			<< "Obstacle angle max: " << obstacle_angle_max << std::endl;
	*/

	double current_angle = laser_scan.angle_min;
	for (int i=0;i<nr_of_points;i++) {		
		
		if (current_angle > obstacle_angle_min && current_angle < obstacle_angle_max) {
			//std::cout << "Range point " << current_angle <<": " << laser_scan.ranges[i] << std::endl;

			if (laser_scan.ranges[i] < laser_scan.range_max && laser_scan.ranges[i] > laser_scan.range_min && laser_scan.ranges[i] < obstacle_distance) {					
				stop_for_obstacle_bool = true;
			}
		}
		
		current_angle += laser_scan.angle_increment;
	}
	
	return stop_for_obstacle_bool;
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "ras_g9_navigation_node");

	ros::NodeHandle n;

	Callback_class callback_object;

	// Create subscribers and publishers:	

    ros::Subscriber path_sub = n.subscribe("/whole_path",1,&Callback_class::path_callback,&callback_object);
    ros::Subscriber position_sub = n.subscribe("/robot_position",1,&Callback_class::position_callback,&callback_object);
    ros::Subscriber laser_scan_sub = n.subscribe("/scan",1,&Callback_class::laser_scan_callback,&callback_object);
    ros::Subscriber arm_is_in_use_sub = n.subscribe("pickupFinished",1,&Callback_class::is_arm_in_use_callback,&callback_object);

    ros::Publisher goal_finished_pub = n.advertise<std_msgs::Int16>("/goal_finished", 1);
	ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);

    double pos_x_robot = 0, pos_y_robot = 0, orientation_z_robot = 0;
    bool following_path = false, stop_for_obstacle_bool = false, rotate_to_orientation = false;
    nav_msgs::Path current_path;
    int current_position_in_path = 0;

	ros::Rate r(CONTROLLER_UPDATE_RATE);
	while(n.ok()) {
		ros::spinOnce(); // Check for incoming messages

        geometry_msgs::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = 0;
        cmd_vel_msg.linear.y = 0;
        cmd_vel_msg.linear.z = 0;
        cmd_vel_msg.angular.x = 0;
        cmd_vel_msg.angular.y = 0;
        cmd_vel_msg.angular.z = 0;
		
		callback_object.get_position(pos_x_robot, pos_y_robot,orientation_z_robot);

        if (callback_object.is_new_path_set()) {
            current_path = callback_object.get_path();
            callback_object.reset_new_path_flag();
            current_position_in_path = 0;
            rotate_to_orientation = false;
            following_path = true;
        }

        if (following_path) {
            double cmd_vel_orientation = 0, cmd_vel_linear = 0, lateral_error = 0, orientation_error = 0;

            if (!rotate_to_orientation) {
                double pos_behind_x = current_path.poses[current_position_in_path].pose.position.x;
                double pos_behind_y = current_path.poses[current_position_in_path].pose.position.y;
                double pos_ahead_x = current_path.poses[current_position_in_path+1].pose.position.x;
                double pos_ahead_y = current_path.poses[current_position_in_path+1].pose.position.y;

                double path_orientation = std::atan2(pos_ahead_y - pos_behind_y, pos_ahead_x - pos_behind_x);

                double pos_x_robot_rotated = (pos_x_robot-pos_behind_x) * std::cos(path_orientation) + (pos_y_robot-pos_behind_y) * std::sin(path_orientation);
                double pos_y_robot_rotated = -(pos_x_robot-pos_behind_x) * std::sin(path_orientation) + (pos_y_robot-pos_behind_y) * std::cos(path_orientation);
                double pos_x_goal_rotated = (pos_ahead_x-pos_behind_x) * std::cos(path_orientation) + (pos_ahead_y-pos_behind_y) * std::sin(path_orientation);

                if (pos_x_robot_rotated > pos_x_goal_rotated) {
                    current_position_in_path++;

                    int path_length = current_path.poses.size();
                    if (current_position_in_path == path_length -1) {
                        // Goal reached
                        std::cout 	<< std::endl
                                    << "Goal reached" << std::endl
                                    << "Distance from goal x: " << pos_ahead_x - pos_x_robot << std::endl
                                    << "Distance from goal y: " << pos_ahead_x - pos_y_robot << std::endl;

                        rotate_to_orientation = true;
                    }
                    continue;
                }

                orientation_error = path_orientation - orientation_z_robot;
                if (orientation_error > M_PI) {orientation_error -= 2*M_PI;}
                if (orientation_error < -M_PI) {orientation_error += 2*M_PI;}

                lateral_error = 0 - pos_y_robot_rotated;

                cmd_vel_orientation = 2 * lateral_error + 0.7 * orientation_error;
                cmd_vel_linear = 0.1;

                if (std::abs(orientation_error) > 0.2 && ( (orientation_error > 0 && lateral_error > 0) || (orientation_error < 0 && lateral_error < 0) ) ) {cmd_vel_linear = 0;}

            } else {
                int path_length = current_path.poses.size();
                double siny = + 2.0 * (current_path.poses[path_length-1].pose.orientation.w * current_path.poses[path_length-1].pose.orientation.z + current_path.poses[path_length-1].pose.orientation.x * current_path.poses[path_length-1].pose.orientation.y);
                double cosy = + 1.0 - 2.0 * (current_path.poses[path_length-1].pose.orientation.y * current_path.poses[path_length-1].pose.orientation.y + current_path.poses[path_length-1].pose.orientation.z * current_path.poses[path_length-1].pose.orientation.z);
                double desired_orientation = std::atan2(siny, cosy);

                orientation_error = desired_orientation - orientation_z_robot;
                if (orientation_error > M_PI) {orientation_error -= 2*M_PI;}
                if (orientation_error < -M_PI) {orientation_error += 2*M_PI;}

                if (std::abs(orientation_error) < 0.05) {
                    std::cout   << std::endl
                                << "Rotated to orientation: " << desired_orientation << std::endl
                                << "Orientation error: " << orientation_error << std::endl;

                    rotate_to_orientation = false;
                    following_path = false;
                    continue;
                }

                cmd_vel_orientation = 5 * orientation_error;
            }

            // Check for obstacles:
            stop_for_obstacle_bool = stop_for_obstacle(callback_object.get_laser_scan());
            if (stop_for_obstacle_bool && !callback_object.is_arm_in_use()) {
               cmd_vel_linear = -0.1;
               cmd_vel_orientation = 0;
            }

            cmd_vel_msg.linear.x = cmd_vel_linear;
            cmd_vel_msg.angular.z = cmd_vel_orientation;

            // Print data:
            std::cout 	<< std::endl
                        << "Following path: " << following_path << std::endl
                        << "Rotating to orientation: " << rotate_to_orientation << std::endl
                        << "Stopped for obstacle: " << stop_for_obstacle_bool << std::endl
						<< "Path length: " << current_path.poses.size() << std::endl
                        << "Position in path: " << current_position_in_path << std::endl
                        << "Orientation error: " << orientation_error << std::endl
                        << "Lateral error: " << lateral_error << std::endl
                        << "Robot position x: " << pos_x_robot << std::endl
                        << "Robot position y: " << pos_y_robot << std::endl
                        << "Robot orientation z: " << orientation_z_robot << std::endl						
                        << "Linear vel cmd: " << cmd_vel_msg.linear.x << std::endl
                        << "Linear vel angular: " << cmd_vel_msg.angular.z << std::endl;
        }

        std_msgs::Int16 goal_finished;
        if (following_path) {
            goal_finished.data = 0;
        } else {
            goal_finished.data = 1;
        }
        goal_finished_pub.publish(goal_finished);

        cmd_vel_pub.publish(cmd_vel_msg);
		r.sleep();
	}
}



















