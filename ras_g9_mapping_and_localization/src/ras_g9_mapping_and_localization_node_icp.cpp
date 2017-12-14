#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <math.h>
#include <iostream>
#include <fstream>
#include <sstream>

#define MAP_AND_POSITION_UPDATE_RATE 10.0 //[Hz]
#define OBSTACLE_INFLATION_RADIUS 0.16 //[m]
#define NR_OF_LINES 5
#define RANSAC_ITERATIONS 20
#define INLIER_THRESHOLD 0.02 //[m]
#define MAP_FILE_PATH "/home/ras19/catkin_ws/src/ras_g9/ras_g9_mapping_and_localization/map/map.txt"
//#define MAP_FILE_PATH "/home/ras/catkin_ws/src/ras_g9/ras_g9_mapping_and_localization/map/map.txt"

class Callback_class {
private:
	sensor_msgs::LaserScan laser_scan_msg;
	double pos_x_robot_odom, pos_y_robot_odom, orientation_z_robot_odom;
	
public:
	Callback_class() :
		pos_x_robot_odom(0),
		pos_y_robot_odom(0),
		orientation_z_robot_odom(0)
			{}

	void laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
		laser_scan_msg = (*msg);
	}

	void odometry_callback(const nav_msgs::Odometry::ConstPtr& msg) {
		double siny = + 2.0 * ((*msg).pose.pose.orientation.w * (*msg).pose.pose.orientation.z + (*msg).pose.pose.orientation.x * (*msg).pose.pose.orientation.y);
		double cosy = + 1.0 - 2.0 * ((*msg).pose.pose.orientation.y * (*msg).pose.pose.orientation.y + (*msg).pose.pose.orientation.z * (*msg).pose.pose.orientation.z);  
		orientation_z_robot_odom = std::atan2(siny, cosy);

		pos_x_robot_odom = (*msg).pose.pose.position.x;
		pos_y_robot_odom = (*msg).pose.pose.position.y;
	}

	sensor_msgs::LaserScan& get_laser_scan() {
		return laser_scan_msg;
	}	


	void get_position_odom(double& pos_x_robot_arg, double& pos_y_robot_arg, double& orientation_z_robot_arg) {
		orientation_z_robot_arg = orientation_z_robot_odom;
		pos_x_robot_arg = pos_x_robot_odom;
		pos_y_robot_arg = pos_y_robot_odom;
	}

};

struct point_xy_t {
	double x,y;
	int id;
};

struct line_t {
    point_xy_t point_a, point_b;
	double r,theta,line_angle,dx,dy;
};

struct line_pair_t {
	std::vector<line_t>::iterator scan_line, map_line;
};

void mark_map(nav_msgs::OccupancyGrid& map_msg, double pos_x, double pos_y) {
    int row = (int) (pos_y / map_msg.info.resolution);
    int col = (int) (pos_x / map_msg.info.resolution);
	
	/*
	std::cout 	<< "Mark map" << std::endl
			<< "X: " << pos_x << std::endl
			<< "Y: " << pos_y << std::endl
			<< "Row: " << row << std::endl
			<< "Col: " << col << std::endl;	
	*/	

	// Mark wall:
	if (row >= 0 && row < map_msg.info.height && col >= 0 && col < map_msg.info.width) {
		int mark_pos_array = row * map_msg.info.width + col;	
		map_msg.data[mark_pos_array] = 100;
	}
	
	// Inflate obstacle
    int inflation_radius = (int) (OBSTACLE_INFLATION_RADIUS / map_msg.info.resolution);
	for (int curr_row = -inflation_radius; curr_row <= inflation_radius; curr_row++) {
		for(int curr_col = -inflation_radius; curr_col <= inflation_radius; curr_col++) {
            if ((curr_row * curr_row + curr_col * curr_col) <= inflation_radius * inflation_radius) {

				if ((row+curr_row) >= 0 && (row+curr_row) < map_msg.info.height && (col+curr_col) >= 0 && (col+curr_col) < map_msg.info.width) {
					int mark_pos_array = (row+curr_row) * map_msg.info.width + (col+curr_col);
					if (map_msg.data[mark_pos_array] == 0) {map_msg.data[mark_pos_array] = 50;}
				}					
			}		
		}
	}
}

void draw_wall_map(nav_msgs::OccupancyGrid& map_msg, double wall_start_x, double wall_start_y, double wall_end_x, double wall_end_y) {
	double dx = wall_end_x - wall_start_x;
	double dy = wall_end_y - wall_start_y;
	double wall_orientation = std::atan(dy / dx);
	if (dx < 0 && dy > 0) {wall_orientation += M_PI;}
	if (dx < 0 && dy < 0) {wall_orientation -= M_PI;}
	if (dx < 0 && dy == 0) {wall_orientation = M_PI;}
	
	/*
	std::cout 	<< "Draw wall" << std::endl
			<< "dx: " << dx << std::endl
			<< "dy: " << dy << std::endl
			<< "wall_orientation: " << wall_orientation << std::endl;
	*/
	
	double current_x = wall_start_x, current_y = wall_start_y;
	while (true) {
		mark_map(map_msg, current_x, current_y);
		current_x += 0.5 * map_msg.info.resolution * std::cos(wall_orientation);	
		current_y += 0.5 * map_msg.info.resolution * std::sin(wall_orientation);	

		double wall_rotated_x = (wall_end_x - wall_start_x) * std::cos(wall_orientation) + (wall_end_y - wall_start_y) * std::sin(wall_orientation);
		double current_rotated_x = (current_x - wall_start_x) * std::cos(wall_orientation) + (current_y - wall_start_y) * std::sin(wall_orientation);
		if (current_rotated_x > wall_rotated_x) {break;}
	}
}

double get_angle_difference_within_90_deg(double angle_1, double angle_2) {
	if (angle_1 > M_PI/2.0) {angle_1 -= M_PI;}
	if (angle_1 < -M_PI/2.0) {angle_1 += M_PI;}
	if (angle_2 > M_PI/2.0) {angle_2 -= M_PI;}
	if (angle_2 < -M_PI/2.0) {angle_2 += M_PI;}

	double diff = angle_2 - angle_1;
	if (diff > M_PI/2.0) {diff -= M_PI;}
	if (diff < -M_PI/2.0) {diff += M_PI;}
	
	return diff;
}

double get_angle_difference_within_180_deg(double angle_1, double angle_2) {
	double diff = angle_2 - angle_1;
	if (diff > M_PI) {diff -= 2*M_PI;}
	if (diff < -M_PI) {diff += 2*M_PI;}
	
	return diff;
}

void translate_laser_scan_xy(std::vector<point_xy_t>& points_xy, const sensor_msgs::LaserScan& laser_scan) {
	if (laser_scan.angle_increment == 0) {return;}

	int nr_of_points = (int) ((laser_scan.angle_max - laser_scan.angle_min) / laser_scan.angle_increment);

	points_xy.clear();
	points_xy.reserve(nr_of_points);

	double current_angle = laser_scan.angle_min;	
	for (int i=0;i<nr_of_points;i++) {
		
		if (laser_scan.ranges[i] < laser_scan.range_max && laser_scan.ranges[i] > laser_scan.range_min) {
			point_xy_t point;

			// TRANSFORM TO ROBOT FRAME
			double angle_offset = -M_PI/2.0;
			double x_offset = 0.065;
			double y_offset = 0;
			
			point.x = laser_scan.ranges[i] * std::cos(current_angle + angle_offset) + x_offset;
			point.y = laser_scan.ranges[i] * std::sin(current_angle + angle_offset) + y_offset;
			point.id = i;		

			points_xy.push_back(point);

		}		
		current_angle += laser_scan.angle_increment;
	}
}
 
line_t create_line_struct(point_xy_t point_a, point_xy_t point_b) {
	/*
	point_xy point_a, point_b;
	double r,theta,line_angle,dx,dy;
	*/	
	
	line_t line;
	line.point_a = point_a;
	line.point_b = point_b;	

	line.dx = point_b.x - point_a.x;
	line.dy = point_b.y - point_a.y;

	if (line.dx == 0) {
		line.line_angle = M_PI/2.0;
		if (point_a.x >= 0) {line.theta = 0;}
		else {line.theta = M_PI;}
	} else {
		line.line_angle = std::atan(line.dy/line.dx);		
		double k = line.dy/line.dx;
		double m = point_a.y - k * point_a.x;

		if (m>=0) {line.theta = M_PI/2.0 + line.line_angle;}
		else {line.theta = -(M_PI/2.0 - line.line_angle);}
	}	

	line.r = std::abs(point_b.x * point_a.y - point_b.y * point_a.x) / std::sqrt((point_b.y - point_a.y)*(point_b.y - point_a.y) + (point_b.x - point_a.x)*(point_b.x - point_a.x));

	return line;
}

void extract_lines(std::vector<line_t>& extracted_lines, std::vector<point_xy_t> points_xy) {	
	extracted_lines.clear();
	extracted_lines.reserve(NR_OF_LINES);

	std::vector<point_xy_t> points_xy_inliers;
	std::vector<point_xy_t> points_xy_inliers_max;

	for (int line_nr = 0; line_nr < NR_OF_LINES; line_nr++) {	
		if (points_xy.size() < 5) {break;}		
		
		point_xy_t point_a_extracted, point_b_extracted;
		
		for (int ransac_iter = 0; ransac_iter < RANSAC_ITERATIONS; ransac_iter++) {	
			int point_a_idx = rand() % points_xy.size();
			int point_b_idx = rand() % points_xy.size();
			while (point_a_idx == point_b_idx) {point_b_idx = rand() % points_xy.size();}

			point_xy_t point_a = points_xy[point_a_idx];
			point_xy_t point_b = points_xy[point_b_idx];

			/*
			std::cout << "point_a [x y, id]: " << point_a.x << " " << point_a.y << " " << point_a.id << std::endl;
			std::cout << "point_b [x y, id]: " << point_b.x << " " << point_b.y << " " << point_b.id << std::endl;
			std::cout << "angle a to b: " << line_angle << std::endl;
			*/
			
			for (std::vector<point_xy_t>::iterator it = points_xy.begin(); it != points_xy.end(); ++it) {
				point_xy_t point_inl = *it;
				
				double distance_to_line = std::abs((point_b.y - point_a.y) * point_inl.x - (point_b.x - point_a.x) * point_inl.y + point_b.x * point_a.y - point_b.y * point_a.x) / std::sqrt((point_b.y - point_a.y)*(point_b.y - point_a.y) + (point_b.x - point_a.x)*(point_b.x - point_a.x));

				if (distance_to_line < INLIER_THRESHOLD) {points_xy_inliers.push_back(point_inl);}
			}

			if (points_xy_inliers.size() > points_xy_inliers_max.size()) {
				points_xy_inliers_max = points_xy_inliers;				
				point_a_extracted = point_a;
				point_b_extracted = point_b;				
			}
			points_xy_inliers.clear();
			points_xy_inliers.reserve(points_xy.size());
		}
		
		if (points_xy_inliers_max.size() > 20) {	
            line_t extracted_line = create_line_struct(point_a_extracted,point_b_extracted);
			extracted_lines.push_back(extracted_line);
		}

		for (std::vector<point_xy_t>::iterator it = points_xy_inliers_max.begin(); it != points_xy_inliers_max.end(); ++it) {
			// Remove inliers from points_xy

			for (std::vector<point_xy_t>::iterator itt = points_xy.begin(); itt != points_xy.end(); ++itt) {	
				if (it->id == itt->id) {points_xy.erase(itt);break;}
			}
		}
				
		points_xy_inliers_max.clear();	
		points_xy_inliers_max.reserve(points_xy.size());
	}
}

void translate_lines(std::vector<line_t>& lines, double x_t, double y_t, double orientation_t) {
	// Translate scan_lines by x_t, y_t, orientation_t
	std::vector<line_t> lines_translated;
	lines_translated.reserve(lines.size());

	for (std::vector<line_t>::iterator it = lines.begin(); it != lines.end(); ++it) {
		line_t line_it = *it;		

		// Take two points {x,y} in the line
		point_xy_t point_a_trans = line_it.point_a;
		point_xy_t point_b_trans = line_it.point_b;

		point_a_trans.x = line_it.point_a.x * std::cos(orientation_t) - line_it.point_a.y * std::sin(orientation_t);
		point_a_trans.y = line_it.point_a.x * std::sin(orientation_t) + line_it.point_a.y * std::cos(orientation_t);

		point_b_trans.x = line_it.point_b.x * std::cos(orientation_t) - line_it.point_b.y * std::sin(orientation_t);
		point_b_trans.y = line_it.point_b.x * std::sin(orientation_t) + line_it.point_b.y * std::cos(orientation_t);
	
		point_a_trans.x += x_t;	
		point_a_trans.y += y_t;
		point_b_trans.x += x_t;
		point_b_trans.y += y_t;

        line_t line_trans = create_line_struct(point_a_trans,point_b_trans);

		lines_translated.push_back(line_trans);
	}
	lines = lines_translated;
}

void translate_points(std::vector<point_xy_t>& points, double d_x, double d_y, double d_angle) {
    for (std::vector<point_xy_t>::iterator it = points.begin(); it != points.end(); ++it) {

        double point_t_x = it->x * std::cos(d_angle) - it->y * std::sin(d_angle);
        double point_t_y = it->x * std::sin(d_angle) + it->y * std::cos(d_angle);

        point_t_x += d_x;
        point_t_y += d_y;

        it->x = point_t_x;
        it->y = point_t_y;
    }
}

void find_closest_wall_in_map(const nav_msgs::OccupancyGrid& map_msg, double pos_x, double pos_y, double within_dist, double& dx_ret, double& dy_ret, bool& found) {
    dx_ret = 0;
    dy_ret = 0;
    found = false;

    int start_col = (int) (pos_x / map_msg.info.resolution);
    int start_row = (int) (pos_y / map_msg.info.resolution);
    int within_dist_map = (int) (within_dist / map_msg.info.resolution);
    double min_dist = 2 * within_dist_map;

    //std::cout << "find closest in map: x y map x map y: " << pos_x << " " << pos_y << " " << start_col << " " << start_row << std::endl;

    for (int curr_row = -within_dist_map; curr_row <= within_dist_map; curr_row++) {
        for(int curr_col = -within_dist_map; curr_col <= within_dist_map; curr_col++) {            
            if ((curr_row * curr_row + curr_col * curr_col) <= within_dist_map * within_dist_map) {
                if ((start_row+curr_row) >= 0 && (start_row+curr_row) < map_msg.info.height && (start_col+curr_col) >= 0 && (start_col+curr_col) < map_msg.info.width) {
                    double distance = std::sqrt(curr_row * curr_row + curr_col * curr_col);

                    int mark_pos_array = (start_row+curr_row) * map_msg.info.width + (start_col+curr_col);
                    if (map_msg.data[mark_pos_array] == 100 && distance < min_dist ) {
                           min_dist = distance;
                           found = true;
                           dx_ret = curr_col * map_msg.info.resolution;
                           dy_ret = curr_row * map_msg.info.resolution;
                    }
                }
            }
        }
    }

    //std::cout << "dx dy: " << dx_ret << " " << dy_ret << std::endl;
}

void match_lines_and_icp(std::vector<line_t>& scan_lines, std::vector<line_t>& map_lines,const nav_msgs::OccupancyGrid& map_msg, std::vector<point_xy_t>& laser_scan_points, double x_guess, double y_guess, double orientation_guess, double& x_ret, double& y_ret, double& orientation_ret) {
	x_ret = x_guess;
	y_ret = y_guess;
	orientation_ret = orientation_guess;

	translate_lines(scan_lines,x_guess,y_guess,orientation_guess);
    translate_points(laser_scan_points,x_guess,y_guess,orientation_guess);

    //Match lines
	std::vector<line_pair_t> line_pairs;

	for (std::vector<line_t>::iterator it = scan_lines.begin(); it != scan_lines.end(); ++it) {
		line_t scan_line = *it;		
		std::vector<line_t>::iterator closest_map_line;

		double min_distance_metric = 10000;
		
		for (std::vector<line_t>::iterator itt = map_lines.begin(); itt != map_lines.end(); ++itt) {
			line_t map_line = *itt;	

			double delta_r = std::abs(map_line.r - scan_line.r);
			double delta_theta = std::abs(get_angle_difference_within_90_deg(scan_line.theta, map_line.theta));
			double distance_metric = delta_r + delta_theta;	
			
			if (delta_r < 0.2 && delta_theta < 0.2 && distance_metric < min_distance_metric) {
				min_distance_metric = distance_metric;
				closest_map_line = itt;		
			}
		}

		if (min_distance_metric < 10000) {
			line_pair_t line_pair_it;
			line_pair_it.scan_line = it;
			line_pair_it.map_line = closest_map_line;
			line_pairs.push_back(line_pair_it);
	
			line_t line_it = *line_pair_it.scan_line;
			std::cout << "line matching d_r d_theta dist_met: " << min_distance_metric << std::endl;		
			std::cout << "line r theta angle: " << line_it.r << " " << line_it.theta << " " << line_it.line_angle << std::endl;
			std::cout << "point_a x y: " << line_it.point_a.x << " " << line_it.point_a.y << std::endl;
			std::cout << "point_b x y: " << line_it.point_b.x << " " << line_it.point_b.y << std::endl;	

			line_it = *line_pair_it.map_line;
			std::cout << "line r theta angle: " << line_it.r << " " << line_it.theta << " " << line_it.line_angle << std::endl;
			std::cout << "point_a x y: " << line_it.point_a.x << " " << line_it.point_a.y << std::endl;
			std::cout << "point_b x y: " << line_it.point_b.x << " " << line_it.point_b.y << std::endl;
		}
	}
		
	std::cout << "Matched lines: " << line_pairs.size() << std::endl;
	
	if(line_pairs.size() == 0) {return;}

	double theta_error_sum = 0;
	for (std::vector<line_pair_t>::iterator it = line_pairs.begin(); it != line_pairs.end(); ++it) {
		double theta_error = get_angle_difference_within_90_deg(it->scan_line->theta, it->map_line->theta);
		theta_error_sum += theta_error;
	}
    double theta_error_avg = theta_error_sum / line_pairs.size();

    theta_error_avg *= 0.5;

	std::cout << "Theta error avg: " << theta_error_avg << std::endl;
		
	translate_lines(scan_lines,0,0,theta_error_avg);	
    translate_points(laser_scan_points,0,0,theta_error_avg);
	orientation_ret += theta_error_avg;	

    double x_error_sum = 0, y_error_sum = 0, x_error_avg = 0, y_error_avg = 0;
	
    int matches = 0;
    for (std::vector<point_xy_t>::iterator it = laser_scan_points.begin(); it != laser_scan_points.end(); ++it) {
        bool match = false;
        double dx = 0, dy = 0;

        find_closest_wall_in_map(map_msg, it->x, it->y, 0.05, dx, dy, match);
        if (match) {
            x_error_sum += dx;
            y_error_sum += dy;
            matches++;
        }
	}
    std::cout << "Matched points: " << matches << std::endl;

    if (matches==0) {return;}

    x_error_avg = x_error_sum / matches;
    y_error_avg = y_error_sum / matches;

    x_error_avg *= 0.5;
    y_error_avg *= 0.5;

	translate_lines(scan_lines,x_error_avg,y_error_avg,0);
    translate_points(laser_scan_points,x_error_avg,y_error_avg,0);
	x_ret += x_error_avg;
	y_ret += y_error_avg;	

	std::cout << "x trans, y trans: " << x_error_avg << " " << y_error_avg << std::endl;	
}


void update_occ_grid(nav_msgs::OccupancyGrid& map_msg, std::vector<point_xy_t>& points_xy, double pos_x_robot, double pos_y_robot, double orientation_z_robot) {
	
	for (std::vector<point_xy_t>::iterator it = points_xy.begin(); it != points_xy.end(); ++it) {
		point_xy_t point_it = *it;
		double x_obst = point_it.x * std::cos(orientation_z_robot) + point_it.y * std::sin(orientation_z_robot);
		double y_obst = -point_it.x * std::sin(orientation_z_robot) + point_it.y * std::cos(orientation_z_robot);
		x_obst += pos_x_robot;		
		y_obst += pos_y_robot;

		mark_map(map_msg,x_obst,y_obst);
	}
}

int main(int argc, char **argv) {		
	ros::init(argc, argv, "ras_g9_mapping_and_localization_node");

	ros::NodeHandle n;

	Callback_class callback_object;

	// Create subscribers and publishers:	
	ros::Subscriber odometry_sub = n.subscribe("/odom",1,&Callback_class::odometry_callback,&callback_object);
    	ros::Subscriber laser_scan_sub = n.subscribe("/scan",1,&Callback_class::laser_scan_callback,&callback_object);
	
	ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("/map",1);
	ros::Publisher map_lines_pub = n.advertise<visualization_msgs::Marker>("/map_lines",1);
	ros::Publisher position_pub = n.advertise<geometry_msgs::PoseStamped>("/robot_position",1);

	tf::TransformBroadcaster position_broadcaster;
	
	// Map metadata:	
	nav_msgs::OccupancyGrid map_msg;

	map_msg.header.seq = 0;
	map_msg.header.stamp = ros::Time::now();
	map_msg.header.frame_id = "map";

	map_msg.info.map_load_time = ros::Time::now(); // The time at which the map was loaded
	map_msg.info.resolution = 0.02; // The map resolution [m/cell]
	map_msg.info.width = 130; // Map width [cells]
	map_msg.info.height = 130; // Map height [cells]
	map_msg.info.origin = geometry_msgs::Pose(); // The origin of the map [m, m, rad].  This is the real-world pose of the cell (0,0) in the map.	

	map_msg.data.resize(map_msg.info.width * map_msg.info.height);

	// Lines, corners:
	std::vector<line_t> map_lines;
	std::vector<line_t> extracted_lines;

	// Read map from file:
	std::cout << "Opening map file" << std::endl;
	std::ifstream map_file(MAP_FILE_PATH);
	std::string line;

	if (map_file.is_open()) {
		while ( getline (map_file,line) ) {
			if (line.empty() || line.at(0) == '#') {continue;}

			double wall_start_x, wall_start_y, wall_end_x, wall_end_y;			
			std::stringstream line_stream(line);
			line_stream >> wall_start_x >> wall_start_y >> wall_end_x >> wall_end_y;
			
			draw_wall_map(map_msg,wall_start_x,wall_start_y,wall_end_x,wall_end_y);
					
			point_xy_t point_a, point_b;
			point_a.x = wall_start_x;
			point_a.y = wall_start_y;
			point_b.x = wall_end_x;
			point_b.y = wall_end_y;
	
            line_t map_line = create_line_struct(point_a,point_b);
			map_lines.push_back(map_line);
		}
		map_file.close();
		std::cout << "Map file read" << std::endl;
	} else { 
		std::cout << "Unable to open map file [is address: " << MAP_FILE_PATH << " correct?]" << std::endl; 	
		return -1;
	}

	// Line viz msg:
	visualization_msgs::Marker line_list_msg;
	line_list_msg.header.frame_id = "map";
	line_list_msg.header.stamp = ros::Time::now();
	line_list_msg.ns = "lines";	
	line_list_msg.action = visualization_msgs::Marker::ADD;
	line_list_msg.pose.orientation.w = 1.0; // ?
	line_list_msg.id = 1;
	line_list_msg.type = visualization_msgs::Marker::LINE_LIST;
	line_list_msg.scale.x = 0.01;
	line_list_msg.color.b = 1.0;
	line_list_msg.color.a = 1.0;

	// Position msg:
	geometry_msgs::PoseStamped position_msg;
	position_msg.header.frame_id = "map";
	position_msg.header.stamp = ros::Time::now();

	// Point cloud xy
	std::vector<point_xy_t> laser_scan_xy;

	// Position:
	double pos_x_robot = 0.2, pos_y_robot = 0.2, orientation_z_robot = M_PI/2.0;
	double pos_x_robot_odom_old = 0, pos_y_robot_odom_old = 0, orientation_z_robot_odom_old = 0;

	ros::Time time_mark;

	bool running = false;

	ros::Rate r(MAP_AND_POSITION_UPDATE_RATE);
	while(n.ok()) {
		time_mark = ros::Time::now();		

		ros::spinOnce(); // Check for incoming messages

		double pos_x_robot_odom = 0, pos_y_robot_odom = 0, orientation_z_robot_odom = 0;		
		callback_object.get_position_odom(pos_x_robot_odom, pos_y_robot_odom,orientation_z_robot_odom);

    	/*
		if (orientation_z_robot_odom_old == 0 && orientation_z_robot_odom > 1.5) {
			orientation_z_robot_odom_old = orientation_z_robot_odom;
			running = true;
		}	
		if (!running) {r.sleep(); continue;}
    	*/

		// Calculate the odometry delta's

    	double delta_orientation_odom = get_angle_difference_within_180_deg(orientation_z_robot_odom_old,orientation_z_robot_odom);
		double delta_x_robot_odom = pos_x_robot_odom - pos_x_robot_odom_old;
		double delta_y_robot_odom = pos_y_robot_odom - pos_y_robot_odom_old;

		double orientation_odom_difference = get_angle_difference_within_180_deg(orientation_z_robot_odom,orientation_z_robot);
    	double delta_x_robot_odom_rot = delta_x_robot_odom * std::cos(orientation_odom_difference) - delta_y_robot_odom * std::sin(orientation_odom_difference);
    	double delta_y_robot_odom_rot = delta_x_robot_odom * std::sin(orientation_odom_difference) + delta_y_robot_odom * std::cos(orientation_odom_difference);

		std::cout << "odom x y or: " << pos_x_robot_odom << " " << pos_y_robot_odom << " " << orientation_z_robot_odom << std::endl;
   		std::cout << "odom delta x y or " << delta_x_robot_odom << " " << delta_y_robot_odom << " " << delta_orientation_odom << std::endl;

		pos_x_robot_odom_old = pos_x_robot_odom;		
		pos_y_robot_odom_old = pos_y_robot_odom;		
		orientation_z_robot_odom_old = orientation_z_robot_odom;
		
		// Extract + match features
		
		translate_laser_scan_xy(laser_scan_xy,callback_object.get_laser_scan());		
		extract_lines(extracted_lines,laser_scan_xy);
    	match_lines_and_icp(extracted_lines,map_lines,map_msg,laser_scan_xy,pos_x_robot + delta_x_robot_odom_rot, pos_y_robot + delta_y_robot_odom_rot, orientation_z_robot + delta_orientation_odom, pos_x_robot, pos_y_robot, orientation_z_robot);
		//update_occ_grid(map_msg,laser_scan_xy,pos_x_robot,pos_y_robot,orientation_z_robot);	

		/*
		// TEMP:
		orientation_z_robot += delta_orientation_odom;
		pos_x_robot += delta_x_robot_odom_rot;
		pos_y_robot += delta_y_robot_odom_rot;
		*/

		std::cout << "x_pos, y_pos, orientation: " << pos_x_robot << " " << pos_y_robot << " " << orientation_z_robot << std::endl;
	
		
		/*
		std::cout 	<< "Position" << std::endl
					<< "x: " << pos_x_robot << std::endl
					<< "y: " << pos_y_robot << std::endl
					<< "orientation: " << orientation_z_robot << std::endl;		
		*/		
		
		/*
		std::cout << std::endl << "Lines extracted: " << std::endl;
		for (std::vector<line_t>::iterator it = extracted_lines.begin(); it != extracted_lines.end(); ++it) {
			line_t line_it = *it;		
			std::cout << "line r theta angle: " << line_it.r << " " << line_it.theta << " " << line_it.line_angle << std::endl;
			std::cout << "point_a x y: " << line_it.point_a.x << " " << line_it.point_a.y << std::endl;
			std::cout << "point_b x y: " << line_it.point_b.x << " " << line_it.point_b.y << std::endl;	
			std::cout << "end_point_a x y: " << line_it.end_point_a.x << " " << line_it.end_point_a.y << std::endl;	
			std::cout << "end_point_b x y: " << line_it.end_point_b.x << " " << line_it.end_point_b.y << std::endl;	
		}

		std::cout << "Corners extracted: " << std::endl;
		for (std::vector<point_xy_t>::iterator it = extracted_corners.begin(); it != extracted_corners.end(); ++it) {
			std::cout << "x y: " << it->x << " " << it->y << std::endl;
		}
		*/


		// Publish extracted lines:
		line_list_msg.points.clear();
		for (std::vector<line_t>::iterator it = extracted_lines.begin(); it != extracted_lines.end(); ++it) {
			geometry_msgs::Point p1, p2;
			line_t line_it = *it;			

			p1.x = line_it.point_a.x + line_it.dx*100;
			p1.y = line_it.point_a.y + line_it.dy*100;
			p2.x = line_it.point_b.x - line_it.dx*100;
			p2.y = line_it.point_b.y - line_it.dy*100;

			line_list_msg.points.push_back(p1);
			line_list_msg.points.push_back(p2);			
		}
		map_lines_pub.publish(line_list_msg);

		// Publish map:
		map_pub.publish(map_msg);

		// Send TF transform:
		tf::Quaternion odom_quat(0,0,0,0);
		odom_quat.setEuler(0,0,orientation_z_robot);

		tf::Vector3 odom_vector(pos_x_robot,pos_y_robot,0.0);
		
		position_broadcaster.sendTransform(
			tf::StampedTransform(
				tf::Transform(odom_quat,odom_vector),
				ros::Time::now(),
				"map",
				"base_link"
			)		  
		);

		// Publish position.
		position_msg.pose.position.x = pos_x_robot;
		position_msg.pose.position.y = pos_y_robot;
		position_msg.pose.position.z = 0;

		position_msg.pose.orientation = tf::createQuaternionMsgFromYaw(orientation_z_robot);
		
		position_pub.publish(position_msg);

		ros::Duration iteration_time = ros::Time::now() - time_mark;
		std::cout << "Iteration time: " << iteration_time << std::endl << std::endl;

        //if(extracted_lines.size() > 0) {break;}

		r.sleep();
	}
}
