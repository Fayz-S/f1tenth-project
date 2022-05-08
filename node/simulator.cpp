#include <ros/ros.h>

// interactive marker
#include <interactive_markers/interactive_marker_server.h>

#include <tf2/impl/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32MultiArray.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include "f1tenth_simulator/pose_2d.hpp"
#include "f1tenth_simulator/ackermann_kinematics.hpp"
#include "f1tenth_simulator/scan_simulator_2d.hpp"

#include "f1tenth_simulator/car_state.hpp"
#include "f1tenth_simulator/car_params.hpp"
#include "f1tenth_simulator/ks_kinematics.hpp"
#include "f1tenth_simulator/st_kinematics.hpp"
#include "f1tenth_simulator/precompute.hpp"

#include <iostream>
#include <math.h>


using namespace racecar_simulator;

class RacecarSimulator {
private:
    // A ROS node
    ros::NodeHandle n;

    // The transformation frames used
    std::string map_frame, base_frame_blue, scan_frame_blue, base_frame_red, scan_frame_red;

    // obstacle states (1D index) and parameters
    std::vector<int> added_obs;
    // listen for clicked point for adding obstacles
    ros::Subscriber obs_sub;
    int obstacle_size;

    // interactive markers' server
    interactive_markers::InteractiveMarkerServer im_server;

    // The car state_blue and parameters
    CarState state_blue;
    CarState state_red;
    double previous_seconds;
    double scan_distance_to_base_link;
    double max_speed, max_steering_angle;
    double max_accel, max_steering_vel, max_decel;
    double desired_speed_blue, desired_steer_ang_blue, desired_speed_red, desired_steer_ang_red;
    double accel_blue, steer_angle_vel_blue, accel_red, steer_angle_vel_red;
    double cube_width;
    CarParams params_blue;
    CarParams params_red;
    double width;

    // A simulator of the laser
    ScanSimulator2D scan_simulator;
    double map_free_threshold;

    // For publishing transformations
    tf2_ros::TransformBroadcaster br;

    // A timer to update the pose
    ros::Timer update_pose_timer_blue;
    ros::Timer update_pose_timer_red;

    // Listen for drive commands
    ros::Subscriber drive_sub_blue;
    ros::Subscriber drive_sub_red;

    // Listen for a map
    ros::Subscriber map_sub;
    bool map_exists = false;

    // Listen for updates to the pose
    ros::Subscriber pose_sub_blue;
    ros::Subscriber pose_sub_red;
    ros::Subscriber pose_rviz_sub;

    // Publish a scan, odometry, and imu data
    bool broadcast_transform;
    bool pub_gt_pose;
    ros::Publisher scan_pub_blue;
    ros::Publisher scan_pub_red;
    ros::Publisher pose_pub;


    ros::Publisher odom_pub;
    ros::Publisher imu_pub;

    // publisher for map with obstacles
    ros::Publisher map_pub;

    // keep an original map for obstacles
    nav_msgs::OccupancyGrid original_map;
    nav_msgs::OccupancyGrid current_map;

    // for obstacle collision
    int map_width, map_height;
    double map_resolution, origin_x, origin_y;

    // safety margin for collisions
    double thresh;
    double speed_clip_diff;

    // precompute cosines of scan angles
    std::vector<double> cosines;

    // scan parameters
    double scan_fov;
    double scan_ang_incr;

    // pi
    const double PI = 3.1415;

    // precompute distance from lidar to edge of car for each beam
    std::vector<double> car_distances;

    // for collision check
    bool TTC = false;
    double ttc_threshold;

    // steering delay
    int buffer_length;
    std::vector<double> steering_buffer_blue;
    std::vector<double> steering_buffer_red;


public:

    RacecarSimulator(): im_server("racecar_sim") {
        // Initialize the node handle
        n = ros::NodeHandle("~");

        // Initialize car state_blue and driving commands
        state_blue = {.x=0.3, .y=0.3, .theta=0, .velocity=0, .steer_angle=0.0, .angular_velocity=0.0, .slip_angle=0.0, .st_dyn=false};
        accel_blue = 0.0;
        steer_angle_vel_blue = 0.0;
        desired_speed_blue = 0.0;
        desired_steer_ang_blue = 0.0;
        
        previous_seconds = ros::Time::now().toSec();

        state_red = {.x=0, .y=-0.3, .theta=0, .velocity=0, .steer_angle=0.0, .angular_velocity=0.0, .slip_angle=0.0, .st_dyn=false};
        accel_red = 0.0;
        steer_angle_vel_red = 0.0;
        desired_speed_red = 0.0;
        desired_steer_ang_red = 0.0;
        

        // Get the topic names
        std::string drive_topic_blue, drive_topic_red, map_topic, scan_topic_blue, scan_topic_red, pose_topic_blue, pose_topic_red, gt_pose_topic,
        pose_rviz_topic, odom_topic, imu_topic;
        n.getParam("drive_topic_blue", drive_topic_blue);
        n.getParam("drive_topic_red", drive_topic_red);
        n.getParam("map_topic", map_topic);
        n.getParam("scan_topic_blue", scan_topic_blue);
        n.getParam("scan_topic_red", scan_topic_red);
        n.getParam("pose_topic_blue", pose_topic_blue);
        n.getParam("pose_topic_red", pose_topic_red);
        n.getParam("odom_topic", odom_topic);
        n.getParam("pose_rviz_topic", pose_rviz_topic);
        n.getParam("imu_topic", imu_topic);
        n.getParam("ground_truth_pose_topic", gt_pose_topic);
        n.getParam("cube_width", cube_width);
        // Get steering delay params_blue
        n.getParam("buffer_length", buffer_length);

        // Get the transformation frame names
        n.getParam("map_frame", map_frame);
        n.getParam("base_frame_blue", base_frame_blue);
        n.getParam("scan_frame_blue", scan_frame_blue);
        n.getParam("base_frame_red", base_frame_red);
        n.getParam("scan_frame_red", scan_frame_red);

        // Fetch the car parameters
        int scan_beams;
        double update_pose_rate, scan_std_dev;
        double scan_max_range;
        n.getParam("wheelbase", params_blue.wheelbase);
        n.getParam("wheelbase", params_red.wheelbase);
        n.getParam("update_pose_rate", update_pose_rate);
        n.getParam("scan_beams", scan_beams);
        n.getParam("scan_field_of_view", scan_fov);
        n.getParam("scan_std_dev", scan_std_dev);
        n.getParam("scan_max_range", scan_max_range);
        n.getParam("map_free_threshold", map_free_threshold);
        n.getParam("scan_distance_to_base_link", scan_distance_to_base_link);
        n.getParam("max_speed", max_speed);
        n.getParam("max_steering_angle", max_steering_angle);
        n.getParam("max_accel", max_accel);
        n.getParam("max_decel", max_decel);
        n.getParam("max_steering_vel", max_steering_vel);
        n.getParam("friction_coeff", params_blue.friction_coeff);
        n.getParam("height_cg", params_blue.h_cg);
        n.getParam("l_cg2rear", params_blue.l_r);
        n.getParam("l_cg2front", params_blue.l_f);
        n.getParam("C_S_front", params_blue.cs_f);
        n.getParam("C_S_rear", params_blue.cs_r);
        n.getParam("moment_inertia", params_blue.I_z);
        n.getParam("mass", params_blue.mass);
        n.getParam("width", width);
        n.getParam("friction_coeff", params_red.friction_coeff);
        n.getParam("height_cg", params_red.h_cg);
        n.getParam("l_cg2rear", params_red.l_r);
        n.getParam("l_cg2front", params_red.l_f);
        n.getParam("C_S_front", params_red.cs_f);
        n.getParam("C_S_rear", params_red.cs_r);
        n.getParam("moment_inertia", params_red.I_z);
        n.getParam("mass", params_red.mass);

        // clip velocity
        n.getParam("speed_clip_diff", speed_clip_diff);

        // Determine if we should broadcast
        n.getParam("broadcast_transform", broadcast_transform);
        n.getParam("publish_ground_truth_pose", pub_gt_pose);

        // Get obstacle size parameter
        n.getParam("obstacle_size", obstacle_size);

        // Initialize a simulator of the laser scanner
        scan_simulator = ScanSimulator2D(
            scan_beams,
            scan_fov,
            scan_std_dev,
            scan_max_range,
            cube_width);

        // Make a publisher for laser scan messages
        scan_pub_blue = n.advertise<sensor_msgs::LaserScan>(scan_topic_blue, 1);
        scan_pub_red = n.advertise<sensor_msgs::LaserScan>(scan_topic_red, 1);

        // Make a publisher for odometry messages
        odom_pub = n.advertise<nav_msgs::Odometry>(odom_topic, 1);

        // Make a publisher for IMU messages
        imu_pub = n.advertise<sensor_msgs::Imu>(imu_topic, 1);

        // Make a publisher for publishing map with obstacles
        map_pub = n.advertise<nav_msgs::OccupancyGrid>("/map", 1);

        // Make a publisher for ground truth pose
        pose_pub = n.advertise<geometry_msgs::PoseStamped>(gt_pose_topic, 10);

        // Start a timer to output the pose
        //
        update_pose_timer_red = n.createTimer(ros::Duration(update_pose_rate), &RacecarSimulator::update_pose_red, this);
        update_pose_timer_blue = n.createTimer(ros::Duration(update_pose_rate), &RacecarSimulator::update_pose_blue, this);


        // Start a subscriber to listen to drive commands
        drive_sub_blue = n.subscribe(drive_topic_blue, 1, &RacecarSimulator::drive_callback_blue, this);
        drive_sub_red = n.subscribe(drive_topic_red, 1, &RacecarSimulator::drive_callback_red, this);

        // Start a subscriber to listen to new maps
        map_sub = n.subscribe(map_topic, 1, &RacecarSimulator::map_callback, this);

        // Start a subscriber to listen to pose messages
        pose_sub_blue = n.subscribe(pose_topic_blue, 1, &RacecarSimulator::pose_callback_blue, this);
        pose_sub_red = n.subscribe(pose_topic_red, 1, &RacecarSimulator::pose_callback_red, this);
        pose_rviz_sub = n.subscribe(pose_rviz_topic, 1, &RacecarSimulator::pose_rviz_callback, this);

        // obstacle subscriber
        obs_sub = n.subscribe("/clicked_point", 1, &RacecarSimulator::obs_callback, this);

        // get collision safety margin
        n.getParam("coll_threshold", thresh);
        n.getParam("ttc_threshold", ttc_threshold);

        scan_ang_incr = scan_simulator.get_angle_increment();

        cosines = Precompute::get_cosines(scan_beams, -scan_fov/2.0, scan_ang_incr);
        car_distances = Precompute::get_car_distances(scan_beams, params_blue.wheelbase, width, 
                scan_distance_to_base_link, -scan_fov/2.0, scan_ang_incr);


        // steering delay buffer
        steering_buffer_blue = std::vector<double>(buffer_length);
        steering_buffer_red = std::vector<double>(buffer_length);

        // OBSTACLE BUTTON:
        // wait for one map message to get the map data array
        boost::shared_ptr<nav_msgs::OccupancyGrid const> map_ptr;
        nav_msgs::OccupancyGrid map_msg;
        map_ptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map");
        if (map_ptr != NULL) {
            map_msg = *map_ptr;
        }
        original_map = map_msg;
        current_map = map_msg;
        std::vector<int8_t> map_data_raw = map_msg.data;
        std::vector<int> map_data(map_data_raw.begin(), map_data_raw.end());

        map_width = map_msg.info.width;
        map_height = map_msg.info.height;
        origin_x = map_msg.info.origin.position.x;
        origin_y = map_msg.info.origin.position.y;
        map_resolution = map_msg.info.resolution;

        // create button for clearing obstacles
        visualization_msgs::InteractiveMarker clear_obs_button;
        clear_obs_button.header.frame_id = "map";
        // clear_obs_button.pose.position.x = origin_x+(1/3)*map_width*map_resolution;
        // clear_obs_button.pose.position.y = origin_y+(1/3)*map_height*map_resolution;
        // TODO: find better positioning of buttons
        clear_obs_button.pose.position.x = 0;
        clear_obs_button.pose.position.y = -5;
        clear_obs_button.scale = 1;
        clear_obs_button.name = "clear_obstacles";
        clear_obs_button.description = "Clear Obstacles\n(Left Click)";
        visualization_msgs::InteractiveMarkerControl clear_obs_control;
        clear_obs_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
        clear_obs_control.name = "clear_obstacles_control";
        // make a box for the button
        visualization_msgs::Marker clear_obs_marker;
        clear_obs_marker.type = visualization_msgs::Marker::CUBE;
        clear_obs_marker.scale.x = clear_obs_button.scale*0.45;
        clear_obs_marker.scale.y = clear_obs_button.scale*0.65;
        clear_obs_marker.scale.z = clear_obs_button.scale*0.45;
        clear_obs_marker.color.r = 0.0;
        clear_obs_marker.color.g = 1.0;
        clear_obs_marker.color.b = 0.0;
        clear_obs_marker.color.a = 1.0;

        clear_obs_control.markers.push_back(clear_obs_marker);
        clear_obs_control.always_visible = true;
        clear_obs_button.controls.push_back(clear_obs_control);

        im_server.insert(clear_obs_button);
        im_server.setCallback(clear_obs_button.name, boost::bind(&RacecarSimulator::clear_obstacles, this, _1));

        im_server.applyChanges();

        ROS_INFO("Simulator constructed.");
    }

    void update_pose_blue(const ros::TimerEvent&) {
        // simulate P controller
        compute_accel_blue(desired_speed_blue);
        double actual_ang_blue = 0.0;
        if (steering_buffer_blue.size() < buffer_length) {
            steering_buffer_blue.push_back(desired_steer_ang_blue);
            actual_ang_blue = 0.0;
        } else {
            steering_buffer_blue.insert(steering_buffer_blue.begin(), desired_steer_ang_blue);
            actual_ang_blue = steering_buffer_blue.back();
            steering_buffer_blue.pop_back();
        }
        set_steer_angle_vel_blue(compute_steer_vel_blue(actual_ang_blue));

        // Update the pose
        ros::Time timestamp = ros::Time::now();
        double current_seconds = timestamp.toSec();
        state_blue = STKinematics::update(
            state_blue,
            accel_blue,
            steer_angle_vel_blue,
            params_blue,
            current_seconds - previous_seconds);
        state_blue.velocity = std::min(std::max(state_blue.velocity, -max_speed), max_speed);
        state_blue.steer_angle = std::min(std::max(state_blue.steer_angle, -max_steering_angle), max_steering_angle);
        

        previous_seconds = current_seconds;

        /// Publish the pose as a transformation
        pub_pose_transform_blue(timestamp);

        /// Publish the steering angle as a transformation so the wheels move
        pub_steer_ang_transform_blue(timestamp);

        // Make an odom message as well and publish it
        pub_odom_blue(timestamp);

        // TODO: make and publish IMU message
        pub_imu(timestamp);


        /// KEEP in sim
        // If we have a map, perform a scan
        if (map_exists) {
            // Get the pose of the lidar, given the pose of base link
            // (base link is the center of the rear axle)
            Pose2D scan_pose;
            scan_pose.x = state_blue.x + scan_distance_to_base_link * std::cos(state_blue.theta);
            scan_pose.y = state_blue.y + scan_distance_to_base_link * std::sin(state_blue.theta);
            scan_pose.theta = state_blue.theta;
            // we need the center position of opponent car
            Pose2D opponent_pose;
            opponent_pose.x = state_red.x;
            opponent_pose.y = state_red.y;
            opponent_pose.theta = state_red.theta;
            // Compute the scan from the lidar
            std::vector<double> scan = scan_simulator.scan(scan_pose, opponent_pose);

            // Convert to float
            std::vector<float> scan_(scan.size());
            for (size_t i = 0; i < scan.size(); i++)
                scan_[i] = scan[i];

            // TTC Calculations are done here so the car can be halted in the simulator:
            // to reset TTC
            bool no_collision = true;
            if (state_blue.velocity != 0) {
                for (size_t i = 0; i < scan_.size(); i++) {
                    // TTC calculations

                    // calculate projected velocity
                    double proj_velocity = state_blue.velocity * cosines[i];
                    double ttc = (scan_[i] - car_distances[i]) / proj_velocity;
                    // if it's small enough to count as a collision
                    if ((ttc < ttc_threshold) && (ttc >= 0.0)) { 
                        if (!TTC) {
                            first_ttc_actions_blue();
                        }

                        no_collision = false;
                        TTC = true;

                        ROS_INFO("Collision detected");
                    }
                }
            }

            // reset TTC
            if (no_collision)
                TTC = false;

            // Publish the laser message
            sensor_msgs::LaserScan scan_msg;
            scan_msg.header.stamp = timestamp;
            scan_msg.header.frame_id = scan_frame_blue;
            scan_msg.angle_min = -scan_simulator.get_field_of_view()/2.;
            scan_msg.angle_max =  scan_simulator.get_field_of_view()/2.;
            scan_msg.angle_increment = scan_simulator.get_angle_increment();
            scan_msg.range_max = 100;
            scan_msg.ranges = scan_;
            scan_msg.intensities = scan_;

            scan_pub_blue.publish(scan_msg);


            // Publish a transformation between base link and laser
            pub_laser_link_transform_blue(timestamp);

        }

    } // end of update_pose

    void update_pose_red(const ros::TimerEvent&) {
        // simulate P controller
        compute_accel_red(desired_speed_red);
        double actual_ang_red = 0.0;
        if (steering_buffer_red.size() < buffer_length) {
            steering_buffer_red.push_back(desired_steer_ang_red);
            actual_ang_red = 0.0;
        } else {
            steering_buffer_red.insert(steering_buffer_red.begin(), desired_steer_ang_red);
            actual_ang_red = steering_buffer_red.back();
            steering_buffer_red.pop_back();
        }
        set_steer_angle_vel_red(compute_steer_vel_red(actual_ang_red));

        // Update the pose
        ros::Time timestamp = ros::Time::now();
        double current_seconds = timestamp.toSec();
        state_red = STKinematics::update(
                state_red,
                accel_red,
                steer_angle_vel_red,
                params_red,
                current_seconds - previous_seconds);
        state_red.velocity = std::min(std::max(state_red.velocity, -max_speed), max_speed);
        state_red.steer_angle = std::min(std::max(state_red.steer_angle, -max_steering_angle), max_steering_angle);

        previous_seconds = current_seconds;

        /// Publish the pose as a transformation
        pub_pose_transform_red(timestamp);

        /// Publish the steering angle as a transformation so the wheels move
        pub_steer_ang_transform_red(timestamp);

        // Make an odom message as well and publish it
        pub_odom_red(timestamp);

        // TODO: make and publish IMU message
        pub_imu(timestamp);


        /// KEEP in sim
        // If we have a map, perform a scan
        if (map_exists) {
            // Get the pose of the lidar, given the pose of base link
            // (base link is the center of the rear axle)
            Pose2D scan_pose;
            scan_pose.x = state_red.x + scan_distance_to_base_link * std::cos(state_red.theta);
            scan_pose.y = state_red.y + scan_distance_to_base_link * std::sin(state_red.theta);
            scan_pose.theta = state_red.theta;
            Pose2D opponent_pose;
            opponent_pose.x = state_blue.x;
            opponent_pose.y = state_blue.y;
            opponent_pose.theta = state_blue.theta;
            // Compute the scan from the lidar
            std::vector<double> scan = scan_simulator.scan(scan_pose, opponent_pose);

            // Convert to float
            std::vector<float> scan_(scan.size());
            for (size_t i = 0; i < scan.size(); i++)
                scan_[i] = scan[i];

            // TTC Calculations are done here so the car can be halted in the simulator:
            // to reset TTC
            bool no_collision = true;
            if (state_red.velocity != 0) {
                for (size_t i = 0; i < scan_.size(); i++) {
                    // TTC calculations

                    // calculate projected velocity
                    double proj_velocity = state_red.velocity * cosines[i];
                    double ttc = (scan_[i] - car_distances[i]) / proj_velocity;
                    // if it's small enough to count as a collision
                    if ((ttc < ttc_threshold) && (ttc >= 0.0)) {
                        if (!TTC) {
                            first_ttc_actions_red();
                        }

                        no_collision = false;
                        TTC = true;

                        ROS_INFO("Collision detected");
                    }
                }
            }

            // reset TTC
            if (no_collision)
                TTC = false;

            // Publish the laser message
            sensor_msgs::LaserScan scan_msg;
            scan_msg.header.stamp = timestamp;
            //
            // red
            //
            scan_msg.header.frame_id = scan_frame_red;
            scan_msg.angle_min = -scan_simulator.get_field_of_view()/2.;
            scan_msg.angle_max =  scan_simulator.get_field_of_view()/2.;
            scan_msg.angle_increment = scan_simulator.get_angle_increment();
            scan_msg.range_max = 100;
            scan_msg.ranges = scan_;
            scan_msg.intensities = scan_;

            scan_pub_red.publish(scan_msg);


            // Publish a transformation between base link and laser
            pub_laser_link_transform_red(timestamp);

        }

    } // end of update_pose

        /// ---------------------- GENERAL HELPER FUNCTIONS ----------------------

    std::vector<int> ind_2_rc(int ind) {
        std::vector<int> rc;
        int row = floor(ind/map_width);
        int col = ind%map_width - 1;
        rc.push_back(row);
        rc.push_back(col);
        return rc;
    }

    int rc_2_ind(int r, int c) {
        return r*map_width + c;

    }

    std::vector<int> coord_2_cell_rc(double x, double y) {
        std::vector<int> rc;
        rc.push_back(static_cast<int>((y-origin_y)/map_resolution));
        rc.push_back(static_cast<int>((x-origin_x)/map_resolution));
        return rc;
    }

    void first_ttc_actions_blue() {
        // completely stop vehicle
        state_blue.velocity = 0.0;
        state_blue.angular_velocity = 0.0;
        state_blue.slip_angle = 0.0;
        state_blue.steer_angle = 0.0;
        steer_angle_vel_blue = 0.0;
        accel_blue = 0.0;
        desired_speed_blue = 0.0;
        desired_steer_ang_blue = 0.0;
    }

    void first_ttc_actions_red() {
        // completely stop vehicle
        state_red.velocity = 0.0;
        state_red.angular_velocity = 0.0;
        state_red.slip_angle = 0.0;
        state_red.steer_angle = 0.0;
        steer_angle_vel_red = 0.0;
        accel_red = 0.0;
        desired_speed_red = 0.0;
        desired_steer_ang_red = 0.0;
    }

    void set_accel_blue(double accel_) {
        accel_blue = std::min(std::max(accel_, -max_accel), max_accel);
    }

    void set_accel_red(double accel_) {
        accel_red = std::min(std::max(accel_, -max_accel), max_accel);
    }
    
    void set_steer_angle_vel_blue(double steer_angle_vel_) {
        steer_angle_vel_blue = std::min(std::max(steer_angle_vel_, -max_steering_vel), max_steering_vel);
    }

    void set_steer_angle_vel_red(double steer_angle_vel_) {
        steer_angle_vel_red = std::min(std::max(steer_angle_vel_, -max_steering_vel), max_steering_vel);
    }

    void add_obs(int ind) {
        std::vector<int> rc = ind_2_rc(ind);
        for (int i=-obstacle_size; i<obstacle_size; i++) {
            for (int j=-obstacle_size; j<obstacle_size; j++) {
                int current_r = rc[0]+i;
                int current_c = rc[1]+j;
                int current_ind = rc_2_ind(current_r, current_c);
                current_map.data[current_ind] = 100;
            }
        }
        map_pub.publish(current_map);
    }

    void clear_obs(int ind) {
        std::vector<int> rc = ind_2_rc(ind);
        for (int i=-obstacle_size; i<obstacle_size; i++) {
            for (int j=-obstacle_size; j<obstacle_size; j++) {
                int current_r = rc[0]+i;
                int current_c = rc[1]+j;
                int current_ind = rc_2_ind(current_r, current_c);
                current_map.data[current_ind] = 0;

            }
        }
        map_pub.publish(current_map);
    }

    double compute_steer_vel_blue(double desired_angle) {
        // get difference between current and desired
        double dif = (desired_angle - state_blue.steer_angle);

        // calculate velocity
        double steer_vel;
        if (std::abs(dif) > .0001)  // if the difference is not trivial
            steer_vel = dif / std::abs(dif) * max_steering_vel;
        else {
            steer_vel = 0;
        }

        return steer_vel;
    }

    double compute_steer_vel_red(double desired_angle) {
        // get difference between current and desired
        double dif = (desired_angle - state_red.steer_angle);

        // calculate velocity
        double steer_vel;
        if (std::abs(dif) > .0001)  // if the difference is not trivial
            steer_vel = dif / std::abs(dif) * max_steering_vel;
        else {
            steer_vel = 0;
        }

        return steer_vel;
    }

    void compute_accel_blue(double desired_velocity) {
        // get difference between current and desired
        double dif = (desired_velocity - state_blue.velocity);

        if (state_blue.velocity > 0) {
            if (dif > 0) {
                // accelerate
                double kp = 2.0 * max_accel / max_speed;
                set_accel_blue(kp * dif);
            } else {
                // brake
                accel_blue = -max_decel; 
            }    
        } else if (state_blue.velocity < 0) {
            if (dif > 0) {
                // brake
                accel_blue = max_decel;

            } else {
                // accelerate
                double kp = 2.0 * max_accel / max_speed;
                set_accel_blue(kp * dif);
            }   
        } else {
	    // zero speed, accel_blue either way
	    double kp = 2.0 * max_accel / max_speed;
	    set_accel_blue(kp * dif);
	    }
    }

    void compute_accel_red(double desired_velocity) {
        // get difference between current and desired
        double dif = (desired_velocity - state_red.velocity);

        if (state_red.velocity > 0) {
            if (dif > 0) {
                // accelerate
                double kp = 2.0 * max_accel / max_speed;
                set_accel_red(kp * dif);
            } else {
                // brake
                accel_red = -max_decel;
            }
        } else if (state_red.velocity < 0) {
            if (dif > 0) {
                // brake
                accel_red = max_decel;

            } else {
                // accelerate
                double kp = 2.0 * max_accel / max_speed;
                set_accel_red(kp * dif);
            }
        } else {
            // zero speed, accel_red either way
            double kp = 2.0 * max_accel / max_speed;
            set_accel_red(kp * dif);
        }
    }

        /// ---------------------- CALLBACK FUNCTIONS ----------------------

    void obs_callback(const geometry_msgs::PointStamped &msg) {
        double x = msg.point.x;
        double y = msg.point.y;
        std::vector<int> rc = coord_2_cell_rc(x, y);
        int ind = rc_2_ind(rc[0], rc[1]);
        added_obs.push_back(ind);
        add_obs(ind);
    }

    void pose_callback_blue(const geometry_msgs::PoseStamped & msg) {
        state_blue.x = msg.pose.position.x;
        state_blue.y = msg.pose.position.y;
        geometry_msgs::Quaternion q = msg.pose.orientation;
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        state_blue.theta = tf2::impl::getYaw(quat);
    }
    
    void pose_callback_red(const geometry_msgs::PoseStamped & msg) {
        state_red.x = msg.pose.position.x;
        state_red.y = msg.pose.position.y;
        geometry_msgs::Quaternion q = msg.pose.orientation;
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        state_red.theta = tf2::impl::getYaw(quat);
    }

    void pose_rviz_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msg) {
        geometry_msgs::PoseStamped temp_pose;
        temp_pose.header = msg->header;
        temp_pose.pose = msg->pose.pose;
        pose_callback_blue(temp_pose);
        pose_callback_red(temp_pose);
    }

    void drive_callback_blue(const ackermann_msgs::AckermannDriveStamped & msg) {
        desired_speed_blue = msg.drive.speed;
        desired_steer_ang_blue = msg.drive.steering_angle;
    }

    void drive_callback_red(const ackermann_msgs::AckermannDriveStamped & msg) {
        desired_speed_red = msg.drive.speed;
        desired_steer_ang_red = msg.drive.steering_angle;
    }

    // button callbacks
    void clear_obstacles(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
        bool clear_obs_clicked = false;
        if (feedback->event_type == 3) {
            clear_obs_clicked = true;
        }
        if (clear_obs_clicked) {
            ROS_INFO("Clearing obstacles.");
            current_map = original_map;
            map_pub.publish(current_map);

            clear_obs_clicked = false;
        }
    }

        void map_callback(const nav_msgs::OccupancyGrid & msg) {
            // Fetch the map parameters
            size_t height = msg.info.height;
            size_t width = msg.info.width;
            double resolution = msg.info.resolution;
            // Convert the ROS origin to a pose
            Pose2D origin;
            // bottom right conner is the origin point
            origin.x = msg.info.origin.position.x;
            origin.y = msg.info.origin.position.y;
            geometry_msgs::Quaternion q = msg.info.origin.orientation;
            tf2::Quaternion quat(q.x, q.y, q.z, q.w);
            ROS_INFO_STREAM(height);
            origin.theta = tf2::impl::getYaw(quat);
            ROS_INFO_STREAM(width);

            // Convert the map to probability values
            std::vector<double> map(msg.data.size());
            for (size_t i = 0; i < height * width; i++) {
                if (msg.data[i] > 100 or msg.data[i] < 0) {
                    map[i] = 0.5; // Unknown
                } else {
                    map[i] = msg.data[i]/100.;
                }
            }

            // Send the map to the scanner
            scan_simulator.set_map(
                map,
                height,
                width,
                resolution,
                origin,
                map_free_threshold);
            map_exists = true;
        }

        /// ---------------------- PUBLISHING HELPER FUNCTIONS ----------------------

        void pub_pose_transform_blue(ros::Time timestamp) {
            // Convert the pose into a transformation
            geometry_msgs::Transform t;
            t.translation.x = state_blue.x;
            t.translation.y = state_blue.y;
            tf2::Quaternion quat;
            quat.setEuler(0., 0., state_blue.theta);
            t.rotation.x = quat.x();
            t.rotation.y = quat.y();
            t.rotation.z = quat.z();
            t.rotation.w = quat.w();

            // publish ground truth pose
            geometry_msgs::PoseStamped ps;
            ps.header.frame_id = "/map";
            ps.pose.position.x = state_blue.x;
            ps.pose.position.y = state_blue.y;
            ps.pose.orientation.x = quat.x();
            ps.pose.orientation.y = quat.y();
            ps.pose.orientation.z = quat.z();
            ps.pose.orientation.w = quat.w();

//            ROS_INFO_STREAM(state_blue.x);
//            ROS_INFO_STREAM(state_blue.y);

            // Add a header to the transformation
            geometry_msgs::TransformStamped ts;
            ts.transform = t;
            ts.header.stamp = timestamp;
            ts.header.frame_id = "/map";
            ts.child_frame_id = base_frame_blue;

            // Publish them
            if (broadcast_transform) {
                br.sendTransform(ts);
            }
            if (pub_gt_pose) {
                pose_pub.publish(ps);
            }
        }

        void pub_pose_transform_red(ros::Time timestamp) {
            // Convert the pose into a transformation
            geometry_msgs::Transform t;
            t.translation.x = state_red.x;
            t.translation.y = state_red.y;
            tf2::Quaternion quat;
            quat.setEuler(0., 0., state_red.theta);
            t.rotation.x = quat.x();
            t.rotation.y = quat.y();
            t.rotation.z = quat.z();
            t.rotation.w = quat.w();
    
            // publish ground truth pose
            geometry_msgs::PoseStamped ps;
            ps.header.frame_id = "/map";
            ps.pose.position.x = state_red.x;
            ps.pose.position.y = state_red.y;
            ps.pose.orientation.x = quat.x();
            ps.pose.orientation.y = quat.y();
            ps.pose.orientation.z = quat.z();
            ps.pose.orientation.w = quat.w();
    
    //            ROS_INFO_STREAM(state_red.x);
    //            ROS_INFO_STREAM(state_red.y);
    
            // Add a header to the transformation
            geometry_msgs::TransformStamped ts;
            ts.transform = t;
            ts.header.stamp = timestamp;
            ts.header.frame_id = "/map";
            ts.child_frame_id = base_frame_red;
    
            // Publish them
            if (broadcast_transform) {
                br.sendTransform(ts);
            }
            if (pub_gt_pose) {
                pose_pub.publish(ps);
            }
        }
        
        void pub_steer_ang_transform_blue(ros::Time timestamp) {
            // Set the steering angle to make the wheels move
            // Publish the steering angle
            tf2::Quaternion quat_wheel;
            quat_wheel.setEuler(0., 0., state_blue.steer_angle);
            geometry_msgs::TransformStamped ts_wheel;
            ts_wheel.transform.rotation.x = quat_wheel.x();
            ts_wheel.transform.rotation.y = quat_wheel.y();
            ts_wheel.transform.rotation.z = quat_wheel.z();
            ts_wheel.transform.rotation.w = quat_wheel.w();
            ts_wheel.header.stamp = timestamp;
            ts_wheel.header.frame_id = "blue/front_left_hinge";
            ts_wheel.child_frame_id = "blue/front_left_wheel";
            br.sendTransform(ts_wheel);
            ts_wheel.header.frame_id = "blue/front_right_hinge";
            ts_wheel.child_frame_id = "blue/front_right_wheel";
            br.sendTransform(ts_wheel);
        }
    
        void pub_steer_ang_transform_red(ros::Time timestamp) {
            // Set the steering angle to make the wheels move
            // Publish the steering angle
            tf2::Quaternion quat_wheel;
            quat_wheel.setEuler(0., 0., state_red.steer_angle);
            geometry_msgs::TransformStamped ts_wheel;
            ts_wheel.transform.rotation.x = quat_wheel.x();
            ts_wheel.transform.rotation.y = quat_wheel.y();
            ts_wheel.transform.rotation.z = quat_wheel.z();
            ts_wheel.transform.rotation.w = quat_wheel.w();
            ts_wheel.header.stamp = timestamp;
            ts_wheel.header.frame_id = "red/front_left_hinge";
            ts_wheel.child_frame_id = "red/front_left_wheel";
            br.sendTransform(ts_wheel);
            ts_wheel.header.frame_id = "red/front_right_hinge";
            ts_wheel.child_frame_id = "red/front_right_wheel";
            br.sendTransform(ts_wheel);
        }

        void pub_laser_link_transform_blue(ros::Time timestamp) {
            // Publish a transformation between base link and laser
            geometry_msgs::TransformStamped scan_ts;
            scan_ts.transform.translation.x = scan_distance_to_base_link;
            scan_ts.transform.rotation.w = 1;
            scan_ts.header.stamp = timestamp;
            scan_ts.header.frame_id = base_frame_blue;
            scan_ts.child_frame_id = scan_frame_blue;
            br.sendTransform(scan_ts);
        }

    void pub_laser_link_transform_red(ros::Time timestamp) {
        // Publish a transformation between base link and laser
        geometry_msgs::TransformStamped scan_ts;
        scan_ts.transform.translation.x = scan_distance_to_base_link;
        scan_ts.transform.rotation.w = 1;
        scan_ts.header.stamp = timestamp;
        scan_ts.header.frame_id = base_frame_red;
        scan_ts.child_frame_id = scan_frame_red;
        br.sendTransform(scan_ts);
    }

        void pub_odom_blue(ros::Time timestamp) {
            // Make an odom message and publish it
            nav_msgs::Odometry odom;
            odom.header.stamp = timestamp;
            odom.header.frame_id = map_frame;
            odom.child_frame_id = base_frame_blue;
            odom.pose.pose.position.x = state_blue.x;
            odom.pose.pose.position.y = state_blue.y;
            tf2::Quaternion quat;
            quat.setEuler(0., 0., state_blue.theta);
            odom.pose.pose.orientation.x = quat.x();
            odom.pose.pose.orientation.y = quat.y();
            odom.pose.pose.orientation.z = quat.z();
            odom.pose.pose.orientation.w = quat.w();
            odom.twist.twist.linear.x = state_blue.velocity;
            odom.twist.twist.angular.z = state_blue.angular_velocity;
            odom_pub.publish(odom);
        }

        void pub_odom_red(ros::Time timestamp) {
            // Make an odom message and publish it
            nav_msgs::Odometry odom;
            odom.header.stamp = timestamp;
            odom.header.frame_id = map_frame;
            odom.child_frame_id = base_frame_red;
            odom.pose.pose.position.x = state_red.x;
            odom.pose.pose.position.y = state_red.y;
            tf2::Quaternion quat;
            quat.setEuler(0., 0., state_red.theta);
            odom.pose.pose.orientation.x = quat.x();
            odom.pose.pose.orientation.y = quat.y();
            odom.pose.pose.orientation.z = quat.z();
            odom.pose.pose.orientation.w = quat.w();
            odom.twist.twist.linear.x = state_red.velocity;
            odom.twist.twist.angular.z = state_red.angular_velocity;
            odom_pub.publish(odom);
        }
    
        void pub_imu(ros::Time timestamp) {
            // Make an IMU message and publish it
            // TODO: make imu message
            sensor_msgs::Imu imu;
            imu.header.stamp = timestamp;
            imu.header.frame_id = map_frame;


            imu_pub.publish(imu);
        }

};


int main(int argc, char ** argv) {
    ros::init(argc, argv, "racecar_simulator");
    RacecarSimulator rs;
    ros::spin();
    return 0;
}
