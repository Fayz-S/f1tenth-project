#include <ros/ros.h>
#include <ros/package.h>

#include <interactive_markers/interactive_marker_server.h>
#include <tf2/impl/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "f1tenth_simulator/pose_2d.hpp"
#include "f1tenth_simulator/ackermann_kinematics.hpp"
#include "f1tenth_simulator/scan_simulator_2d.hpp"

#include "f1tenth_simulator/car_state.hpp"
#include "f1tenth_simulator/car_params.hpp"
#include "f1tenth_simulator/vehicle_model.hpp"
#include "f1tenth_simulator/precompute.hpp"

#include <iostream>
#include <fstream>
#include <math.h>
#include <ctime>
#include <string>

using namespace racecar_simulator;

class RacecarSimulator {
private:
    // A ROS node
    ros::NodeHandle n;

    double update_pose_rate;

    // For publishing transformations
    tf2_ros::TransformBroadcaster br;

    std::string drive_topic_blue, drive_topic_red, scan_topic_blue, scan_topic_red, pose_topic_blue, pose_topic_red,
                map_topic, gt_pose_topic,pose_rviz_topic, odom_topic, imu_topic, data_topic, reference_line,
                carState_topic_red, switch_topic_red;

    // The transformation frames used
    std::string map_frame, base_frame_blue, scan_frame_blue, base_frame_red, scan_frame_red;

    // The car state_blue and parameters
    CarState state_blue, state_red;
    CarParams params_blue, params_red;
    double desired_speed_blue, desired_steer_ang_blue, desired_speed_red, desired_steer_ang_red;
    double previous_seconds_blue, previous_seconds_red;
    double scan_distance_to_base_link;
    double cube_width;
    double width;

    // A timer to update the pose
    ros::Timer update_pose_timer_blue, update_pose_timer_red;

    // A simulator of the laser
    ScanSimulator2D scan_simulator;

    // Publish a scan, odometry, and imu data
    bool broadcast_transform;
    bool pub_gt_pose;

    ros::Publisher scan_pub_blue, scan_pub_red;
    ros::Publisher pose_pub;
    ros::Publisher carState_pub_red;
    ros::Publisher switch_pub_red;
    ros::Publisher odom_pub;
    ros::Publisher imu_pub;
    ros::Publisher reference_line_pub;

    ros::Subscriber drive_sub_blue, drive_sub_red;
    ros::Subscriber pose_sub_blue, pose_sub_red;
    ros::Subscriber data_sub;

    // publisher for map with obstacles
    ros::Publisher map_pub;
    // Listen for a map
    ros::Subscriber map_sub;
    double map_free_threshold;
    bool map_exists = false;
    // keep an original map for obstacles
    nav_msgs::OccupancyGrid original_map;
    nav_msgs::OccupancyGrid current_map;
    // name of current map
    std::string map_name;
    int map_width, map_height;
    double map_resolution, origin_x, origin_y;

    // precompute cosines of scan angles
    std::vector<double> cosines;
    // precompute distance from lidar to edge of car for each beam
    std::vector<double> car_distances;
    // for collision check
    bool TTC = false;
    double ttc_threshold;

    // scan parameters
    double scan_fov;
    double scan_ang_incr;
    int scan_beams;
    double scan_std_dev;
    double scan_max_range;

    // flag that indicate start or stop recording data
    bool log_data_flag = 0;
    // the path where save data
    char *path = "/media/psf/Ubuntu";
    // save vehicle state
    std::vector<std::string> car_state_blue, car_state_red;
    // save data for machine learning
    std::vector<std::string> steering_gas_lidar_blue;

public:
    RacecarSimulator() {
        // Initialize the node handle
        n = ros::NodeHandle("~");

        // Get the topic names
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

        // Get the transformation frame names
        n.getParam("map_frame", map_frame);
        n.getParam("base_frame_blue", base_frame_blue);
        n.getParam("scan_frame_blue", scan_frame_blue);
        n.getParam("base_frame_red", base_frame_red);
        n.getParam("scan_frame_red", scan_frame_red);


        n.getParam("update_pose_rate", update_pose_rate);
        n.getParam("scan_beams", scan_beams);
        n.getParam("scan_field_of_view", scan_fov);
        n.getParam("scan_std_dev", scan_std_dev);
        n.getParam("scan_max_range", scan_max_range);
        n.getParam("map_free_threshold", map_free_threshold);
        n.getParam("scan_distance_to_base_link", scan_distance_to_base_link);

        n.getParam("width", width);

        n.getParam("wheelbase", params_blue.wheelbase);
        n.getParam("friction_coeff", params_blue.friction_coeff);
        n.getParam("height_cg", params_blue.h_cg);
        n.getParam("l_cg2rear", params_blue.l_r);
        n.getParam("l_cg2front", params_blue.l_f);
        n.getParam("C_S_front", params_blue.cs_f);
        n.getParam("C_S_rear", params_blue.cs_r);
        n.getParam("moment_inertia", params_blue.Iz);
        n.getParam("mass", params_blue.mass);
        n.getParam("empirical_drivetrain_parameters_1", params_blue.Cm1);
        n.getParam("empirical_drivetrain_parameters_2", params_blue.Cm2);
        n.getParam("empirical_drivetrain_parameters_3", params_blue.Cm3);
        n.getParam("empirical_Pacejka_parameters_B_f", params_blue.B_f);
        n.getParam("empirical_Pacejka_parameters_C_f", params_blue.C_f);
        n.getParam("empirical_Pacejka_parameters_D_f", params_blue.D_f);
        n.getParam("empirical_Pacejka_parameters_B_r", params_blue.B_r);
        n.getParam("empirical_Pacejka_parameters_C_r", params_blue.C_r);
        n.getParam("empirical_Pacejka_parameters_D_r", params_blue.D_r);
        n.getParam("max_speed", params_blue.max_speed);
        n.getParam("max_steering_angle", params_blue.max_steering_angle);
        n.getParam("max_steering_vel", params_blue.max_steering_vel);
        n.getParam("max_accel", params_blue.max_accel);
        n.getParam("max_decel", params_blue.max_decel);

        n.getParam("wheelbase", params_red.wheelbase);
        n.getParam("friction_coeff", params_red.friction_coeff);
        n.getParam("height_cg", params_red.h_cg);
        n.getParam("l_cg2rear", params_red.l_r);
        n.getParam("l_cg2front", params_red.l_f);
        n.getParam("C_S_front", params_red.cs_f);
        n.getParam("C_S_rear", params_red.cs_r);
        n.getParam("moment_inertia", params_red.Iz);
        n.getParam("mass", params_red.mass);
        n.getParam("empirical_drivetrain_parameters_1", params_red.Cm1);
        n.getParam("empirical_drivetrain_parameters_2", params_red.Cm2);
        n.getParam("empirical_drivetrain_parameters_3", params_red.Cm3);
        n.getParam("empirical_Pacejka_parameters_B_f", params_red.B_f);
        n.getParam("empirical_Pacejka_parameters_C_f", params_red.C_f);
        n.getParam("empirical_Pacejka_parameters_D_f", params_red.D_f);
        n.getParam("empirical_Pacejka_parameters_B_r", params_red.B_r);
        n.getParam("empirical_Pacejka_parameters_C_r", params_red.C_r);
        n.getParam("empirical_Pacejka_parameters_D_r", params_red.D_r);
        n.getParam("max_speed", params_red.max_speed);
        n.getParam("max_steering_angle", params_red.max_steering_angle);
        n.getParam("max_steering_vel", params_red.max_steering_vel);
        n.getParam("max_accel", params_red.max_accel);
        n.getParam("max_decel", params_red.max_decel);

        n.getParam("data_topic", data_topic);
        n.getParam("carState_topic_red", carState_topic_red);
        n.getParam("reference_line", reference_line);
        n.getParam("switch_topic_red", switch_topic_red);
        n.getParam("map_name", map_name);

        // Determine if we should broadcast
        n.getParam("broadcast_transform", broadcast_transform);
        n.getParam("publish_ground_truth_pose", pub_gt_pose);
        n.getParam("ttc_threshold", ttc_threshold);

        // monaco x:16 y:-2 t:0
        // de-espana x:18 y:31 t:3.14
        // Malaysian x:18 y:31 t:3.14
        state_blue = {.x=18, .y=31, .theta=3.14, .velocity_x=0, .velocity_y=0, .steer_angle=0.0, .angular_velocity=0.0, .slip_angle=0.0, .st_dyn=false};
        desired_speed_blue = 0.0;
        desired_steer_ang_blue = 0.0;

        // monaco x:10 y:-1 t:0
        // de-espana x:22 y:30.5 t:3.14
        // Malaysian x:22 y:27.5 t:3.14
        state_red = {.x=22, .y=30.5, .theta=3.14, .velocity_x=0, .velocity_y=0, .steer_angle=0.0, .angular_velocity=0.0, .slip_angle=0.0, .st_dyn=false};
        desired_speed_red = 0.0;
        desired_steer_ang_red = 0.0;

        previous_seconds_blue = ros::Time::now().toSec();
        previous_seconds_red = ros::Time::now().toSec();

        // Start a timer to output the pose
        update_pose_timer_blue = n.createTimer(ros::Duration(update_pose_rate), &RacecarSimulator::update_pose_blue, this);
        update_pose_timer_red = n.createTimer(ros::Duration(update_pose_rate), &RacecarSimulator::update_pose_red, this);

        // Start a subscriber to listen to drive commands
        drive_sub_blue = n.subscribe(drive_topic_blue, 1, &RacecarSimulator::drive_callback_blue, this);
        drive_sub_red = n.subscribe(drive_topic_red, 1, &RacecarSimulator::drive_callback_red, this);

        // Initialize a simulator of the laser scanner
        scan_simulator = ScanSimulator2D(scan_beams, scan_fov, scan_std_dev, scan_max_range, cube_width);

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

        carState_pub_red = n.advertise<std_msgs::String>(carState_topic_red, 1);

        switch_pub_red = n.advertise<std_msgs::Bool>(switch_topic_red, 1);

        reference_line_pub = n.advertise<visualization_msgs::Marker>(reference_line, 0);

        // Start a subscriber to listen to new maps
        map_sub = n.subscribe(map_topic, 1, &RacecarSimulator::map_callback, this);

        // Start a subscriber to listen to pose messages
        pose_sub_blue = n.subscribe(pose_topic_blue, 1, &RacecarSimulator::pose_callback_blue, this);
        pose_sub_red = n.subscribe(pose_topic_red, 1, &RacecarSimulator::pose_callback_red, this);

        data_sub = n.subscribe(data_topic, 1, &RacecarSimulator::data_callback, this);


        scan_ang_incr = scan_simulator.get_angle_increment();

        cosines = Precompute::get_cosines(scan_beams, -scan_fov / 2.0, scan_ang_incr);
        car_distances = Precompute::get_car_distances(scan_beams, params_blue.wheelbase, width,
                                                      scan_distance_to_base_link, -scan_fov / 2.0, scan_ang_incr);

        // wait for one map message to get the map data array
        boost::shared_ptr<nav_msgs::OccupancyGrid const> map_ptr;
        nav_msgs::OccupancyGrid map_msg;
        map_ptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(map_topic);
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


        ROS_INFO("Simulator constructed.");
    }

    /**
     * main loop and the core of the simulator
     * First, update car state
     * Second, update LiDAR scan data
     * Third, using the LiDAR data to check collision
     */
    void update_pose_blue(const ros::TimerEvent &) {
        // Update the car state
        ros::Time timestamp = ros::Time::now();
        double current_seconds = timestamp.toSec();

        state_blue = STKinematics::update(
                state_blue,
                desired_speed_blue,
                desired_steer_ang_blue,
                params_blue,
                current_seconds - previous_seconds_blue);

        state_blue.velocity_x = std::min(std::max(state_blue.velocity_x, -params_blue.max_speed), params_blue.max_speed);
        state_blue.steer_angle = std::min(std::max(state_blue.steer_angle, -params_blue.max_steering_angle), params_blue.max_steering_angle);

//        ROS_INFO_STREAM("V blue "<<state_blue.velocity_x);

        previous_seconds_blue = current_seconds;

        // Publish the pose as a transformation
        pub_pose_transform_blue(timestamp);

        // Publish the steering angle as a transformation so the wheels move
        pub_steer_ang_transform_blue(timestamp);

        // Make an odom message as well and publish it
        pub_odom_blue(timestamp);

        // If we have a map, perform a scan
        if (map_exists) {
            // calculating the pose of the lidar, given the pose of base link
            // (base link is the center of the rear axle)
            Pose2D scan_pose;
            scan_pose.x = state_blue.x + scan_distance_to_base_link * std::cos(state_blue.theta);
            scan_pose.y = state_blue.y + scan_distance_to_base_link * std::sin(state_blue.theta);
            scan_pose.theta = state_blue.theta;

            // we need the pose of opponent car for simulating LiDAR data
            Pose2D opponent_pose;
            opponent_pose.x = state_red.x;
            opponent_pose.y = state_red.y;
            opponent_pose.theta = state_red.theta;

            // Compute the scan from the lidar
            std::vector<double> scan = scan_simulator.scan(scan_pose, opponent_pose, false);

            // Convert to float
            std::vector<float> scan_(scan.size());
            // concat scan to a single string (this is for gathering data for machine learning, not necessarily)
            std::string scan_string;

            for (size_t i = 0; i < scan.size(); i++) {
                scan_[i] = scan[i];
                scan_string += std::to_string(scan_[i]) + ",";
            }

            // In order to implement box collider for both cars, which treating each car as a box or a rectangle in this 2D world
            // We can simplify the problem into decide whether two rectangle is overlapping or not
            // To solve this problem, we can check whether each point of a rectangle is IN another rectangle or not
            // to do this, first, we need the coordinate of all corner points of two rectangles (red and blue)
            // second, use the vector cross product https://stackoverflow.com/questions/2752725/finding-whether-a-point-lies-inside-a-rectangle-or-not
            // third, loop four points, if none of them in another rectangle, we can say there is no collision between two cars
            // all four points of blue car
            // front-right
            double x1_blue = scan_pose.x - width / 2 * std::cos(state_blue.theta + M_PI / 2);
            double y1_blue = scan_pose.y + width / 2 * std::sin(state_blue.theta + M_PI / 2);
            // front-left
            double x2_blue = scan_pose.x + width / 2 * std::cos(state_blue.theta + M_PI / 2);
            double y2_blue = scan_pose.y - width / 2 * std::sin(state_blue.theta + M_PI / 2);
            // rear-left
            double x3_blue = state_blue.x + width / 2 * std::cos(state_blue.theta + M_PI / 2);
            double y3_blue = state_blue.y - width / 2 * std::sin(state_blue.theta + M_PI / 2);
            // rear-right
            double x4_blue = state_blue.x - width / 2 * std::cos(state_blue.theta + M_PI / 2);
            double y4_blue = state_blue.y + width / 2 * std::sin(state_blue.theta + M_PI / 2);

            // add them to a vector
            std::vector<std::vector<double>> points_blue = {{x1_blue, y1_blue},
                                                            {x2_blue, y2_blue},
                                                            {x3_blue, y3_blue},
                                                            {x4_blue, y4_blue}};

            // all four points of the opponent car(red), calculation same as above
            double x1_red = state_red.x + params_red.wheelbase * std::cos(state_red.theta) -
                            width / 2 * std::cos(state_red.theta + M_PI / 2);
            double y1_red = state_red.y + params_red.wheelbase * std::sin(state_red.theta) +
                            width / 2 * std::sin(state_red.theta + M_PI / 2);

            double x2_red = state_red.x + params_red.wheelbase * std::cos(state_red.theta) +
                            width / 2 * std::cos(state_red.theta + M_PI / 2);
            double y2_red = state_red.y + params_red.wheelbase * std::sin(state_red.theta) -
                            width / 2 * std::sin(state_red.theta + M_PI / 2);

            double x3_red = state_red.x + width / 2 * std::cos(state_red.theta + M_PI / 2);
            double y3_red = state_red.y - width / 2 * std::sin(state_red.theta + M_PI / 2);

            double x4_red = state_red.x - width / 2 * std::cos(state_red.theta + M_PI / 2);
            double y4_red = state_red.y + width / 2 * std::sin(state_red.theta + M_PI / 2);

            // TTC Calculations are done here so the car can be halted in the simulator:
            // to reset TTC
            bool no_collision = true;

            if (state_blue.velocity_x != 0) {
                for (size_t i = 0; i < scan_.size(); i++) {
                    // TTC calculations
                    // calculate projected velocity
                    double proj_velocity = state_blue.velocity_x * cosines[i];
                    double ttc = (scan_[i] - car_distances[i]) / proj_velocity;
                    // if it's small enough to count as a collision
                    if ((ttc < ttc_threshold) && (ttc >= 0.0)) {
                        if (!TTC) {
                            first_ttc_actions_blue();
                        }

                        no_collision = false;
                        TTC = true;

                        ROS_INFO("LiDAR collision detected: BLUE");
                    }
                }
                // loop four points of blue car
                for (int i = 0; i < 4; ++i) {
                    if ((vector_cross(x1_red, y1_red, x2_red, y2_red, points_blue[i][0], points_blue[i][1]) *
                         vector_cross(x3_red, y3_red, x4_red, y4_red, points_blue[i][0], points_blue[i][1]) >= 0) &&
                        (vector_cross(x2_red, y2_red, x3_red, y3_red, points_blue[i][0], points_blue[i][1]) *
                         vector_cross(x4_red, y4_red, x1_red, y1_red, points_blue[i][0], points_blue[i][1]) >= 0)) {
                        if (!TTC) {
                            first_ttc_actions_blue();
                        }
                        no_collision = false;
                        TTC = true;

                        ROS_INFO("Box collider detected: BLUE");
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
            scan_msg.angle_min = -scan_simulator.get_field_of_view() / 2.;
            scan_msg.angle_max = scan_simulator.get_field_of_view() / 2.;
            scan_msg.angle_increment = scan_simulator.get_angle_increment();
            scan_msg.range_max = 100;
            scan_msg.ranges = scan_;
            scan_msg.intensities = scan_;
            scan_pub_blue.publish(scan_msg);

            // Publish a transformation between base link and laser
            pub_laser_link_transform_blue(timestamp);
            // for machine learning, not necessarily
            save_data_blue(scan_string);
        }
        publish_reference_line();
    }

    /**
     * basically same as updata_pose_blue
     */
    void update_pose_red(const ros::TimerEvent &) {
        // Update the pose
        ros::Time timestamp = ros::Time::now();
        double current_seconds = timestamp.toSec();

        state_red = STKinematics::update(
                state_red,
                desired_speed_red,
                desired_steer_ang_red,
                params_red,
                current_seconds - previous_seconds_red);

        state_red.velocity_x = std::min(std::max(state_red.velocity_x, -params_red.max_speed), params_red.max_speed);
        state_red.steer_angle = std::min(std::max(state_red.steer_angle, -params_red.max_steering_angle), params_red.max_steering_angle);

//        ROS_INFO_STREAM("V red "<<state_red.velocity_x);

        previous_seconds_red = current_seconds;
        // for machine learning, not necessarily
        pub_carState_red(toString(state_red));

        /// Publish the pose as a transformation
        pub_pose_transform_red(timestamp);

        /// Publish the steering angle as a transformation so the wheels move
        pub_steer_ang_transform_red(timestamp);

        // Make an odom message as well and publish it
        pub_odom_red(timestamp);


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
            std::vector<double> scan = scan_simulator.scan(scan_pose, opponent_pose, true);

            // Convert to float
            std::vector<float> scan_(scan.size());
            for (size_t i = 0; i < scan.size(); i++)
                scan_[i] = scan[i];

            // same as above
            double x1_red = scan_pose.x - width / 2 * std::cos(state_red.theta + M_PI / 2);
            double y1_red = scan_pose.y + width / 2 * std::sin(state_red.theta + M_PI / 2);

            double x2_red = scan_pose.x + width / 2 * std::cos(state_red.theta + M_PI / 2);
            double y2_red = scan_pose.y - width / 2 * std::sin(state_red.theta + M_PI / 2);

            double x3_red = state_red.x + width / 2 * std::cos(state_red.theta + M_PI / 2);
            double y3_red = state_red.y - width / 2 * std::sin(state_red.theta + M_PI / 2);

            double x4_red = state_red.x - width / 2 * std::cos(state_red.theta + M_PI / 2);
            double y4_red = state_red.y + width / 2 * std::sin(state_red.theta + M_PI / 2);

            std::vector<std::vector<double>> points_red = {{x1_red, y1_red},
                                                           {x2_red, y2_red},
                                                           {x3_red, y3_red},
                                                           {x4_red, y4_red}};

            double x1_blue = state_blue.x + params_blue.wheelbase * std::cos(state_blue.theta) -
                             width / 2 * std::cos(state_blue.theta + M_PI / 2);
            double y1_blue = state_blue.y + params_blue.wheelbase * std::sin(state_blue.theta) +
                             width / 2 * std::sin(state_blue.theta + M_PI / 2);

            double x2_blue = state_blue.x + params_blue.wheelbase * std::cos(state_blue.theta) +
                             width / 2 * std::cos(state_blue.theta + M_PI / 2);
            double y2_blue = state_blue.y + params_blue.wheelbase * std::sin(state_blue.theta) -
                             width / 2 * std::sin(state_blue.theta + M_PI / 2);

            double x3_blue = state_blue.x + width / 2 * std::cos(state_blue.theta + M_PI / 2);
            double y3_blue = state_blue.y - width / 2 * std::sin(state_blue.theta + M_PI / 2);

            double x4_blue = state_blue.x - width / 2 * std::cos(state_blue.theta + M_PI / 2);
            double y4_blue = state_blue.y + width / 2 * std::sin(state_blue.theta + M_PI / 2);


            // TTC Calculations are done here so the car can be halted in the simulator:
            // detection based on the scan data
            bool no_collision = true;
            if (state_red.velocity_x != 0) {
                for (size_t i = 0; i < scan_.size(); i++) {
                    // TTC calculations
                    // calculate projected velocity
                    // the vector of velocity can be seen as always point to the middle beam, and cosines here is the cos of beams not cos of map frame,
                    // hence, the middle beam direction has cos0 = 1, and so on.
                    double proj_velocity = state_red.velocity_x * cosines[i];
                    double ttc = (scan_[i] - car_distances[i]) / proj_velocity;
                    // if it's small enough to count as a collision
                    if ((ttc < ttc_threshold) && (ttc >= 0.0)) {
                        if (!TTC) {
                            first_ttc_actions_red();
                        }

                        no_collision = false;
                        TTC = true;

                        ROS_INFO("LiDAR collision detected: RED");
                    }
                }
                for (int i = 0; i < 4; ++i) {
                    if ((vector_cross(x1_blue, y1_blue, x2_blue, y2_blue, points_red[i][0], points_red[i][1]) *
                         vector_cross(x3_blue, y3_blue, x4_blue, y4_blue, points_red[i][0], points_red[i][1]) >= 0) &&
                        (vector_cross(x2_blue, y2_blue, x3_blue, y3_blue, points_red[i][0], points_red[i][1]) *
                         vector_cross(x4_blue, y4_blue, x1_blue, y1_blue, points_red[i][0], points_red[i][1]) >= 0)) {
                        if (!TTC) {
                            first_ttc_actions_red();
                        }
                        no_collision = false;
                        TTC = true;

                        ROS_INFO("Box collider detected: RED");
                    }
                }
            }

            // reset TTC
            if (no_collision)
                TTC = false;

            // this is the flag for switching between MPC and overtaking algorithm
            // if red car can see blue car within a certain distance, then using overtaking algorithm, otherwise MPC
            std_msgs::Bool msg;
            msg.data = scan_simulator.see_opponent();
            switch_pub_red.publish(msg);

            // Publish the laser message
            sensor_msgs::LaserScan scan_msg;
            scan_msg.header.stamp = timestamp;
            scan_msg.header.frame_id = scan_frame_red;
            scan_msg.angle_min = -scan_simulator.get_field_of_view() / 2.;
            scan_msg.angle_max = scan_simulator.get_field_of_view() / 2.;
            scan_msg.angle_increment = scan_simulator.get_angle_increment();
            scan_msg.range_max = 100;
            scan_msg.ranges = scan_;
            scan_msg.intensities = scan_;
            scan_pub_red.publish(scan_msg);

            // Publish a transformation between base link and laser
            pub_laser_link_transform_red(timestamp);

            save_data_red();
        }
    }

    /// ---------------------- GENERAL HELPER FUNCTIONS ----------------------
    double vector_cross(double x1, double y1, double x2, double y2, double x, double y) {
        return (x2 - x1) * (y - y1) - (x - x1) * (y2 - y1);
    }

    void save_data_blue(std::string scan) {
        // if the flag is true, then record data to vector, but not write to file yet
        // note that the frequency of recording is same as update_pose_rate, which depends on how fast your machine is as well as the storage
        if (log_data_flag) {
            std::string ml_data = std::to_string(desired_speed_blue) + "," + std::to_string(desired_steer_ang_blue) + "," + scan;
            steering_gas_lidar_blue.push_back(ml_data);

            std::string current_car_state = toString(state_blue);
            car_state_blue.push_back(current_car_state);
        } else {
            // if the flag is false, then write data to files
            // But we also need to check whether the data vector is empty or not,
            // like the flag is false when simulator started, but there is nothing to write
            if (!steering_gas_lidar_blue.empty()) {
                // use current time to distinct different files
                // this is very helpful since you can match three files by sorting them in name order
                time_t now = time(0);
                std::string date(std::ctime(&now));
                std::ofstream file_blue(path + std::string("/ML_dataset_blue") + date + std::string(".csv"));

                // if the file can be created
                if (file_blue.is_open()) {
                    ROS_WARN("Starting writing ML data (blue)");

                    // write heading first
                    file_blue << "Speed,Steering_angle,LiDAR_scan\n";
                    // save every rows
                    for (std::string s: steering_gas_lidar_blue) {
                        file_blue << s + "\n";
                    }

                    // close the file
                    file_blue.close();
                    // clear the vector
                    steering_gas_lidar_blue.clear();
                    ROS_WARN("Finishing writing data to %s/ML_dataset_blue%s.csv", path, std::ctime(&now));
                } else {
                    ROS_ERROR("Cannot create a file (blue)");
                }
            }
            if (!car_state_blue.empty()) {
                time_t now = time(0);
                std::string date(std::ctime(&now));
                std::ofstream file_blue(path + std::string("/car_state_blue_") + date + std::string(".csv"));

                if (file_blue.is_open()) {
                    ROS_WARN("Starting writing data (blue)");

                    // heading
                    file_blue << "Position_X,Position_Y,Theta,Velocity_X,Velocity_Y,Steering_angle,Angular_velocity,slip_angle\n";
                    for (std::string s: car_state_blue) {
                        file_blue << s + "\n";
                    }

                    file_blue.close();
                    car_state_blue.clear();
                    ROS_WARN("Finishing writing data to %s/car_state_blue_%s.csv", path, std::ctime(&now));
                } else {
                    ROS_ERROR("Cannot create a file (blue)");
                }
            }
        }
    }

    void save_data_red() {
        // same as above
        if (log_data_flag) {
            std::string current_car_state = toString(state_red);
            car_state_red.push_back(current_car_state);
        } else {
            if (!car_state_red.empty()) {
                time_t now = time(0);
                std::string date(std::ctime(&now));
                std::ofstream file_red(path + std::string("/car_state_red_") + date + std::string(".csv"));
                if (file_red.is_open()) {
                    ROS_WARN("Starting writing data (red)");
                    // heading
                    file_red << "Position_X,Position_Y,Theta,Velocity_X,Velocity_Y,Steering_angle,Angular_velocity,slip_angle\n";
                    for (std::string s: car_state_red) {
                        file_red << s + "\n";
                    }
                    file_red.close();
                    car_state_red.clear();
                    ROS_WARN("Finishing writing data to %s/car_state_red_%s.csv", path, std::ctime(&now));
                } else {
                    ROS_ERROR("Cannot create a file (red)");
                }
            }
        }
    }

    std::string toString(CarState & carState) {
        return std::to_string(carState.x)                + ","
             + std::to_string(carState.y)                + ","
             + std::to_string(carState.theta)            + ","
             + std::to_string(carState.velocity_x)       + ","
             + std::to_string(carState.velocity_y)       + ","
             + std::to_string(carState.steer_angle)      + ","
             + std::to_string(carState.angular_velocity) + ","
             + std::to_string(carState.slip_angle);
    }

    void first_ttc_actions_blue() {
        // completely stop vehicle
        state_blue.velocity_x = 0.0;
        state_blue.velocity_y = 0.0;
        state_blue.angular_velocity = 0.0;
        state_blue.slip_angle = 0.0;
        state_blue.steer_angle = 0.0;
        desired_speed_blue = 0.0;
        desired_steer_ang_blue = 0.0;
    }

    void first_ttc_actions_red() {
        // completely stop vehicle
        state_red.velocity_x = 0.0;
        state_red.velocity_y = 0.0;
        state_red.angular_velocity = 0.0;
        state_red.slip_angle = 0.0;
        state_red.steer_angle = 0.0;
        desired_speed_red = 0.0;
        desired_steer_ang_red = 0.0;
    }

    /// ---------------------- CALLBACK FUNCTIONS ----------------------

    void data_callback(const std_msgs::Bool &msg) {
        log_data_flag = msg.data;
        if (log_data_flag) {
            ROS_INFO_STREAM("start logging driving data");
        } else {
            ROS_INFO_STREAM("stop logging driving data and save to file");
        }
    }

    void pose_callback_blue(const geometry_msgs::PoseStamped &msg) {
        state_blue.x = msg.pose.position.x;
        state_blue.y = msg.pose.position.y;
        geometry_msgs::Quaternion q = msg.pose.orientation;
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        state_blue.theta = tf2::impl::getYaw(quat);
    }

    void pose_callback_red(const geometry_msgs::PoseStamped &msg) {
        state_red.x = msg.pose.position.x;
        state_red.y = msg.pose.position.y;
        geometry_msgs::Quaternion q = msg.pose.orientation;
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        state_red.theta = tf2::impl::getYaw(quat);
    }

    void drive_callback_blue(const ackermann_msgs::AckermannDriveStamped &msg) {
        desired_speed_blue = msg.drive.speed;
        desired_steer_ang_blue = msg.drive.steering_angle;
    }

    void drive_callback_red(const ackermann_msgs::AckermannDriveStamped &msg) {
        desired_speed_red = msg.drive.speed;
        desired_steer_ang_red = msg.drive.steering_angle;
    }


    void map_callback(const nav_msgs::OccupancyGrid &msg) {
        // http://docs.ros.org/en/lunar/api/nav_msgs/html/msg/OccupancyGrid.html
        // Fetch the map parameters
        size_t height = msg.info.height;
        size_t width = msg.info.width;
        map_resolution = msg.info.resolution;
        // Convert the ROS origin to a pose
        Pose2D origin;
        // bottom right conner is the origin point
        origin.x = msg.info.origin.position.x;
        origin.y = msg.info.origin.position.y;

        geometry_msgs::Quaternion q = msg.info.origin.orientation;
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        origin.theta = tf2::impl::getYaw(quat);

        ROS_INFO_STREAM("map height: " << height);
        ROS_INFO_STREAM("map width: " << width);

        // msg.data [0, 100], convert to [0, 1]. 0 means definitely not occupied, 1 means definitely occupied
        std::vector<double> map(msg.data.size());

        for (size_t i = 0; i < height * width; i++) {
            if (msg.data[i] > 100 or msg.data[i] < 0) {
                map[i] = 0.5; // Unknown
            } else {
//                ROS_INFO_STREAM((int)msg.data[i]);
                map[i] = msg.data[i] / 100.;
            }
        }

        // Send the map to the scanner
        scan_simulator.set_map(map, height, width, map_resolution, origin, map_free_threshold);

        map_exists = true;
    }

    /// ---------------------- PUBLISHING HELPER FUNCTIONS ----------------------

    void publish_reference_line() {

        std::fstream readcsv(ros::package::getPath("f1tenth_simulator") + "/maps/" + map_name + "_minTime.csv");

        // https://wiki.ros.org/rviz/DisplayTypes/Marker
        visualization_msgs::Marker msg;
        msg.header.frame_id = map_frame;
        msg.header.stamp = ros::Time();
        msg.ns = "points";
        msg.id = 0;
        msg.type = visualization_msgs::Marker::SPHERE_LIST;
        msg.action = visualization_msgs::Marker::ADD;
        msg.pose.position.z = 0;
        msg.pose.orientation.w = 1.0;
        msg.scale.x = 0.1;
        msg.scale.y = 0.1;
        msg.scale.z = 0.1;
        msg.color.a = 1.0;
        msg.color.r = 0.96;
        msg.color.g = 0.82;
        msg.color.b = 0.4;

        // weight is the scale in map_to_centerline.py in Racetrack-Preparation
        // bias is offset value, need some attempts
        // Australia
//        int weight_x = 5;
//        int weight_y = 5;
//        int bias_x = -120;
//        int bias_y = 0;
        // Shanghai
//        int weight_x = 5;
//        int weight_y = 5;
//        int bias_x = 0;
//        int bias_y = -65.1;
        // Gulf-Air-Bahrain
//        int weight_x = 6;
//        int weight_y = 6;
//        int bias_x = 0;
//        int bias_y = -65.4;
        // Malaysian
//        int weight_x = 3;
//        int weight_y = 3;
//        int bias_x = 0;
//        int bias_y = -60;
        int weight_x = 3;
        int weight_y = 3;
        int bias_x = 0;
        int bias_y = -60;

        std::string line;
        // this is the heading
        getline(readcsv, line);

        while (getline(readcsv, line)) {
            std::vector<double> data_line;
            std::string raw_data;
            std::istringstream readstr(line);

            for (int i = 0; i < 7; ++i) {
                getline(readstr, raw_data, ';');
                data_line.push_back(atof(raw_data.c_str()));
            }

            geometry_msgs::Point p;
            p.x = data_line[1] * weight_x * map_resolution + origin_x + bias_x;
            p.y = -(data_line[2] * weight_y * map_resolution - origin_y + bias_y);

            msg.points.push_back(p);
        }

//            ROS_INFO_STREAM(msg);
        reference_line_pub.publish(msg);
    }

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
        ps.header.frame_id = map_frame;
        ps.pose.position.x = state_blue.x;
        ps.pose.position.y = state_blue.y;
        ps.pose.orientation.x = quat.x();
        ps.pose.orientation.y = quat.y();
        ps.pose.orientation.z = quat.z();
        ps.pose.orientation.w = quat.w();

        // Add a header to the transformation
        geometry_msgs::TransformStamped ts;
        ts.transform = t;
        ts.header.stamp = timestamp;
        ts.header.frame_id = map_frame;
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
        ps.header.frame_id = map_frame;
        ps.pose.position.x = state_red.x;
        ps.pose.position.y = state_red.y;
        ps.pose.orientation.x = quat.x();
        ps.pose.orientation.y = quat.y();
        ps.pose.orientation.z = quat.z();
        ps.pose.orientation.w = quat.w();

        // Add a header to the transformation
        geometry_msgs::TransformStamped ts;
        ts.transform = t;
        ts.header.stamp = timestamp;
        ts.header.frame_id = map_frame;
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
        odom.twist.twist.linear.x = state_blue.velocity_x;
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
        odom.twist.twist.linear.x = state_red.velocity_x;
        odom.twist.twist.angular.z = state_red.angular_velocity;
        odom_pub.publish(odom);
    }


    void pub_carState_red(std::string string){
        std_msgs::String msg;
        msg.data = string;
        carState_pub_red.publish(msg);
    }
};


int main(int argc, char ** argv) {
    ros::init(argc, argv, "racecar_simulator");
    RacecarSimulator rs;
    ros::spin();
    return 0;
}
