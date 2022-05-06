#include "f1tenth_simulator/pose_2d.hpp"
#include "f1tenth_simulator/scan_simulator_2d.hpp"
#include "f1tenth_simulator/distance_transform.hpp"

using namespace racecar_simulator;

ScanSimulator2D::ScanSimulator2D(
    int num_beams_, 
    double field_of_view_, 
    double scan_std_dev_, 
    double ray_tracing_epsilon_,
    int theta_discretization) 
  : num_beams(num_beams_),
    field_of_view(field_of_view_),
    scan_std_dev(scan_std_dev_),
    ray_tracing_epsilon(ray_tracing_epsilon_),
    theta_discretization(theta_discretization)
{
  // Initialize laser settings
  angle_increment = field_of_view/(num_beams - 1);

  // Initialize the output
  scan_output = std::vector<double>(num_beams);

  // Initialize the noise
  noise_generator = std::mt19937(std::random_device{}());
  noise_dist = std::normal_distribution<double>(0., scan_std_dev);

  // Precompute sines and cosines
  //
  theta_index_increment = theta_discretization * angle_increment/(2 * M_PI);
  sines = std::vector<double>(theta_discretization + 1);
  cosines = std::vector<double>(theta_discretization + 1);

  for (int i = 0; i <= theta_discretization; i++) {
      // calculate theta on the discretization from 0 to 2Pi
    double theta = (2 * M_PI * i)/((double) theta_discretization);
    // calculate sin and cos of that theta value
    sines[i] = std::sin(theta);
    cosines[i] = std::cos(theta);
  }
}

const std::vector<double> ScanSimulator2D::scan(const Pose2D & pose) {
  scan(pose, scan_output.data());
  return scan_output;
}

void ScanSimulator2D::scan(const Pose2D & pose, double * scan_data) {
  // Make theta discrete by mapping the range [-pi,pi] onto [0, theta_discretization)
  // field_of_view/2 = 3/4 PI
  // (pose.theta - field_of_view/2.)/(2 * M_PI) is to calculate the difference between current car orientation and lidar scan start orientation
  // the result range is [-3/8, 5/8) * theta_discretization
  double theta_index = theta_discretization * (pose.theta - field_of_view/2.)/(2 * M_PI);

  // Make sure it is wrapped into [0, theta_discretization)
  // fmod: mod% for float numbers, sign depends on the first number
  theta_index = std::fmod(theta_index, theta_discretization);
  while (theta_index < 0) theta_index += theta_discretization;

  // process through each beam (1081)
  for (int i = 0; i < num_beams; i++) {
    // Compute the distance to the nearest point
    scan_data[i] = trace_ray(pose.x, pose.y, theta_index);

    // Add Gaussian noise to the ray trace
    if (scan_std_dev > 0)
        scan_data[i] += noise_dist(noise_generator);

    // Increment the scan
    theta_index += theta_index_increment;
    // Make sure it stays in the range [0, theta_discretization)
    while (theta_index >= theta_discretization) 
      theta_index -= theta_discretization;
  }
}

double ScanSimulator2D::trace_ray(double x, double y, double theta_index) const {
  // Add 0.5 to make this operation round to ceil rather than floor
  // why?
  int theta_index_ = theta_index + 0.5;
  double s = sines[theta_index_];
  double c = cosines[theta_index_];

  // Initialize the distance to the nearest obstacle
  double distance_to_nearest = distance_transform(x, y);
  double total_distance = distance_to_nearest;

  while (distance_to_nearest > ray_tracing_epsilon) {
    // Move in the direction of the ray
    // by distance_to_nearest
    x += distance_to_nearest * c;
    y += distance_to_nearest * s;
    
    // Compute the nearest distance at that point
    distance_to_nearest = distance_transform(x, y);
    total_distance += distance_to_nearest;
  }

  return total_distance;
}

double ScanSimulator2D::distance_transform(double x, double y) const {
  // Convert the pose to a grid cell
  int cell = xy_to_cell(x, y);
  if (cell < 0) return 0;
  return dt[cell];
}

int ScanSimulator2D::xy_to_cell(double x, double y) const {
    int row, col;
    xy_to_row_col(x, y, &row, &col);
    return row_col_to_cell(row, col);
}

void ScanSimulator2D::xy_to_row_col(double x, double y, int * row, int * col) const {
    // distance between x or y and the bottom right conner of the map
    double x_trans = x - origin.x;
    double y_trans = y - origin.y;

    // Rotate the state into the map
    // usually the theta of map is 0, hence, sin = 0 and cos = 1
    // imagine the map is a rectangle, and the car is located in the map.
    // there are two coordinate of the car, one is about the ground truth frame of reference and another is about the map reference
    // x and y is the coordinate about the ground truth, but we want coordinate of the map.
    // better calculate these on a paper, just some high school math.
    double x_rot =   x_trans * origin_c + y_trans * origin_s;
    double y_rot = - x_trans * origin_s + y_trans * origin_c;

    // Clip the state to be a cell
    // if the car is located outside the map
    if (x_rot < 0 or x_rot >= width * resolution or y_rot < 0 or y_rot >= height * resolution) {
        *col = -1;
        *row = -1;
    } else {
        // transform coordinate of the car about the map into width and height
        *col = std::floor(x_rot/resolution);
        *row = std::floor(y_rot/resolution);
    }
}

int ScanSimulator2D::row_col_to_cell(int row, int col) const {
    // calculate how many full columns first, then plus the last column
    return row * width + col;
}

// overload for changing map on the fly
void ScanSimulator2D::set_map(const std::vector<double> & map, double free_threshold) {
  for (size_t i = 0; i < map.size(); i++) {
    if (0 <= map[i] and map[i] <= free_threshold) {
      dt[i] = 99999; // Free
    } else {
      dt[i] = 0; // Occupied
    }
  }
  DistanceTransform::distance_2d(dt, width, height, resolution);
}

void ScanSimulator2D::set_map(
        const std::vector<double> & map,
        size_t height_,
        size_t width_,
        double resolution_,
        const Pose2D & origin_,
        double free_threshold) {

    // Assign parameters
    height = height_;
    width = width_;
    resolution = resolution_;
    origin = origin_;
    origin_c = std::cos(origin.theta);
    origin_s = std::sin(origin.theta);

    // Threshold the map
    dt = std::vector<double>(map.size());
    for (size_t i = 0; i < map.size(); i++) {
        if (0 <= map[i] and map[i] <= free_threshold) {
            dt[i] = 99999; // Free
        } else {
            dt[i] = 0; // Occupied
        }
    }
    // this is to calculate the distance from each pixel to the nearest occupied pixel in coordinate
    // so the elements in dt vector represent the distance where we can directly use them.
    DistanceTransform::distance_2d(dt, width, height, resolution);
}
