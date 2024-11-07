#include "cbmp.h"
#include "cmath"
#include <unordered_map>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

#define PI 3.14159265358979323846

CBM_Pred::CBM_Pred() : nh_(), private_nh_("~"), MAX_PREDICTION_SCORE_(1.0){
  private_nh_.param<double>("interval_sec", interval_sec_, 0.1);
  private_nh_.param<int>("num_prediction", num_prediction_, 10);
  private_nh_.param<double>("filter_out_close_object_threshold", filter_out_close_object_threshold_, 1.5);
  private_nh_.param<double>("filter_out_far_object_threshold", filter_out_far_object_threshold_, 10.0);

  private_nh_.param<double>("k", k_, 0.5);
  private_nh_.param<double>("Kp", Kp_, 1.0);
  private_nh_.param<double>("L", L_, 2.720);
  private_nh_.param<double>("max_steer", max_steer_, 120.0);
  private_nh_.param<double>("max_accel", max_accel_, 3.0);
  private_nh_.param<double>("min_accel", min_accel_, -2.0);

  // Subscriber
  current_pose_sub_ = nh_.subscribe("/current_pose", 10, &CBM_Pred::currentPoseCallback, this);
  current_vel_sub_ = nh_.subscribe("/current_velocity", 10, &CBM_Pred::currentVelocityCallback, this);
  detected_objects_sub_ = nh_.subscribe("/detection/objects", 10, &CBM_Pred::objectsCallback, this);
  
  // Publisher
  predicted_objects_pub_ = nh_.advertise<autoware_msgs::DetectedObjectArray>("/prediction/objects", 10);

  // Open the file
  std::ifstream file("/home/jobalchi/Ros1/pred_ws/src/cbmp/map/lane_1.csv"); // Change the file name as needed
  std::string line;

  // Skip the first three lines (metadata)
  for (int i = 0; i < 3; i++){
    std::getline(file, line);
  }

  // Read the data
  while (std::getline(file, line)){
    std::stringstream ss(line);
    std::string value;
    std::vector<double> row;

    // Split by semicolon
    while (std::getline(ss, value, ';')){
      row.push_back(std::stod(value)); // Convert string to double
    }

    // Assign data to corresponding columns
    cx_.push_back(row[1]);   // x_m
    cy_.push_back(row[2]);   // y_m
    cyaw_.push_back(row[3]); // psi_rad (yaw angle)
    ck_.push_back(row[4]);   // kappa_radpm (curvature)
  }

  // Output the result (for verification)
  // for (size_t i = 0; i < cx_.size(); ++i) {
  //   std::cout << "cx: " << cx_[i] << ", cy: " << cy_[i] << ", cyaw: " << cyaw_[i] << ", ck: " << ck_[i] << std::endl;
  // }

  file.close();
}

CBM_Pred::~CBM_Pred(){
  // None
}

void CBM_Pred::currentPoseCallback(const geometry_msgs::PoseStamped& input){
  cur_posx_ = input.pose.position.x;
  cur_posy_ = input.pose.position.y;
  cur_yaw_ = generateYawFromQuaternion(input.pose.orientation);
  cur_new_yaw_ = normalizeYaw(cur_yaw_);
}

void CBM_Pred::currentVelocityCallback(const geometry_msgs::TwistStamped& input){
  cur_velx_ = input.twist.linear.x;
  cur_vely_ = input.twist.linear.y;
}

void CBM_Pred::objectsCallback(const autoware_msgs::DetectedObjectArray& input){
  autoware_msgs::DetectedObjectArray output;
  output = input;

  for (const auto &object : input.objects){
    std::vector<autoware_msgs::DetectedObject> predicted_objects_vec;

    if (isObjectValid(object)){
      makePrediction(object, predicted_objects_vec);

      output.objects.insert(output.objects.end(), predicted_objects_vec.begin(), predicted_objects_vec.end());
    }
  }
  predicted_objects_pub_.publish(output);
}

bool CBM_Pred::isObjectValid(const autoware_msgs::DetectedObject &in_object){
  double distance = std::sqrt(std::pow(in_object.pose.position.x - cur_posx_, 2) + std::pow(in_object.pose.position.y - cur_posy_, 2));

  if (
    // !in_object.valid ||
    std::isnan(in_object.pose.orientation.x) ||
    std::isnan(in_object.pose.orientation.y) ||
    std::isnan(in_object.pose.orientation.z) ||
    std::isnan(in_object.pose.orientation.w) ||
    std::isnan(in_object.pose.position.x) ||
    std::isnan(in_object.pose.position.y) ||
    std::isnan(in_object.pose.position.z) ||
    std::isnan(in_object.velocity.linear.x) ||
    std::isnan(in_object.velocity.linear.y) ||
    std::isnan(in_object.velocity.linear.z) ||
    std::isnan(in_object.velocity.angular.z) ||
    (in_object.velocity.linear.x == 0 && in_object.velocity.linear.y == 0) ||
    (distance <= filter_out_close_object_threshold_ || distance >= filter_out_far_object_threshold_) ||
    (in_object.pose_reliable == false)
    // (in_object.dimensions.x <= 0) ||
    // (in_object.dimensions.y <= 0) ||
    // (in_object.dimensions.z <= 0)
    ){
    return false;
  }
  return true;
}

void CBM_Pred::makePrediction(const autoware_msgs::DetectedObject& object, std::vector<autoware_msgs::DetectedObject>& predicted_objects_vec){
  autoware_msgs::DetectedObject target_object = object;

  target_object.score = MAX_PREDICTION_SCORE_;
  
  for (int ith_prediction = 0; ith_prediction < num_prediction_; ith_prediction++){
    autoware_msgs::DetectedObject predicted_object = generatePredictedObject(target_object);

    predicted_object.score = (-1/(interval_sec_*num_prediction_))*ith_prediction*interval_sec_ + MAX_PREDICTION_SCORE_;
    predicted_objects_vec.push_back(predicted_object);
    
    target_object = predicted_object;
  }
}

autoware_msgs::DetectedObject CBM_Pred::generatePredictedObject(const autoware_msgs::DetectedObject& object){
  autoware_msgs::DetectedObject predicted_object;
  // CTRA Model
  // predicted_object = moveConstantTurnRateAcceleration(object);

  // Stanley + CTRA Model
  predicted_object = controlBasedMotionPrediction(object);

  return predicted_object;
}

std::unordered_map<int, geometry_msgs::Twist> previous_velocities_;

autoware_msgs::DetectedObject CBM_Pred::moveConstantTurnRateAcceleration(const autoware_msgs::DetectedObject& object){
  autoware_msgs::DetectedObject predicted_object;
  predicted_object = object;
  
  double px = object.pose.position.x;
  double py = object.pose.position.y;
  double yaw = generateYawFromQuaternion(object.pose.orientation);
  double new_yaw = normalizeYaw(yaw);
  double yawd = object.velocity.angular.z;
  double velocity = std::sqrt(std::pow(object.velocity.linear.x, 2) + std::pow(object.velocity.linear.y, 2));
  double prev_velocity = 0.0;

  if (previous_velocities_.find(object.id) != previous_velocities_.end()) {
    prev_velocity = std::sqrt(std::pow(previous_velocities_[object.id].linear.x, 2) + std::pow(previous_velocities_[object.id].linear.y, 2));
  }

  double acceleration = (velocity - prev_velocity) / interval_sec_;

  previous_velocities_[object.id] = object.velocity;
  
  // predicted state values
  double prediction_px, prediction_py, delta_x, delta_y, delta_velocity;
  
  delta_velocity = acceleration * interval_sec_;
  velocity += delta_velocity;
  
  // avoid division by zero
  if (fabs(yawd) > 0.001) {
      delta_x = velocity / yawd * (sin(new_yaw + yawd * interval_sec_) - sin(new_yaw));
      delta_y = velocity / yawd * (cos(new_yaw) - cos(new_yaw + yawd * interval_sec_));
      prediction_px = px + delta_x;
      prediction_py = py + delta_y;
  }
  else {
      delta_x = velocity * interval_sec_ * cos(new_yaw);
      delta_y = velocity * interval_sec_ * sin(new_yaw);
      prediction_px = px + delta_x;
      prediction_py = py + delta_y;
  }
  
  double prediction_yaw = new_yaw + yawd * interval_sec_;
    
  predicted_object.pose.position.x = prediction_px;
  predicted_object.pose.position.y = prediction_py;
  tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, prediction_yaw);
  predicted_object.pose.orientation.x = q[0];
  predicted_object.pose.orientation.y = q[1];
  predicted_object.pose.orientation.z = q[2];
  predicted_object.pose.orientation.w = q[3];

  return predicted_object;
}

autoware_msgs::DetectedObject CBM_Pred::controlBasedMotionPrediction(const autoware_msgs::DetectedObject& object){
  autoware_msgs::DetectedObject predicted_object;
  predicted_object = object;
  
  double px = object.pose.position.x;
  double py = object.pose.position.y;
  double yaw = generateYawFromQuaternion(object.pose.orientation);
  double new_yaw = normalizeYaw(yaw);
  double yawd = object.velocity.angular.z;
  double velocity = std::sqrt(std::pow(object.velocity.linear.x, 2) + std::pow(object.velocity.linear.y, 2));
  double prev_velocity = 0.0;

  if (previous_velocities_.find(object.id) != previous_velocities_.end()) {
    prev_velocity = std::sqrt(std::pow(previous_velocities_[object.id].linear.x, 2) + std::pow(previous_velocities_[object.id].linear.y, 2));
  }

  double acceleration = (velocity - prev_velocity) / interval_sec_;

  previous_velocities_[object.id] = object.velocity;

  Opp_State state;
  state.x = px;
  state.y = py;
  state.yaw = new_yaw;
  state.yaw_d = yawd;
  state.vel = velocity;
  state.acc = acceleration;

  auto [target_idx, error_front_axle] = calcTargetIndex(state, cx_, cy_);
  double target_vel = 100 / 3.6; // m/s
  // double target_vel = velocity; // m/s
  double ai = pidControl(target_vel, state.vel);
  auto [di, new_target_idx] = stanleyControl(state, cx_, cy_, cyaw_, target_idx);

  Opp_State ctrl_state;
  ctrl_state = stateUpdate(state, ai, di);
      
  predicted_object.pose.position.x = ctrl_state.x;
  predicted_object.pose.position.y = ctrl_state.y;
  tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, ctrl_state.yaw);
  predicted_object.pose.orientation.x = q[0];
  predicted_object.pose.orientation.y = q[1];
  predicted_object.pose.orientation.z = q[2];
  predicted_object.pose.orientation.w = q[3];

  return predicted_object;
}

Opp_State CBM_Pred::stateUpdate(const Opp_State& state, double acceleration, double delta){
  Opp_State updated_state = state;

  double updated_delta = rangeClip(delta, degreeToRadian(-max_steer_), degreeToRadian(max_steer_));
  double updated_acceleration = rangeClip(acceleration, min_accel_, max_accel_);

  // predicted state values
  double prediction_px, prediction_py, prediction_yaw, prediction_velocity, delta_x, delta_y, delta_velocity;

  delta_velocity = updated_acceleration * interval_sec_;
  prediction_velocity = delta_velocity + updated_state.vel;

  // avoid division by zero
  if (fabs(updated_state.yaw_d) > 0.001) {
      delta_x = prediction_velocity / updated_state.yaw_d * (sin(updated_state.yaw + updated_state.yaw_d * interval_sec_) - sin(updated_state.yaw));
      delta_y = prediction_velocity / updated_state.yaw_d * (cos(updated_state.yaw) - cos(updated_state.yaw + updated_state.yaw_d * interval_sec_));
      prediction_px = updated_state.x + delta_x;
      prediction_py = updated_state.y + delta_y;
  }
  else {
      delta_x = prediction_velocity * interval_sec_ * cos(updated_state.yaw);
      delta_y = prediction_velocity * interval_sec_ * sin(updated_state.yaw);
      prediction_px = updated_state.x + delta_x;
      prediction_py = updated_state.y + delta_y;
  }

  // prediction_yaw = updated_state.yaw + updated_state.yaw_d * interval_sec_;
  prediction_yaw = updated_state.yaw + updated_state.vel / L_ * tan(updated_delta) * interval_sec_;
  
  updated_state.x = prediction_px;
  updated_state.y = prediction_py;
  updated_state.yaw = prediction_yaw;
  updated_state.yaw_d = updated_state.yaw_d;
  updated_state.vel = prediction_velocity;
  updated_state.acc = updated_acceleration;

  return updated_state;
}

std::pair<int, double> CBM_Pred::calcTargetIndex(const Opp_State& state, const std::vector<double>& cx, const std::vector<double>& cy) {
  // Calc front axle position
  double fx = state.x + L_ * std::cos(state.yaw);
  double fy = state.y + L_ * std::sin(state.yaw);

  // Search nearest point index
  std::vector<double> dx(cx.size());
  std::vector<double> dy(cy.size());

  for (size_t i = 0; i < cx.size(); i++) {
      dx[i] = fx - cx[i];
      dy[i] = fy - cy[i];
  }

  std::vector<double> d(cx.size());
  for (size_t i = 0; i < cx.size(); i++) {
      d[i] = std::hypot(dx[i], dy[i]);
  }
  
  int target_idx = std::distance(d.begin(), std::min_element(d.begin(), d.end()));

  // Project RMS error onto front axle vector
  std::vector<double> front_axle_vec = {-std::cos(state.yaw + PI / 2), -std::sin(state.yaw + PI / 2)};
  double error_front_axle = dx[target_idx] * front_axle_vec[0] + dy[target_idx] * front_axle_vec[1];

  return std::make_pair(target_idx, error_front_axle);
}

double CBM_Pred::pidControl(double target, double current){
  double output = Kp_ * (target - current);
  return output;
}

std::pair<double, int> CBM_Pred::stanleyControl(const Opp_State& state, const std::vector<double>& cx, const std::vector<double>& cy, const std::vector<double>& cyaw, int last_target_idx){
  // Calculate current target index and front axle error
  auto [current_target_idx, error_front_axle] = calcTargetIndex(state, cx, cy);

  if (last_target_idx >= current_target_idx){
      current_target_idx = last_target_idx;
  }

  // theta_e: Correct the heading error
  double theta_e = (this->normalizeYaw(cyaw[current_target_idx]) + PI / 2.0) - state.yaw;

  // theta_d: Correct the cross-track error
  double theta_d = std::atan2(k_ * error_front_axle, state.vel);

  // Steering angle calculation
  double delta = theta_e + theta_d;

  return std::make_pair(delta, current_target_idx);
}

double CBM_Pred::generateYawFromQuaternion(const geometry_msgs::Quaternion& quaternion){
  tf::Quaternion q(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

double CBM_Pred::normalizeYaw(double yaw){
  double normalized_yaw = 0.0;

  if (yaw >= 0 && yaw <= PI) {
      normalized_yaw = yaw;
  }
  else if (yaw < 0 && yaw > -PI) {
      normalized_yaw = yaw + 2.0 * PI;
  }
  else {
      normalized_yaw = 0.0;
  }
  return normalized_yaw;
}

double CBM_Pred::rangeClip(double value, double lower, double upper){
  return std::min(upper, std::max(value, lower));
}

double CBM_Pred::degreeToRadian(double degree){
  return degree * (PI / 180.0);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "cbmp_node");
  CBM_Pred node;
  ros::spin();
  return 0;
}
