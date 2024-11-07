#ifndef CBM_PRED
#define CBM_PRED

#include <ros/ros.h>

#include <tf/transform_datatypes.h>

#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"

// Stanley - CTRA prediction struct
struct Opp_State {
  double x;        // x-coordinate
  double y;        // y-coordinate
  double yaw;      // yaw angle
  double yaw_d;    // yaw angle
  double vel;      // speed
  double acc;      // accel
};

class CBM_Pred {
private:
  // nodehandle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // ros Subscriber
  ros::Subscriber detected_objects_sub_;
  ros::Subscriber current_pose_sub_;
  ros::Subscriber current_vel_sub_;

  // ros publisher
  ros::Publisher predicted_objects_pub_;

  // current position & velocity
  double cur_posx_;
  double cur_posy_;
  double cur_yaw_;
  double cur_new_yaw_;
  double cur_velx_;
  double cur_vely_;

  // max prediction score
  const double MAX_PREDICTION_SCORE_;

  // prediction param
  double interval_sec_;
  int num_prediction_;
  double filter_out_close_object_threshold_;
  double filter_out_far_object_threshold_;

  // stanley param
  double k_ = 0.5;             // control gain
  double Kp_ = 1.0;            // speed proportional gain
  // double dt_ = 0.1;            // [s] time difference
  double L_ = 2.720;           // [m] Wheel base of vehicle
  double max_steer_ = 120.0;    // [degree] max steering angle
  double max_accel_ = 3.0;    // [m/s^2] max accel
  double min_accel_ = -2.0;    // [m/s^2] min accel

  // Map lane variables to store each vector (of type double)
  std::vector<double> cx_, cy_, cyaw_, ck_;
  std::vector<double> cx_1, cy_1, cyaw_1, ck_1;
  std::vector<double> cx_2, cy_2, cyaw_2, ck_2;
  std::vector<double> cx_3, cy_3, cyaw_3, ck_3;

  // callback function
  void currentPoseCallback(const geometry_msgs::PoseStamped& input);

  void currentVelocityCallback(const geometry_msgs::TwistStamped& input);

  void objectsCallback(const autoware_msgs::DetectedObjectArray& input);
  
  // Stanley & CTRA prediction function
  bool isObjectValid(const autoware_msgs::DetectedObject &in_object);

  void makePrediction(const autoware_msgs::DetectedObject& object, std::vector<autoware_msgs::DetectedObject>& predicted_objects);

  autoware_msgs::DetectedObject generatePredictedObject(const autoware_msgs::DetectedObject& object);

  autoware_msgs::DetectedObject moveConstantTurnRateAcceleration(const autoware_msgs::DetectedObject& object);

  autoware_msgs::DetectedObject controlBasedMotionPrediction(const autoware_msgs::DetectedObject& object);

  Opp_State stateUpdate(const Opp_State& state, double acceleration, double delta); // acceleration, steering
  
  std::pair<int, double> calcTargetIndex(const Opp_State& state, const std::vector<double>& cx_, const std::vector<double>& cy_);
 
  double pidControl(double target, double current);

  std::pair<double, int> stanleyControl(const Opp_State& state, const std::vector<double>& cx_, const std::vector<double>& cy_, const std::vector<double>& cyaw_, int last_target_idx);

  // general function
  void loadLaneData(const std::string& filename, std::vector<double>& cx, std::vector<double>& cy, std::vector<double>& cyaw, std::vector<double>& ck);
  
  int selectClosestLane(double x, double y);

  double calcDistanceToLane(double x, double y, const std::vector<double>& cx, const std::vector<double>& cy);
  
  double generateYawFromQuaternion(const geometry_msgs::Quaternion& quaternion);

  double normalizeYaw(double yaw);

  double rangeClip(double value, double lower, double upper);

  double degreeToRadian(double degree);

public:
  CBM_Pred();
  ~CBM_Pred();
};

#endif  // CBM_PRED
