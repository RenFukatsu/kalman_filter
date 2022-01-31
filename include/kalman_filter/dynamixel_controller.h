#ifndef DYNAMIXEL_CONTROLLER_H_
#define DYNAMIXEL_CONTROLLER_H_

#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include "color_detector_msgs/TargetAngleList.h"
#include "color_detector_msgs/TargetPosition.h"
#include "color_detector_params/hsv.h"
#include "color_detector_srvs/ColorEnable.h"
#include "dynamixel_angle_msgs/DynamixelAngle.h"
#include "kalman_filter/TargetArray.h"
#include "kalman_filter/kalman_filter.h"

class DynamixelController {
 public:
    DynamixelController();
    void read_targets_info_parameter();
    void read_tracker_ids();
    void pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr&, size_t);
    void update_kalman_filter(size_t, const geometry_msgs::PoseWithCovarianceStampedConstPtr&);
    void calc_target_pose_on_world(std::string, const color_detector_msgs::TargetPositionConstPtr&,
                                   const geometry_msgs::TransformStamped&, geometry_msgs::PoseStamped*);
    void timer_callback(const ros::TimerEvent&);
    void controll_dynamixel();
    bool is_near_to_observe(int, int);
    double norm(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b);
    double norm(const geometry_msgs::Point& a, const geometry_msgs::Point& b);
    double norm(double x1, double y1, double x2, double y2);
    void direct_camera(int, int);
    double calc_direction_angle(const geometry_msgs::Pose&, const geometry_msgs::Pose&);
    void process();

 private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Time start_time_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    ros::Publisher target_pub_;
    std::vector<ros::Publisher> dynamixel_pubs_;
    std::vector<ros::Subscriber> pose_subs_;
    std::map<int, KalmanFilter> kalman_filters_;
    std::vector<std::pair<int, std::string>> targets_info_;
    std::vector<int> tracker_robot_ids_;
    int HZ;
    int MIN_CLUSTER;
    double MOTION_NOISE;
    double MEASUREMENT_NOISE;
    double LIFETIME_THRESHOLD;
    double OBSERVABLE_DISTANCE;
    bool USE_DYNAMIXEL;
    ros::Timer timer_;
    std::vector<geometry_msgs::PoseWithCovarianceStamped> poses_;
};

#endif  // DYNAMIXEL_CONTROLLER_H_
