#ifndef TARGET_TRACKER_H_
#define TARGET_TRACKER_H_

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

class TargetTracker {
 public:
    TargetTracker();
    void read_robots_parameter();
    void position_callback(const color_detector_msgs::TargetPositionConstPtr&);
    void angle_callback(const color_detector_msgs::TargetAngleListConstPtr&);
    void update_kalman_filter(size_t, const color_detector_msgs::TargetPositionConstPtr&);
    void calc_target_pose_on_world(std::string, const color_detector_msgs::TargetPositionConstPtr&,
                                   const geometry_msgs::TransformStamped&, geometry_msgs::PoseStamped*);
    void call_color_enable_service(ros::ServiceClient*, std::map<std::string, bool>*, std::string);
    void set_color_map();
    void visualize_ellipse();
    void timer_callback(const ros::TimerEvent&);
    void process();

 private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Time start_time_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    ros::Publisher target_pub_;
    std::vector<ros::Publisher> dynamixel_pubs_;
    std::vector<ros::Subscriber> position_subs_;
    std::vector<ros::Subscriber> angle_subs_;
    std::vector<ros::ServiceClient> color_enable_clients_;
    std::map<std::string, KalmanFilter> kalman_filters_;
    std::vector<std::pair<int, std::string>> robots_;
    std::vector<std::map<std::string, bool>> color_enables_;
    int HZ;
    int MIN_CLUSTER;
    double MOTION_NOISE;
    double MEASUREMENT_NOISE;
    double LIFETIME_THRESHOLD;
    bool USE_DYNAMIXEL;
    ros::Timer timer_;
    ros::Publisher ellipse_pub_;
    std::map<std::string, std_msgs::ColorRGBA> color_map_;
};

#endif  // TARGET_TRACKER_H_
