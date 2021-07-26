#ifndef MULTI_ROBOTS_Ellipse_VISUALISER_H_
#define MULTI_ROBOTS_Ellipse_VISUALISER_H_

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include "color_detector_msgs/TargetPosition.h"
#include "color_detector_params/hsv.h"
#include "kalman-filter/kalman_filter.h"

class EllipseVisualizer {
 public:
    EllipseVisualizer();
    void measurement_callback(const color_detector_msgs::TargetPositionConstPtr &pos);
    void process();
    void timer_callback(const ros::TimerEvent &event);
    void set_color_map();
    void update_kalman_filter(size_t idx, const color_detector_msgs::TargetPositionConstPtr &pos);
    void calc_target_pose_on_world(std::string roomba, const color_detector_msgs::TargetPositionConstPtr &target,
                                   const geometry_msgs::TransformStamped &transform,
                                   geometry_msgs::PoseStamped *output_pose);

 private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Timer timer_;
    std::vector<ros::Subscriber> measurement_subs_;
    ros::Publisher ellipse_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::vector<std::string> colors_;
    std::vector<KalmanFilter> kalman_filters_;
    ros::Time start_time_;
    double HZ;
    double THRESHOLD;
    std::map<std::string, std_msgs::ColorRGBA> color_map_;
};

#endif  // MULTI_ROBOTS_Ellipse_VISUALISER_H_
