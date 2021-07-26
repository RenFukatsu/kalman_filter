#include "kalman-filter/ellipse_visualizer.h"

EllipseVisualizer::EllipseVisualizer() : private_nh_("~"), tf_listener_(tf_buffer_), start_time_(ros::Time::now()) {
    ellipse_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("ellipses", 1);
    double MOTION_NOISE, MEASUREMENT_NOISE;
    private_nh_.param("HZ", HZ, 10.0);
    private_nh_.param("THRESHOLD", THRESHOLD, 0.1);
    private_nh_.param("MOTION_NOISE", MOTION_NOISE, 0.1);
    private_nh_.param("MEASUREMENT_NOISE", MEASUREMENT_NOISE, 0.5);

    std::vector<color_detector_params_hsv::ThresholdHSV> _;
    color_detector_params_hsv::init(colors_, _);
    kalman_filters_.resize(colors_.size());
    measurement_subs_.resize(colors_.size());
    for (size_t i = 0; i < colors_.size(); i++) {
        std::string topic = "roomba" + std::to_string(i + 1) + "/target/position";
        measurement_subs_[i] = nh_.subscribe(topic, 1, &EllipseVisualizer::measurement_callback, this);
        kalman_filters_[i].set_motion_noise(MOTION_NOISE);
        kalman_filters_[i].set_measurement_noise(MEASUREMENT_NOISE);
    }
    set_color_map();
}

void EllipseVisualizer::set_color_map() {
    color_map_["green"].r = 0.0f;
    color_map_["green"].g = 0.5f;
    color_map_["green"].b = 0.0f;
    color_map_["green"].a = 0.3f;
    color_map_["yellow"].r = 1.0f;
    color_map_["yellow"].g = 1.0f;
    color_map_["yellow"].b = 0.0f;
    color_map_["yellow"].a = 0.3f;
    color_map_["blue"].r = 0.0f;
    color_map_["blue"].g = 0.0f;
    color_map_["blue"].b = 1.0f;
    color_map_["blue"].a = 0.3f;
    color_map_["orange"].r = 1.0f;
    color_map_["orange"].g = 0.6f;
    color_map_["orange"].b = 0.0f;
    color_map_["orange"].a = 0.3f;
    color_map_["purple"].r = 0.5f;
    color_map_["purple"].g = 0.0f;
    color_map_["purple"].b = 0.5f;
    color_map_["purple"].a = 0.3f;
    color_map_["red"].r = 1.0f;
    color_map_["red"].g = 0.0f;
    color_map_["red"].b = 0.0f;
    color_map_["red"].a = 0.3f;
}

void EllipseVisualizer::measurement_callback(const color_detector_msgs::TargetPositionConstPtr &pos) {
    for (size_t i = 0; i < colors_.size(); i++) {
        if (colors_[i] == pos->color) {
            update_kalman_filter(i, pos);
        }
    }
}

void EllipseVisualizer::update_kalman_filter(size_t idx, const color_detector_msgs::TargetPositionConstPtr &pos) {
    geometry_msgs::TransformStamped transform_stamped;
    std::string roomba = "roomba" + std::to_string(idx + 1);
    try {
        transform_stamped = tf_buffer_.lookupTransform("map", roomba + "/camera_link", ros::Time(0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN_STREAM(ex.what());
        ros::Duration(1.0).sleep();
        return;
    }
    geometry_msgs::PoseStamped target_pose;
    calc_target_pose_on_world(roomba, pos, transform_stamped, &target_pose);
    kalman_filters_[idx].update(target_pose.pose.position.x, target_pose.pose.position.y,
                                (ros::Time::now() - start_time_).toSec());
}

void EllipseVisualizer::calc_target_pose_on_world(std::string roomba,
                                                  const color_detector_msgs::TargetPositionConstPtr &target,
                                                  const geometry_msgs::TransformStamped &transform,
                                                  geometry_msgs::PoseStamped *output_pose) {
    geometry_msgs::PoseStamped target_pose;
    target_pose.header = target->header;
    target_pose.header.frame_id = roomba + "/camera_link";
    target_pose.pose.position.x = target->z;
    target_pose.pose.position.y = -target->x;
    target_pose.pose.position.z = target->y;
    target_pose.pose.orientation.w = 1;
    target_pose.pose.orientation.x = 0;
    target_pose.pose.orientation.y = 0;
    target_pose.pose.orientation.z = 0;

    tf2::doTransform(target_pose, *output_pose, transform);
    return;
}

void EllipseVisualizer::timer_callback(const ros::TimerEvent &event) {
    visualization_msgs::MarkerArray markers;
    for (size_t i = 0; i < colors_.size(); i++) {
        ros::Time now = ros::Time::now();
        kalman_filters_[i].estimate_update((now - start_time_).toSec());
        std::string roomba = "roomba" + std::to_string(i + 1);
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = now;
        marker.ns = roomba + "/kf";
        marker.id = i;

        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.lifetime = ros::Duration();
        if (kalman_filters_[i].get_likelihood() < THRESHOLD) {
            marker.action = visualization_msgs::Marker::DELETE;
            markers.markers.push_back(marker);
            continue;
        }
        marker.action = visualization_msgs::Marker::ADD;

        std::vector<double> ellipse = kalman_filters_[i].get_ellipse();
        marker.scale.x = ellipse[0];
        marker.scale.y = ellipse[1];
        marker.scale.z = 0.2;
        marker.pose.position.x = kalman_filters_[i].get_x();
        marker.pose.position.y = kalman_filters_[i].get_y();
        marker.pose.position.z = 0.2;
        double theta = std::acos(ellipse[1] / ellipse[0]);
        marker.pose.orientation.w = std::cos(theta / 2);
        marker.pose.orientation.x = std::cos(ellipse[2]) * std::sin(theta / 2);
        marker.pose.orientation.y = std::sin(ellipse[2]) * std::sin(theta / 2);
        marker.pose.orientation.z = 0.0;
        marker.color = color_map_[colors_[i]];

        markers.markers.push_back(marker);
    }
    ellipse_pub_.publish(markers);
}

void EllipseVisualizer::process() {
    timer_ = nh_.createTimer(ros::Duration(1.0 / HZ), &EllipseVisualizer::timer_callback, this);
    ros::spin();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ellipse_visualizer");
    EllipseVisualizer ev;
    ev.process();
    return 0;
}
