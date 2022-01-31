#include "kalman_filter/dynamixel_controller.h"

DynamixelController::DynamixelController() : private_nh_("~"), tf_listener_(tf_buffer_), start_time_(ros::Time::now()) {
    private_nh_.param("HZ", HZ, 1);
    private_nh_.param("USE_DYNAMIXEL", USE_DYNAMIXEL, true);
    private_nh_.param("MIN_CLUSTER", MIN_CLUSTER, 100);
    private_nh_.param("MOTION_NOISE", MOTION_NOISE, 0.03);
    private_nh_.param("MEASUREMENT_NOISE", MEASUREMENT_NOISE, 0.1);
    private_nh_.param("LIFETIME_THRESHOLD", LIFETIME_THRESHOLD, 0.1);

    read_targets_info_parameter();
    std::vector<std::string> colors;
    color_detector_params_hsv::init(colors);
    poses_.resize(colors.size());
    pose_subs_.resize(colors.size());
    dynamixel_pubs_.resize(targets_info_.size());
    for (size_t i = 0; i < colors.size(); i++) {
        std::string topic = "/roomba" + std::to_string(i) + "/amcl_pose";
        pose_subs_[i] = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
            topic.c_str(), 1, boost::bind(&DynamixelController::pose_callback, this, _1, i));
    }
    for (size_t i = 0; i < targets_info_.size(); i++) {
        ROS_INFO_STREAM("target roomba" << targets_info_[i].first << "'s color is " << targets_info_[i].second);
        std::string roomba = "roomba" + std::to_string(targets_info_[i].first);
        dynamixel_pubs_[i] = nh_.advertise<dynamixel_angle_msgs::DynamixelAngle>(roomba + "/dynamixel/angle", 1);
    }

    target_pub_ = nh_.advertise<kalman_filter::TargetArray>("target", 1);
    ellipse_pub_ = private_nh_.advertise<visualization_msgs::MarkerArray>("ellipses", 1);
    set_color_map();
}

void DynamixelController::read_targets_info_parameter() {
    XmlRpc::XmlRpcValue index_list, color_list;
    ROS_ASSERT(private_nh_.getParam("ROOMBA_INDEXES", index_list));
    ROS_ASSERT(index_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    ROS_ASSERT(private_nh_.getParam("ROOMBA_COLORS", color_list));
    ROS_ASSERT(color_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    ROS_ASSERT(index_list.size() == color_list.size());

    for (size_t i = 0; i < index_list.size(); i++) {
        ROS_ASSERT(index_list[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
        ROS_ASSERT(color_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
        targets_info_.emplace_back(index_list[i], color_list[i]);
    }
}

void DynamixelController::update_kalman_filter(size_t id,
                                               const geometry_msgs::PoseWithCovarianceStampedConstPtr &target_pose) {
    if (kalman_filters_.count(id) == 0) {
        kalman_filters_[id].set_motion_noise(MOTION_NOISE);
        kalman_filters_[id].set_measurement_noise(MEASUREMENT_NOISE);
    }
    kalman_filters_[id].update(target_pose->pose.pose.position.x, target_pose->pose.pose.position.y,
                               (ros::Time::now() - start_time_).toSec());
}

void DynamixelController::pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose_ptr, size_t id) {
    poses_[id] = *pose_ptr;
    for (const auto &target : targets_info_) {
        if (target.first == id) {
            update_kalman_filter(id, pose_ptr);
        }
    }
}

void DynamixelController::calc_target_pose_on_world(std::string roomba,
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

void DynamixelController::set_color_map() {
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

void DynamixelController::visualize_ellipse() {
    visualization_msgs::MarkerArray markers;
    for (size_t i = 0; i < targets_info_.size(); i++) {
        if (kalman_filters_.count(targets_info_[i].first) == 0) continue;
        std::string roomba = "roomba" + std::to_string(targets_info_[i].first);
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = roomba + "/kf";
        marker.id = i;

        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.lifetime = ros::Duration();
        if (kalman_filters_[targets_info_[i].first].get_likelihood() < LIFETIME_THRESHOLD) {
            marker.action = visualization_msgs::Marker::DELETE;
            markers.markers.push_back(marker);
            continue;
        }
        marker.action = visualization_msgs::Marker::ADD;

        std::vector<double> ellipse = kalman_filters_[targets_info_[i].first].get_ellipse();
        marker.scale.x = ellipse[0];
        marker.scale.y = ellipse[1];
        marker.scale.z = 0.2;
        marker.pose.position.x = kalman_filters_[targets_info_[i].first].get_x();
        marker.pose.position.y = kalman_filters_[targets_info_[i].first].get_y();
        marker.pose.position.z = 0.2;
        double theta = std::acos(ellipse[1] / ellipse[0]);
        marker.pose.orientation.w = std::cos(theta / 2);
        marker.pose.orientation.x = std::cos(ellipse[2]) * std::sin(theta / 2);
        marker.pose.orientation.y = std::sin(ellipse[2]) * std::sin(theta / 2);
        marker.pose.orientation.z = 0.0;
        marker.color = color_map_[targets_info_[i].second];

        markers.markers.push_back(marker);
    }
    ellipse_pub_.publish(markers);
}

void DynamixelController::timer_callback(const ros::TimerEvent &event) {
    kalman_filter::TargetArray msg;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    for (size_t i = 0; i < targets_info_.size(); i++) {
        if (kalman_filters_.count(targets_info_[i].first) == 0) continue;
        ros::Time now = ros::Time::now();
        auto &kf = kalman_filters_[targets_info_[i].first];
        kf.estimate_update((now - start_time_).toSec());
        kalman_filter::Target target;
        target.header.frame_id = "map";
        target.header.stamp = now;
        target.id = i;
        target.position.x = kf.get_x();
        target.position.y = kf.get_y();
        target.twist.linear.x = kf.get_vx();
        target.twist.linear.y = kf.get_vy();
        target.covariance = kf.get_p();
        msg.targets.push_back(target);
    }
    target_pub_.publish(msg);

    // visualize_ellipse();
}

void DynamixelController::process() {
    timer_ = nh_.createTimer(ros::Duration(1.0 / HZ), &DynamixelController::timer_callback, this);
    ros::spin();
}
