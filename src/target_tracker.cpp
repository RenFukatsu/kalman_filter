#include "kalman_filter/target_tracker.h"

TargetTracker::TargetTracker() : private_nh_("~"), tf_listener_(tf_buffer_), start_time_(ros::Time::now()) {
    private_nh_.param("HZ", HZ, 10);
    private_nh_.param("USE_DYNAMIXEL", USE_DYNAMIXEL, true);
    private_nh_.param("MIN_CLUSTER", MIN_CLUSTER, 100);
    private_nh_.param("MOTION_NOISE", MOTION_NOISE, 0.03);
    private_nh_.param("MEASUREMENT_NOISE", MEASUREMENT_NOISE, 0.1);
    private_nh_.param("LIFETIME_THRESHOLD", LIFETIME_THRESHOLD, 0.1);

    read_robots_parameter();
    dynamixel_pubs_.resize(robots_.size());
    position_subs_.resize(robots_.size());
    angle_subs_.resize(robots_.size());
    color_enable_clients_.resize(robots_.size());
    color_enables_.resize(robots_.size());
    for (size_t i = 0; i < robots_.size(); i++) {
        ROS_INFO_STREAM("roomba" << robots_[i].first << "'s color is " << robots_[i].second);
        std::string roomba = "roomba" + std::to_string(robots_[i].first);
        dynamixel_pubs_[i] = nh_.advertise<dynamixel_angle_msgs::DynamixelAngle>(roomba + "/dynamixel/angle", 1);
        position_subs_[i] = nh_.subscribe(roomba + "/target/position", 1, &TargetTracker::position_callback, this);
        angle_subs_[i] = nh_.subscribe(roomba + "/target/angle", 1, &TargetTracker::angle_callback, this);
        color_enable_clients_[i] = nh_.serviceClient<color_detector_srvs::ColorEnable>(roomba + "/color_enable");
        for (const auto &p : robots_) {
            color_enables_[i][p.second] = false;
        }
    }

    target_pub_ = nh_.advertise<kalman_filter::TargetArray>("target", 1);
    ellipse_pub_ = private_nh_.advertise<visualization_msgs::MarkerArray>("ellipses", 1);
    set_color_map();
}

void TargetTracker::read_robots_parameter() {
    XmlRpc::XmlRpcValue index_list, color_list;
    if (!private_nh_.getParam("ROOMBA_INDEXES", index_list)) {
        ROS_ERROR_STREAM("cannnot read indexes parameter");
        return;
    }
    ROS_ASSERT(index_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    if (!private_nh_.getParam("ROOMBA_COLORS", color_list)) {
        ROS_ERROR_STREAM("cannnot read colors parameter");
        return;
    }
    ROS_ASSERT(color_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    ROS_ASSERT(index_list.size() == color_list.size());

    for (size_t i = 0; i < index_list.size(); i++) {
        ROS_ASSERT(index_list[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
        ROS_ASSERT(color_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
        robots_.emplace_back(index_list[i], color_list[i]);
    }
}

void TargetTracker::update_kalman_filter(size_t idx, const color_detector_msgs::TargetPositionConstPtr &pos) {
    geometry_msgs::TransformStamped transform_stamped;
    std::string roomba = "roomba" + std::to_string(robots_[idx].first);
    try {
        transform_stamped = tf_buffer_.lookupTransform("map", roomba + "/camera_link", ros::Time(0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN_STREAM(ex.what());
        ros::Duration(1.0).sleep();
        return;
    }
    geometry_msgs::PoseStamped target_pose;
    calc_target_pose_on_world(roomba, pos, transform_stamped, &target_pose);
    std::string color = robots_[idx].second;
    if (kalman_filters_.count(color) == 0) {
        kalman_filters_[color].set_motion_noise(MOTION_NOISE);
        kalman_filters_[color].set_measurement_noise(MEASUREMENT_NOISE);
    }
    kalman_filters_[color].update(target_pose.pose.position.x, target_pose.pose.position.y,
                                  (ros::Time::now() - start_time_).toSec());
}

void TargetTracker::calc_target_pose_on_world(std::string roomba,
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

void TargetTracker::angle_callback(const color_detector_msgs::TargetAngleListConstPtr &angles) {
    if (angles->data.empty()) {
        ROS_WARN("angle list is empty.");
        return;
    }
    color_detector_msgs::TargetAngle angle;
    double min_likelihood = 1e5;
    for (const auto &agl : angles->data) {
        if (agl.cluster_num < MIN_CLUSTER) continue;
        kalman_filters_[agl.color].estimate_update((ros::Time::now() - start_time_).toSec());
        double likelihood = kalman_filters_[agl.color].get_likelihood();
        if (likelihood < min_likelihood) {
            angle = agl;
            min_likelihood = likelihood;
        }
    }
    if (min_likelihood == 1e5) {
        ROS_DEBUG_STREAM("cannnot find roomba");
        return;
    }
    if (!isfinite(angle.radian)) {
        ROS_WARN_STREAM(angle.color << "'s radian is " << angle.radian);
        return;
    }
    if (!USE_DYNAMIXEL) return;
    dynamixel_angle_msgs::DynamixelAngle msg;
    msg.theta = angle.radian;
    int roomba_idx = -1;
    for (size_t i = 0; i < robots_.size(); i++) {
        if (robots_[i].first == angles->my_number) {
            roomba_idx = i;
            break;
        }
    }
    ROS_ASSERT(roomba_idx != -1);
    dynamixel_pubs_[roomba_idx].publish(msg);
    ROS_DEBUG_STREAM("camera direction to " << angle.color);
    call_color_enable_service(&color_enable_clients_[roomba_idx], &color_enables_[roomba_idx], angle.color);
}

void TargetTracker::call_color_enable_service(ros::ServiceClient *client, std::map<std::string, bool> *color_enable,
                                              std::string color) {
    for (auto itr = color_enable->begin(); itr != color_enable->end(); itr++) {
        if (itr->first == color && itr->second == false) {
            color_detector_srvs::ColorEnable srv;
            srv.request.color = itr->first;
            srv.request.color = true;
            if (client->call(srv)) {
                itr->second = true;
            } else {
                ROS_ERROR_STREAM("Failed to call service color_enable. Couldn't activate " << itr->first << ".");
            }
        }
        if (itr->first != color && itr->second == true) {
            color_detector_srvs::ColorEnable srv;
            srv.request.color = itr->first;
            srv.request.color = false;
            if (client->call(srv)) {
                itr->second = false;
            } else {
                ROS_ERROR_STREAM("Failed to call service color_enable. Couldn't deactivate " << itr->first << ".");
            }
        }
    }
}

void TargetTracker::position_callback(const color_detector_msgs::TargetPositionConstPtr &position) {
    for (size_t i = 0; i < robots_.size(); i++) {
        if (robots_[i].second == position->color) {
            update_kalman_filter(i, position);
        }
    }
}

void TargetTracker::set_color_map() {
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

void TargetTracker::visualize_ellipse() {
    visualization_msgs::MarkerArray markers;
    for (size_t i = 0; i < robots_.size(); i++) {
        if (kalman_filters_.count(robots_[i].second) == 0) continue;
        std::string roomba = "roomba" + std::to_string(robots_[i].first);
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = roomba + "/kf";
        marker.id = i;

        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.lifetime = ros::Duration();
        if (kalman_filters_[robots_[i].second].get_likelihood() < LIFETIME_THRESHOLD) {
            marker.action = visualization_msgs::Marker::DELETE;
            markers.markers.push_back(marker);
            continue;
        }
        marker.action = visualization_msgs::Marker::ADD;

        std::vector<double> ellipse = kalman_filters_[robots_[i].second].get_ellipse();
        marker.scale.x = ellipse[0];
        marker.scale.y = ellipse[1];
        marker.scale.z = 0.2;
        marker.pose.position.x = kalman_filters_[robots_[i].second].get_x();
        marker.pose.position.y = kalman_filters_[robots_[i].second].get_y();
        marker.pose.position.z = 0.2;
        double theta = std::acos(ellipse[1] / ellipse[0]);
        marker.pose.orientation.w = std::cos(theta / 2);
        marker.pose.orientation.x = std::cos(ellipse[2]) * std::sin(theta / 2);
        marker.pose.orientation.y = std::sin(ellipse[2]) * std::sin(theta / 2);
        marker.pose.orientation.z = 0.0;
        marker.color = color_map_[robots_[i].second];

        markers.markers.push_back(marker);
    }
    ellipse_pub_.publish(markers);
}

void TargetTracker::timer_callback(const ros::TimerEvent &event) {
    kalman_filter::TargetArray msg;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    for (size_t i = 0; i < robots_.size(); i++) {
        if (kalman_filters_.count(robots_[i].second) == 0) continue;
        ros::Time now = ros::Time::now();
        auto &kf = kalman_filters_[robots_[i].second];
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

    visualize_ellipse();
}

void TargetTracker::process() {
    timer_ = nh_.createTimer(ros::Duration(1.0 / HZ), &TargetTracker::timer_callback, this);
    ros::spin();
}
