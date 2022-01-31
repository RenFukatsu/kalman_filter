#include "kalman_filter/dynamixel_controller.h"

DynamixelController::DynamixelController() : private_nh_("~"), tf_listener_(tf_buffer_), start_time_(ros::Time::now()) {
    private_nh_.param("HZ", HZ, 1);
    private_nh_.param("USE_DYNAMIXEL", USE_DYNAMIXEL, true);
    private_nh_.param("MIN_CLUSTER", MIN_CLUSTER, 100);
    private_nh_.param("MOTION_NOISE", MOTION_NOISE, 0.03);
    private_nh_.param("MEASUREMENT_NOISE", MEASUREMENT_NOISE, 0.1);
    private_nh_.param("LIFETIME_THRESHOLD", LIFETIME_THRESHOLD, 0.1);
    private_nh_.param("OBSERVABLE_DISTANCE", OBSERVABLE_DISTANCE, 8.0);

    read_targets_info_parameter();
    read_tracker_ids();
    std::vector<std::string> colors;
    color_detector_params_hsv::init(colors);
    poses_.resize(colors.size());
    pose_subs_.resize(colors.size());
    dynamixel_pubs_.resize(tracker_robot_ids_.size());
    for (size_t i = 0; i < colors.size(); i++) {
        std::string topic = "/roomba" + std::to_string(i) + "/amcl_pose";
        pose_subs_[i] = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
            topic.c_str(), 1, boost::bind(&DynamixelController::pose_callback, this, _1, i));
    }
    for (size_t i = 0; i < tracker_robot_ids_.size(); i++) {
        std::string roomba = "roomba" + std::to_string(tracker_robot_ids_[i]);
        dynamixel_pubs_[i] = nh_.advertise<dynamixel_angle_msgs::DynamixelAngle>(roomba + "/dynamixel/angle", 1);
    }

    target_pub_ = nh_.advertise<kalman_filter::TargetArray>("target", 1);
}

void DynamixelController::read_targets_info_parameter() {
    XmlRpc::XmlRpcValue index_list, color_list;
    ROS_ASSERT(private_nh_.getParam("TARGET_INDEXES", index_list));
    ROS_ASSERT(index_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    ROS_ASSERT(private_nh_.getParam("TARGET_COLORS", color_list));
    ROS_ASSERT(color_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    ROS_ASSERT(index_list.size() == color_list.size());

    for (size_t i = 0; i < index_list.size(); i++) {
        ROS_ASSERT(index_list[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
        ROS_ASSERT(color_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
        targets_info_.emplace_back(index_list[i], color_list[i]);
    }
}

void DynamixelController::read_tracker_ids() {
    XmlRpc::XmlRpcValue id_list;
    ROS_ASSERT(private_nh_.getParam("TRACKER_ROOMBA_IDS", id_list));
    ROS_ASSERT(id_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (size_t i = 0; i < id_list.size(); i++) {
        ROS_ASSERT(id_list[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
        tracker_robot_ids_.push_back(id_list[i]);
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

    controll_dynamixel();

    // visualize_ellipse();
}

void DynamixelController::controll_dynamixel() {
    static std::map<int, int> direct_count;
    for (size_t i = 0; i < tracker_robot_ids_.size(); i++) {
        int tracker_id = tracker_robot_ids_[i];
        std::vector<int> observable_targets;
        for (size_t j = 0; j < targets_info_.size(); j++) {
            int target_id = targets_info_[j].first;
            if (is_near_to_observe(tracker_id, target_id)) {
                observable_targets.push_back(target_id);
            }
        }
        int min_direct_count = 1e8;
        int direct_target_id = -1;
        for (auto oti : observable_targets) {
            if (min_direct_count > direct_count[oti]) {
                min_direct_count = direct_count[oti];
                direct_target_id = oti;
            }
        }
        if (direct_target_id != -1) {
            direct_count[direct_target_id]++;
            direct_camera(tracker_id, direct_target_id);
        }
    }
}

void DynamixelController::direct_camera(int tracker_id, int target_id) {
    double radian = calc_direction_angle(poses_[tracker_id].pose.pose, poses_[target_id].pose.pose);
    for (size_t i = 0; i < tracker_robot_ids_.size(); i++) {
        if (tracker_robot_ids_[i] == tracker_id) {
            dynamixel_angle_msgs::DynamixelAngle msg;
            msg.theta = radian;
            dynamixel_pubs_[i].publish(msg);
        }
    }
}

double DynamixelController::calc_direction_angle(const geometry_msgs::Pose &target,
                                                 const geometry_msgs::Pose &tracker) {
    double tx = target.position.x;
    double ty = target.position.y;
    double sx = tracker.position.x;
    double sy = tracker.position.y;
    double theta = std::atan2(ty - sy, tx - sx);
    tf2::Quaternion quat;
    tf2::convert(tracker.orientation, quat);
    double r, p, y;
    tf2::Matrix3x3(quat).getRPY(r, p, y);
    double radian = theta - y;
    if (radian < 0) {
        radian += 2 * M_PI;
    }

    return radian;
}

bool DynamixelController::is_near_to_observe(int tracker_id, int target_id) {
    const auto &tracker_pose = poses_[tracker_id].pose.pose;
    const auto &target_pose = poses_[target_id].pose.pose;
    if (norm(tracker_pose, target_pose) <= OBSERVABLE_DISTANCE) return true;
    return false;
}

double DynamixelController::norm(const geometry_msgs::Pose &a, const geometry_msgs::Pose &b) {
    return norm(a.position, b.position);
}

double DynamixelController::norm(const geometry_msgs::Point &a, const geometry_msgs::Point &b) {
    return norm(a.x, a.y, b.x, b.y);
}

double DynamixelController::norm(double x1, double y1, double x2, double y2) {
    return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

void DynamixelController::process() {
    timer_ = nh_.createTimer(ros::Duration(1.0 / HZ), &DynamixelController::timer_callback, this);
    ros::spin();
}
