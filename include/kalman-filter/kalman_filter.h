#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include <Eigen/Dense>
#include <iostream>
#include <vector>

// ref: https://ja.wikipedia.org/wiki/カルマンフィルター
class KalmanFilter {
 private:
    double last_measured_time_;
    Eigen::Vector4d x_;              // estimate states
    Eigen::Vector4d u_;              // motion vector
    Eigen::Matrix4d p_;              // uncertainty covariance
    Eigen::Matrix4d f_;              // state transition matrix
    Eigen::Matrix<double, 2, 4> h_;  // mesurement function
    Eigen::Matrix2d r_;              // mesurement noise
    Eigen::Matrix4d i_;              // identity matrix
    double sigma_a_;                 // motion noise
    double likelihood_;
    double ellipse_major_axis_;
    double ellipse_short_axis_;
    double ellipse_theta_;

    void measurement_update(double measured_x, double measured_y) {
        Eigen::RowVector2d z(measured_x, measured_y);
        Eigen::Vector2d y = z.transpose() - (h_ * x_);
        Eigen::Matrix2d s = h_ * p_ * h_.transpose() + r_;
        Eigen::Matrix<double, 4, 2> k = p_ * h_.transpose() * s.inverse();
        x_ = x_ + (k * y);
        p_ = (i_ - (k * h_)) * p_;
    }

    Eigen::Matrix4d state_transition_noise(double dt) {
        Eigen::Matrix<double, 4, 2> g;
        g << dt * dt / 2.0, 0.0, 0.0, dt * dt / 2.0, dt, 0.0, 0.0, dt;
        return sigma_a_ * sigma_a_ * g * g.transpose();
    }

    void motion_update(double measured_time) {
        double dt = measured_time - last_measured_time_;
        last_measured_time_ = measured_time;
        f_(0, 2) = dt;
        f_(1, 3) = dt;

        x_ = (f_ * x_) + u_;
        p_ = f_ * p_ * f_.transpose() + state_transition_noise(dt);
    }

    // ref: https://myenigma.hatenablog.com/category/robot?page=1403618922
    void calculate_likelihood() {
        static const double CHI2 = 9.21034;  // chi-square, 99%
        Eigen::Matrix2d m = p_.block<2, 2>(0, 0);
        Eigen::EigenSolver<Eigen::Matrix2d> solver(m);
        if (solver.info() != Eigen::Success) {
            std::cerr << "Eigen solver error : " << solver.info() << std::endl;
            return;
        }
        Eigen::Vector2d e_values = solver.eigenvalues().real();
        Eigen::Matrix2cd e_vectors = solver.eigenvectors();
        if (e_values(0) > e_values(1)) {
            ellipse_major_axis_ = std::sqrt(CHI2 * e_values(0));
            ellipse_short_axis_ = std::sqrt(CHI2 * e_values(1));
            ellipse_theta_ = std::atan2(e_vectors.col(0).y().real(), e_vectors.col(0).x().real());
        } else {
            ellipse_major_axis_ = std::sqrt(CHI2 * e_values(1));
            ellipse_short_axis_ = std::sqrt(CHI2 * e_values(0));
            ellipse_theta_ = std::atan2(e_vectors.col(1).y().real(), e_vectors.col(1).x().real());
        }

        if (ellipse_major_axis_ * ellipse_short_axis_ > 1e-5) {
            likelihood_ = 1. / (ellipse_major_axis_ * ellipse_short_axis_);
        } else {
            likelihood_ = 1e5;
        }
    }

 public:
    KalmanFilter() : last_measured_time_(0.0), sigma_a_(0.1) {
        x_.setZero();
        u_.setZero();
        p_ << 1000, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 1000;
        f_.setIdentity();
        h_.setIdentity();
        r_ << 0.1, 0, 0, 0.1;
        i_.setIdentity();
    }

    explicit KalmanFilter(double init_x, double init_y, double measured_time, double uncertainty_x = 0.1,
                          double uncertainty_y = 0.1)
        : KalmanFilter() {
        last_measured_time_ = measured_time;
        x_(0) = init_x;
        x_(1) = init_y;
        p_(0, 0) = uncertainty_x;
        p_(1, 1) = uncertainty_y;
    }

    void set_measurement_noise(double sigma) { r_ << sigma, 0, 0, sigma; }

    void set_motion_noise(double sigma) { sigma_a_ = sigma; }

    void update(double measured_x, double measured_y, double measured_time) {
        motion_update(measured_time);
        measurement_update(measured_x, measured_y);
        calculate_likelihood();
    }

    void estimate_update(double measured_time) {
        motion_update(measured_time);
        calculate_likelihood();
    }

    double get_x() { return x_(0); }

    double get_y() { return x_(1); }

    double get_likelihood() { return likelihood_; }

    // return {major_length, short_length, theta}
    std::vector<double> get_ellipse() { return {ellipse_major_axis_, ellipse_short_axis_, ellipse_theta_}; }
};

#endif  // KALMAN_FILTER_H_
