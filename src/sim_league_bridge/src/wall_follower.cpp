#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>

using std::placeholders::_1;

class WallFollow : public rclcpp::Node {
public:
    WallFollow() : Node("wall_follower"), 
    error_1(0.0), prev_error_1(0.0), error_integral_1(0.0), 
    error_2(0.0), prev_error_2(0.0), error_integral_2(0.0), 
    current_t(0.0), prev_t(0.0),
    kp(2.0), ki(0.0), kd(0.1) {
        twist_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/autodrive/f1tenth_1/lidar", 10, std::bind(&WallFollow::callback, this, std::placeholders::_1));
    }

private:
    double getrange(const sensor_msgs::msg::LaserScan::SharedPtr msg, double angle) {
        int index = (angle - msg->angle_min) / msg->angle_increment;
        return msg->ranges[index];
    }

    double calculate_error(const sensor_msgs::msg::LaserScan::SharedPtr msg, double desired, double angle1, double angle2) {
        double theta = 0.0;       // Example value
        double lookahead = 0.5;

        theta = angle1 - angle2; 

        assert(theta >= msg->angle_min && theta <= msg->angle_max);

        double a = getrange(msg, angle1);
        double b = getrange(msg, angle2);
        // RCLCPP_INFO(this->get_logger(), "a: %f, b: %f", a, b);

        double alpha = std::atan2(a * std::sin(theta), a * std::cos(theta) - b);
        double dist_to_wall = b * std::cos(alpha);
        double future_diviation = lookahead * std::sin(alpha);
        // RCLCPP_INFO(this->get_logger(), "dist: %f, %f", dist_to_wall, future_diviation);

        double error = desired - (dist_to_wall + future_diviation);
        // RCLCPP_INFO(this->get_logger(), "error: %f", error);

        return error;
    }

    double steering_pid() {

        double prev_error;
        double avg_error;
        double avg_error_integral;

        // if(std::abs(error_1) > 2.5 || std::abs(error_2) > 2.5) {
        //     if(std::abs(error_1) > 2.5) {
        //         prev_error = prev_error_2;
        //         avg_error = error_2;
        //         avg_error_integral = error_integral_2;
        //     }
        //     else {
        //         prev_error = prev_error_1;
        //         avg_error = error_1;
        //         avg_error_integral = error_integral_1;
        //     }
        // }
        // else {
        //     prev_error = (prev_error_1 + prev_error_2);
        //     avg_error = (error_1 + error_2) / 2.0;
        //     avg_error_integral = (error_integral_1 + error_integral_2) / 2.0;
        // }

        prev_error = (prev_error_1);
        avg_error = (error_1);
        avg_error_integral = (error_integral_1);

        return kp * avg_error + kd * (avg_error - prev_error) / (current_t - prev_t) + ki * avg_error_integral;

        // return kp * error_1 + kd * (error_1 - prev_error_1) / (current_t - prev_t) + ki * error_integral_1;
    }

    double to_radians(double theta) {
        return M_PI * theta / 180.0;
    }

    void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Calculate error
        
        // RCLCPP_INFO(this->get_logger(), "Get error from left wall");
        prev_error_1 = this->error_1;
        this->error_1 = calculate_error(msg, 0.6, to_radians(-50), to_radians(-85));
        error_integral_1 += error_1;

        // RCLCPP_INFO(this->get_logger(), "Get error from right wall");
        prev_error_2 = this->error_2;
        this->error_2 = calculate_error(msg, 0.6, to_radians(85), to_radians(50));
        error_integral_2 += error_2;

        // Calculate time
        prev_t = current_t;
        current_t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        // Calculate PID
        double steering = steering_pid();
        double desired_vel = 0;
        RCLCPP_INFO(this->get_logger(), "Steering: %f", steering);

        // We go slower if we need to a large steering angle correction
        if (std::abs(steering) >= 0 && std::abs(steering) < this->to_radians(5)) {
            desired_vel = 5.0;
        }
        else if (std::abs(steering) >= this->to_radians(5) && std::abs(steering) < this->to_radians(10)) {
            desired_vel = 2.75;
        }
        else if (std::abs(steering) >= this->to_radians(10) && std::abs(steering) < this->to_radians(30)) {
            desired_vel = 1.5;
        } else {
            desired_vel = 0.75;
        }

        double error = desired_vel - current_velocity;
        current_velocity += 0.05 * error;

        // Publish
        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = current_velocity;
        twist.angular.z = steering;
        twist_pub->publish(twist);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;

    double error_1, prev_error_1, error_integral_1;
    double error_2, prev_error_2, error_integral_2;
    double kp, ki, kd;
    double prev_t, current_t;
    double current_velocity = 0.0;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting wall follower node...");
    auto node = std::make_shared<WallFollow>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}