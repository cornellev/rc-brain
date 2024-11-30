#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include "cev_msgs/msg/sensor_collect.hpp"
#include <cmath>

using std::placeholders::_1;

/** NOTES
 * R = L / tan(steering_angle)
 * Circle Center = (-R, 0)
 *
 * Obstacle = (rsin(theta), rcos(theta))
 *
 * Angle from center to obstacle = tan(O_y / (O_x + R))
 * Dist from center to obstacle = sqrt((O_x + R)^2 + O_y^2)
 * 
 * Circum dist to obstacle = R * angle from center to obstacle
 * Time to hit obstacle = circum dist to obstacle / velocity
 *
 * NOTE: LIDAR'S 0 is forward, and angles increment clockwise
 */


class AutobrakeNode : public rclcpp::Node
{
public:
    AutobrakeNode() : Node("autobrake")
    {
        brake_.data = MAX_VEL;

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 1, std::bind(&AutobrakeNode::checkCollision, this, _1));

        sensor_collect_sub_ = this->create_subscription<cev_msgs::msg::SensorCollect>(
            "sensor_collect", 1, std::bind(&AutobrakeNode::setVars, this, _1));

        rc_movement_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
            "rc_movement_msg", 1, std::bind(&AutobrakeNode::setTargets, this, _1));

        brake_pub_ = this->create_publisher<std_msgs::msg::Float32>("auto_max_vel", 1);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&AutobrakeNode::publishBrake, this)
        );

        RCLCPP_INFO(this->get_logger(), "Autobrake node initialized.");
    }

private:  // TODO: Make these constants configurable and use transforms instead of direct lidar
    const float VEHICLE_LENGTH = 0.185;
    const float VEHICLE_WIDTH = 0.18;
    const float AUTOBRAKE_TIME = 0.7;
    const float AUTOBRAKE_DISTANCE = 0.2;
    const float MAX_VEL = 1.5;
    const int MIN_COLLISIONS_FOR_BRAKE = 3;
    const float LIDAR_ROTATIONAL_OFFSET = M_PI;
    const float LIDAR_HORIZONTAL_OFFSET = 0.035;

    float steering_angle_ = 0;
    float velocity_ = 0;
    float target_velocity_ = 0;
    std_msgs::msg::Float32 brake_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<cev_msgs::msg::SensorCollect>::SharedPtr sensor_collect_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr rc_movement_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr brake_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Calculate the maximum velocity based on the distance to an obstacle
    float maxVelocity(float dist)
    {
        if (dist < 0)
            return MAX_VEL;
        if (dist < AUTOBRAKE_DISTANCE)
            return 0;

        return std::max(0.0f, -3.5f + std::sqrt(49 + 40 * dist) / 2 - 0.2f);
    }

    // LIDAR'S 0 is forward, and angles increment clockwise
    // Check for potential collisions based on laser scan data
    void checkCollision(const sensor_msgs::msg::LaserScan::SharedPtr data)
    {
        int invert_flag = 1;
        float min_vel = MAX_VEL;

        if (steering_angle_ < 0)
            invert_flag = -1;

        // Calculate turning radius (L / tan(steering_angle)), or infinity if driving straight
        float turning_radius = std::abs(steering_angle_) > 0.01 ? VEHICLE_LENGTH / std::tan(invert_flag * steering_angle_) : std::numeric_limits<float>::infinity();
        
        // Calculate turning radii of inside and outside wheels
        float inner_wheel_radius = turning_radius - VEHICLE_WIDTH / 2;
        float outer_wheel_radius = turning_radius + VEHICLE_WIDTH / 2;

        for (size_t i = 0; i < data->ranges.size(); ++i)
        {
            if (data->ranges[i] < data->range_min || data->ranges[i] > data->range_max)
                continue;

            float dist_to_obstacle = std::numeric_limits<float>::infinity();

            // X and Y coordinates of obstacle relative to vehicle
            float theta = LIDAR_ROTATIONAL_OFFSET + data->angle_min + i * data->angle_increment;
            float r = data->ranges[i];
            float x = invert_flag * (r * std::sin(theta) + LIDAR_HORIZONTAL_OFFSET);
            float y = r * std::cos(theta);

            if (turning_radius == std::numeric_limits<float>::infinity())
            {
                // If driving straight, check if obstacle is within the width of the vehicle
                if (std::abs(x) < VEHICLE_WIDTH / 2)
                    dist_to_obstacle = y;
                else
                    continue;
            }
            else
            {
                // For curving, calculate radius distance to obstacle from center of movement circle
                float radius_dist_to_obstacle = std::sqrt((x + turning_radius) * (x + turning_radius) + y * y);

                if (outer_wheel_radius > radius_dist_to_obstacle && radius_dist_to_obstacle > inner_wheel_radius)
                {
                    // Calculate angle from center of movement circle to obstacle
                    float angle_from_center_to_obstacle = std::atan2(y, turning_radius + x);
                    if (angle_from_center_to_obstacle < 0)
                        angle_from_center_to_obstacle += 2 * M_PI;
                    angle_from_center_to_obstacle = fmod(angle_from_center_to_obstacle, 2 * M_PI);
                    
                    // Calculate circumferential distance to obstacle
                    dist_to_obstacle = turning_radius * angle_from_center_to_obstacle;
                }
                else
                    continue;
            }

            // If the obstacle is within a valid distance, adjust the minimum velocity
            if (dist_to_obstacle >= 0)
                min_vel = std::min(min_vel, maxVelocity(dist_to_obstacle));
        }

        // Set the brake value to the minimum calculated velocity
        brake_.data = min_vel;
    }

    // Callback to set the velocity and steering angle from the sensor_collect topic
    void setVars(const cev_msgs::msg::SensorCollect::SharedPtr data)
    {
        velocity_ = data->velocity;
        steering_angle_ = -data->steering_angle;  // Invert the steering angle
    }

    // Callback to set the target velocity from the AckermannDrive topic
    void setTargets(const ackermann_msgs::msg::AckermannDrive::SharedPtr data)
    {
        target_velocity_ = data->speed;
    }

    // Publish the brake value at a regular interval
    void publishBrake()
    {
        brake_pub_->publish(brake_);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutobrakeNode>());
    rclcpp::shutdown();
    return 0;
}
