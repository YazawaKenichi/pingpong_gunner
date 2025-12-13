#include "pingpong_gunner/core.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <pingpong_msgs/msg/shot_params.hpp>

namespace GunControllerNodes
{
GunControllerNode::GunControllerNode() : rclcpp_lifecycle::LifecycleNode("gun_controller_node"), duty_accel_(DUTY_ACCEL_DEFAULT), duty_deccel_(DUTY_DECCEL_DEFAULT)
{
    RCLCPP_INFO(get_logger(), "Gun Controller Node constructed.");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn GunControllerNode::on_configure(const rclcpp_lifecycle::State & /* state */)
{
    RCLCPP_INFO(get_logger(), "on configure");

    this->declare_parameter<float>("duty_accel", DUTY_ACCEL_DEFAULT);
    this->declare_parameter<float>("duty_deccel", DUTY_DECCEL_DEFAULT);
    this->duty_accel_ = this->get_parameter("duty_accel").as_double();
    this->duty_deccel_ = this->get_parameter("duty_deccel").as_double();
    RCLCPP_INFO(get_logger(), "Parameters: {duty_accel: %f, duty_deccel: %f}", this->duty_accel_, this->duty_deccel_);

    this->duty_publisher_left_ = this->create_publisher<std_msgs::msg::Float32>("/pico/gun/left/pwm/duty", 10);
    this->duty_publisher_right_ = this->create_publisher<std_msgs::msg::Float32>("/pico/gun/right/pwm/duty", 10);
    this->position_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/pico/stepper/position/raw", 10);
    this->pose_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("/pico/pose", 10);

    this->duty_subscriber_velocity_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/pico/gun/velocity", 10, std::bind(&GunControllerNode::velocity_callback, this, std::placeholders::_1));
    this->shot_params_subscriber_ = this->create_subscription<pingpong_msgs::msg::ShotParams>("/shot_command", 10, std::bind(&GunControllerNode::shot_params_callback, this, std::placeholders::_1));

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn GunControllerNode::on_activate(const rclcpp_lifecycle::State & /* state */)
{
    RCLCPP_INFO(get_logger(), "on activate");
    timer_ = this->create_wall_timer(std::chrono::milliseconds(TIMER_PERIOD_MS), std::bind(&GunControllerNode::timer_callback, this));
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn GunControllerNode::on_deactivate(const rclcpp_lifecycle::State & /* state */)
{
    RCLCPP_INFO(get_logger(), "on deactivate");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn GunControllerNode::on_cleanup(const rclcpp_lifecycle::State & /* state */)
{
    RCLCPP_INFO(get_logger(), "on cleanup");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn GunControllerNode::on_shutdown(const rclcpp_lifecycle::State & /* state */)
{
    RCLCPP_INFO(get_logger(), "on shutdown");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

#define DIRECTION_MIN -100
#define DIRECTION_MAX  100
#define POWER_MIN -100
#define POWER_MAX  100

float GunControllerNode::direction_limit(float direction)
{
    if(direction < DIRECTION_MIN)
    {
        direction = DIRECTION_MIN;
    }
    else if(direction > DIRECTION_MAX)
    {
        direction = DIRECTION_MAX;
    }
    return direction;
}

float GunControllerNode::power_limit(float power)
{
    if(power < POWER_MIN)
    {
        power = POWER_MIN;
    }
    else if(power > POWER_MAX)
    {
        power = POWER_MAX;
    }
    return power;
}

void GunControllerNode::velocity_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if(msg->data.size() >= 2)
    {
        float power = power_limit(msg->data[0]);
        float direction = direction_limit(msg->data[1]);
        target_duty.left  = (power + direction) / (float) (DIRECTION_MAX + POWER_MAX);
        target_duty.right = (power - direction) / (float) (DIRECTION_MAX + POWER_MAX);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "target_duty expects 2 elements, got %zu", msg->data.size());
    }
}

void GunControllerNode::shot_params_callback(const pingpong_msgs::msg::ShotParams::SharedPtr msg)
{
    this->position_.data = msg->pos;
    this->pose_.x = msg->roll_deg;
    this->pose_.y = msg->pitch_deg;
    this->pose_.z = msg->yaw_deg;
    this->target_duty.left = msg->pow_left;
    this->target_duty.right = msg->pow_right;
}

//! 加速度と減速度を考慮
float GunControllerNode::duty_acceldeccel_limitter(float target, float now)
{
    float a = (this->duty_accel_ * MS2S(TIMER_PERIOD_MS));
    float d = (this->duty_deccel_ * MS2S(TIMER_PERIOD_MS));
    if(target - now > 0)
    {
        if(target - now > a)
        {
            //! 加速度が想定以上の時
            return now + a;
        }
        else
        {
            return target;
        }
    }
    else if(target - now < 0)
    {
        if(now - target > d)
        {
            //! 減速度が想定以上の時
            return now - d;
        }
        else
        {
            return target;
        }
    }
    else
    {
        //! target == now のときにここに到達するはず。
        return now;
    }
}

void GunControllerNode::set_duty(gun_duty_t duty_rate_)
{
    //! 左モータのデューティ比をパブリッシュ
    auto message_left = std_msgs::msg::Float32();
    message_left.data = duty_rate_.left;
    this->duty_publisher_left_->publish(message_left);

    //! 右モータのデューティ比をパブリッシュ
    auto message_right = std_msgs::msg::Float32();
    message_right.data = duty_rate_.right;
    this->duty_publisher_right_->publish(message_right);

    now_duty = duty_rate_;
}

void GunControllerNode::set_position()
{
    auto msg = std_msgs::msg::Float32();
    msg = this->position_;
    this->position_publisher_->publish(msg);
}

void GunControllerNode::set_pose()
{
    auto msg = geometry_msgs::msg::Vector3();
    msg = this->pose_;
    this->pose_publisher_->publish(msg);
}

void GunControllerNode::set_gun()
{
    gun_duty_t tmp_duty;
    tmp_duty.left = this->duty_acceldeccel_limitter(target_duty.left, now_duty.left);
    tmp_duty.right = this->duty_acceldeccel_limitter(target_duty.right, now_duty.right);

    GunControllerNode::set_duty(tmp_duty);
}

void GunControllerNode::timer_callback()
{
    set_position();
    set_pose();
    set_gun();
}
}

