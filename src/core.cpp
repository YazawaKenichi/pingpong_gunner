#include "pingpong_gunner/core.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>

namespace GunControllerNodes
{
GunControllerNode::GunControllerNode() : rclcpp_lifecycle::LifecycleNode("gun_controller_node")
{
    RCLCPP_INFO(get_logger(), "Gun Controller Node constructed.");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn GunControllerNode::on_configure(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "on configure");
    this->duty_publisher_left_ = this->create_publisher<std_msgs::msg::Float32>("/pico/gun/left/pwm/duty", 10);
    this->duty_publisher_right_ = this->create_publisher<std_msgs::msg::Float32>("/pico/gun/right/pwm/duty", 10);
    this->duty_subscriber_velocity_ = this->create_subscription<std_msgs::msg::MultiArray>("/pico/gun/velocity", 10, std::bind(&velocity_callback, this, std::placeholders::_1));
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn GunControllerNode::on_activate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "on activate");
    timer_ = this->create_wall_timer(std::chrono::milliseconds(TIMER_PERIOD_MS), std::bind(&GunControllerNode::timer_callback, this));
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn GunControllerNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "on deactivate");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn GunControllerNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "on cleanup");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn GunControllerNode::on_shutdown(const rclcpp_lifecycle::State & state)
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
        float direction = direction_limit(msg->data[0]);
        float power = power_limit(msg->data[1]);
        target_duty.left  = (power + direction) / (float) (DIRECTION_MAX + POWER_MAX);
        target_duty.right = (power - direction) / (float) (DIRECTION_MAX + POWER_MAX);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "target_duty expects 2 elements, got %zu", msg->data.size());
    }
}

//! 加速度と減速度を考慮
float GunControllerNode::duty_acceldeccel_limitter(float target, float now)
{
    if(target - now > 0)
    {
        float ad = (DUTY_ACCEL * MS2S(TIMER_PERIOD_MS));
        if(target - now > ad)
        {
            //! 加速度が想定以上の時
            return now + ad;
        }
        else
        {
            return target;
        }
    }
    if(target - now < 0)
    {
        float ad = (DUTY_DECCEL * MS2S(TIMER_PERIOD_MS));
        if(now - target > ad)
        {
            //! 減速度が想定以上の時
            return now - ad;
        }
        else
        {
            return target;
        }
    }
    //! target == now のときにここに到達するはず。
    return now;
}

void GunControllerNode::set_duty(gun_duty_t duty_rate_)
{
    //! 左モータのデューティ比をパブリッシュ
    auto message_left = std_msgs::msg::Float32();
    message_left.data = duty_rate_.left;
    this->duty_publisher_left_->publish(message_left);
    now_duty.left = duty_rate_.left;

    //! 右モータのデューティ比をパブリッシュ
    auto message_right = std_msgs::msg::Float32();
    message_right.data = duty_rate_.right;
    this->duty_publisher_right_->publish(message_right);
    now_duty.right = duty_rate_.right;
}

void GunControllerNode::timer_callback()
{
    gun_duty_t tmp_duty;
    ////////// target_duty の更新をするプログラムを書いてないのでこのままだと動かない //////////
    tmp_duty.left = this->duty_acceldeccel_limitter(target_duty.left, now_duty.left);
    tmp_duty.right = this->duty_acceldeccel_limitter(target_duty.right, now_duty.right);

    GunControllerNode::set_duty(tmp_duty);
}
}


