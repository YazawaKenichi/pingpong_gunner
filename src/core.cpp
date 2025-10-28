#include "pingpong_gunner/core.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

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


