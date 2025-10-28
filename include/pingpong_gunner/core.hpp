#ifndef __CORE_HPP__
#define __CORE_HPP__

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>

#define DUTY_ACCEL  10
#define DUTY_DECCEL 10

#define TIMER_PERIOD_MS 1

#define MS2S(X) (X / (float) 1000.0f)

typedef struct
{
    float left;
    float right;
} gun_duty_t;

namespace GunControllerNodes
{
class GunControllerNode : public rclcpp_lifecycle::LifecycleNode
{
    public:
        GunControllerNode();
    protected:
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

    private:
        void timer_callback();
        void velocity_callback(const std_msgs::msg::Float32MultiArray::SharedPtr);

        float direction_limit(float);
        float power_limit(float);
        float duty_acceldeccel_limitter(float, float);
        void set_duty(gun_duty_t);

        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr duty_publisher_left_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr duty_publisher_right_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr duty_subscriber_velocity_;
        rclcpp::TimerBase::SharedPtr timer_;

        gun_duty_t target_duty;
        gun_duty_t now_duty;
};
}

#endif

