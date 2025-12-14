#ifndef __CORE_HPP__
#define __CORE_HPP__

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <pingpong_msgs/msg/shot_params.hpp>

#define DUTY_ACCEL_DEFAULT  10
#define DUTY_DECCEL_DEFAULT 10

#define DIRECTION_MIN -100
#define DIRECTION_MAX  100
#define POWER_MIN -100
#define POWER_MAX  100
#define DUTY_MIN -40
#define DUTY_MAX 40

#define TIMER_PERIOD_MS 1

#define MS2S(X) ((X) / (float) 1000.0f)
#define RESCALE(X, I_MAX, I_MIN, O_MAX, O_MIN) ((O_MAX - O_MIN) * (float) (((X) - I_MIN) / (float) (I_MAX - I_MIN)) + O_MIN)

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
        void shot_params_callback(const pingpong_msgs::msg::ShotParams::SharedPtr);

        float direction_limit(float);
        float power_limit(float);
        float duty_acceldeccel_limitter(float, float);
        void set_position();
        void set_pose();
        void set_gun();
        void set_duty(gun_duty_t);

        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr duty_publisher_left_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr duty_publisher_right_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr position_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pose_publisher_;
        rclcpp::Subscription<pingpong_msgs::msg::ShotParams>::SharedPtr shot_params_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr duty_subscriber_velocity_;
        rclcpp::TimerBase::SharedPtr timer_;

        gun_duty_t target_duty;
        gun_duty_t now_duty;
        std_msgs::msg::Float32 position_;
        geometry_msgs::msg::Vector3 pose_;
        float duty_accel_;
        float duty_deccel_;
};
}

#endif

