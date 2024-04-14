#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <type_traits>

#include "bupt_interfaces/msg/joystick.hpp"

template <typename T>
typename std::enable_if<std::is_fundamental<T>::value, T>::type
clamp(const T &val, const T &min, const T &max)
{
    if (val < min)
        return min;
    else if (val > max)
        return max;
    else
        return val;
}

class JoystickController : public rclcpp::Node
{
private:
    rclcpp::Subscription<bupt_interfaces::msg::Joystick>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    double max_linear_speed;
    double max_angular_speed;
    double max_linear_acc;

    double last_linear_acc;

    double current_speed_x;
    double current_speed_y;
    double current_rotation;

    static constexpr double MAX_ACTION[4] = {1616, 1665, 1681, 1617};
    static constexpr double eps = 1e-2;

public:
    JoystickController(double max_linear_speed, double max_angular_speed, double max_linear_acc) : Node("joystick_controller"), max_linear_speed(max_linear_speed), max_linear_acc(max_linear_acc),
                                                                                                   max_angular_speed(max_angular_speed), current_speed_x(0), current_speed_y(0), current_rotation(0)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        subscription_ = this->create_subscription<bupt_interfaces::msg::Joystick>(
            "joystick", 10, [this](const bupt_interfaces::msg::Joystick::SharedPtr msg_ptr)
            { publish(msg_ptr); });
    }
    void publish(const bupt_interfaces::msg::Joystick::SharedPtr &msg_ptr)
    {
        auto twist = geometry_msgs::msg::Twist();
        auto msg = *msg_ptr;
        double speed = 0, rotation = 0, current_x = 0, current_y = 0;

        if (abs(msg.action[1]) < 1400)
        {
            twist.angular.z = 0;
        }
        else
        {
            twist.angular.z = max_angular_speed * msg.action[1] / MAX_ACTION[1];
        }

        if (msg.action[0] < -50)
        {
            current_rotation = 0;
            current_speed_x = 0;
            current_speed_y = 0;
            twist.linear.x = 0;
            twist.linear.y = 0;
            twist.linear.z = 0;
            twist.angular.x = 0;
            twist.angular.y = 0;
            twist.angular.z = 0;
            publisher_->publish(twist);
            return;
        }
        double current_acc = 0;
        if (msg.action[0] > 100)
        {
            current_acc = max_linear_acc * (msg.action[0] / MAX_ACTION[0]);
        }

        double current_speed = hypot(current_speed_x, current_speed_y);
        if (current_speed > eps)
        {
            current_x = current_speed_x / current_speed;
            current_y = current_speed_y / current_speed;
        }
        else
        {
            current_x = 0;
            current_y = 0;
        }

        double msg_x = 0, msg_y = 0;
        if (abs(msg.action[2]) > 200 || abs(msg.action[3]) > 200)
        {
            auto r = hypot(msg.action[2], msg.action[3]);
            msg_x = msg.action[2] / r;
            msg_y = msg.action[3] / r;
        }

        double target_acc_x = 2 * msg_x - current_x;
        double target_acc_y = 2 * msg_y - current_y;
        auto r = hypot(target_acc_x, target_acc_y);

        double target_acc = 0;
        if (r > eps)
            target_acc = hypot(target_acc_x, target_acc_y) / r * current_acc;
        else
            target_acc = current_acc;
        target_acc_x = target_acc * target_acc_x;
        target_acc_y = target_acc * target_acc_y;

        current_speed_x += target_acc_x * 0.02;
        current_speed_y += target_acc_y * 0.02;

        current_speed = hypot(current_speed_x, current_speed_y);
        auto current_speed_norm = clamp(current_speed, 0.0, max_linear_speed);

        if (current_speed > eps)
        {
            current_speed_x = current_speed_x / current_speed * current_speed_norm;
            current_speed_y = current_speed_y / current_speed * current_speed_norm;
        }
        else
        {
            current_speed_x = 0;
            current_speed_y = 0;
        }

        twist.linear.x = current_speed_x;
        twist.linear.y = current_speed_y;
        twist.linear.z = 0;

        publisher_->publish(twist);
    }
};

class RAII
{
    std::shared_ptr<rclcpp::Node> node;

public:
    RAII(int argc, char *argv[])
    {
        rclcpp::init(argc, argv);
        node = std::make_shared<JoystickController>(10, 5, 5);
        rclcpp::spin(node);
    }
    ~RAII()
    {
        rclcpp::shutdown();
    }
};

int main(int argc, char *argv[])
{
    auto raii = RAII(argc, argv);
    return 0;
}