#include <bupt_interfaces/msg/joystick.hpp>
#include <bupt_interfaces/srv/joy_stick_interaction.hpp>
#include <cmath>
#include <condition_variable>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>

template <typename T>
typename std::enable_if<std::is_fundamental<T>::value, T>::type clamp(const T &val, const T &min, const T &max) {
	if (val < min)
		return min;
	else if (val > max)
		return max;
	else
		return val;
}


class Joystick_Vel_Server : public rclcpp::Node {
private:
	rclcpp::Subscription<bupt_interfaces::msg::Joystick>::SharedPtr joysub_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velpub_;
	rclcpp::Service<bupt_interfaces::srv::JoyStickInteraction>::SharedPtr joysrv_;

	double max_linear_speed;
	double max_angular_speed;

	double prev_linear_x = 0.0;
	double prev_linear_y = 0.0;
	double prev_angular_z = 0.0;

	static constexpr double MAX_ACTION[4] = { 1616, 1665, 1681, 1617 };
	static constexpr double eps = 1e-2;
	/* 指数滑动平均 (alpha 控制平滑度，0.0~1.0) */
	static constexpr double alpha = 0.2;

	bool cond_joy_;

	std::mutex mtx_;
	std::condition_variable cv_;

	void joysub_callback(bupt_interfaces::msg::Joystick::SharedPtr msg);
	void joysrv_callback(bupt_interfaces::srv::JoyStickInteraction::Request::SharedPtr req);

public:
	Joystick_Vel_Server(const std::string &name, double max_linear_speed, double max_angular_speed);
};

Joystick_Vel_Server::Joystick_Vel_Server(const std::string &name, double max_linear_speed, double max_angular_speed) :
		Node(name),
		max_linear_speed(max_linear_speed),
		max_angular_speed(max_angular_speed),
		cond_joy_(true),
		mtx_(std::mutex()),
		cv_(std::condition_variable()) {
	joysub_ = this->create_subscription<bupt_interfaces::msg::Joystick>(
			"joystick", 10,
			std::bind(&Joystick_Vel_Server::joysub_callback, this, std::placeholders::_1));
	velpub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
	joysrv_ = this->create_service<bupt_interfaces::srv::JoyStickInteraction>(
			"joyvel_srv", std::bind(&Joystick_Vel_Server::joysrv_callback, this, std::placeholders::_1));
}

void Joystick_Vel_Server::joysub_callback(bupt_interfaces::msg::Joystick::SharedPtr msg) {
	{
		std::unique_lock<std::mutex> lock(mtx_);
		cv_.wait(lock, [this] {
			return cond_joy_;
		});
	}

	auto twist = geometry_msgs::msg::Twist();
	double coe = std::hypot(msg->action[0], msg->action[1]) / std::hypot(MAX_ACTION[0], MAX_ACTION[1]);
	double current_speed = max_linear_speed * coe;
	double target_linear_x = current_speed * (msg->action[0] * 1.0 / MAX_ACTION[0]);
	double target_linear_y = current_speed * (msg->action[1] * 1.0 / MAX_ACTION[1]);

	twist.linear.x = alpha * target_linear_x + (1 - alpha) * prev_linear_x;
	twist.linear.y = alpha * target_linear_y + (1 - alpha) * prev_linear_y;

	if (std::abs(twist.linear.x) < eps)
		twist.linear.x = 0;
	if (std::abs(twist.linear.y) < eps)
		twist.linear.y = 0;

	prev_linear_x = twist.linear.x;
	prev_linear_y = twist.linear.y;

	coe = std::hypot(msg->action[2], msg->action[3]) / std::hypot(MAX_ACTION[2], MAX_ACTION[3]);
	current_speed = max_angular_speed * coe;
	if (msg->action[3] > 0) {
		current_speed = -current_speed;
	}
	double target_angular_z = current_speed;
	twist.angular.z = alpha * target_angular_z + (1-alpha) * prev_angular_z;
	if (std::abs(twist.angular.z) < eps)
		twist.angular.z = 0;
	prev_angular_z = twist.angular.z;

	velpub_->publish(twist);
}

void Joystick_Vel_Server::joysrv_callback(
		bupt_interfaces::srv::JoyStickInteraction::Request::SharedPtr req) {
	if (req->open == true) {
		if (!cond_joy_) {
			mtx_.lock();
			cond_joy_ = true;
			mtx_.unlock();
			cv_.notify_one();
		}
	} else {
		if (cond_joy_) {
			mtx_.lock();
			cond_joy_ = false;
			mtx_.unlock();
		}
	}
}

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Joystick_Vel_Server>("joystick_vel_server", 10, 5));
	rclcpp::shutdown();
	return 0;
}
