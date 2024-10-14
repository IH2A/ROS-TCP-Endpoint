#pragma once
#include "rosCommunication.hpp"
#include <memory>

class UnityTcpSender;

class RosSubscriber : public RosReceiver {
public:
	RosSubscriber(UnityTcpSender *unity_tcp_sender, const std::string &topic, const std::string &message_type, int queue_size = 10);

	std::string get_message_type() const override;

private:
	void send(std::shared_ptr<rclcpp::SerializedMessage> serialized_message);

	std::string message_type;
	int queue_size;
	rclcpp::GenericSubscription::SharedPtr subscription;
	RosData ros_data;
};
