
#pragma once
#include "rosCommunication.hpp"
#include <memory>

class RosPublisher : public RosSender {
public:
	RosPublisher(const std::string& topic, const std::string &message_type, int queue_size = 10);

	std::string get_message_type() const override;
	void send(const RosData &message);

private:
	std::string message_type;
	int queue_size;
	rclcpp::GenericPublisher::SharedPtr publisher;
};
