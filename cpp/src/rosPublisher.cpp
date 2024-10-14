#include "rosPublisher.hpp"
#include "tcpServerNode.hpp"
#include <algorithm>

RosPublisher::RosPublisher(const std::string& topic, const std::string& message_type, int queue_size)
	: RosSender(topic + "_RosPublisher", topic), message_type(message_type), queue_size(queue_size) {

	publisher = create_generic_publisher(topic, message_type, rclcpp::QoS(rclcpp::KeepLast(queue_size), rmw_qos_profile_sensor_data));
}

std::string RosPublisher::get_message_type() const {
	return message_type;
}

void RosPublisher::send(const RosData &message) {
	rclcpp::SerializedMessage serializedMessage { message.size() };
	auto & rcl_serializedMessage = serializedMessage.get_rcl_serialized_message();
	rcl_serializedMessage.buffer_length = message.size();
	std::copy_n(message.cbegin(), rcl_serializedMessage.buffer_length, rcl_serializedMessage.buffer);
	publisher->publish(serializedMessage);
}
