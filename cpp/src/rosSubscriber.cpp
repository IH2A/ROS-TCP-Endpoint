#include "rosSubscriber.hpp"
#include "tcpServerNode.hpp"

RosSubscriber::RosSubscriber(UnityTcpSender* unity_tcp_sender, const std::string& topic, const std::string& message_type, int queue_size)
	: RosReceiver(unity_tcp_sender, topic + "_RosSubscriber", topic), message_type(message_type), queue_size(queue_size) {

	subscription = create_generic_subscription(
		topic, 
		message_type, 
		rclcpp::QoS(rclcpp::KeepLast(queue_size), rmw_qos_profile_sensor_data),
		std::bind(&RosSubscriber::send, this, std::placeholders::_1)
	);
}

std::string RosSubscriber::get_message_type() const {
	return message_type;
}

void RosSubscriber::send(std::shared_ptr<rclcpp::SerializedMessage> serialized_message) {
	auto& rcl_serializedMessage = serialized_message->get_rcl_serialized_message();
	ros_data.resize(rcl_serializedMessage.buffer_length);
	std::copy_n(rcl_serializedMessage.buffer, rcl_serializedMessage.buffer_length, ros_data.begin());
	unity_tcp_sender->send_unity_message(topic, ros_data, UnityTcpSender::Reliability::BestEffort);
}
