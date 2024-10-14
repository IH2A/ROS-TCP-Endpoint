#include "rosCommunication.hpp"

std::regex RosNode::strip_topic_name_regex{"[^A-Za-z0-9_]"};	// node name must not contain characters other than alphanumerics or '_'

RosNode::RosNode(const std::string& node_name, const std::string& topic) : rclcpp::Node(strip_topic_name(node_name)), topic(topic) {
}

std::string RosNode::get_message_type() const {
	return std::string{};
}

std::string RosNode::strip_topic_name(const std::string& topic) {
	try {
		return std::regex_replace(topic, strip_topic_name_regex, "");
	} catch (const std::regex_error&) {
		return topic;
	}
}

RosSender::RosSender(const std::string& node_name, const std::string& topic) : RosNode(node_name, topic) {}

RosReceiver::RosReceiver(UnityTcpSender* unity_tcp_sender, const std::string& node_name, const std::string& topic) : RosNode(node_name, topic), unity_tcp_sender(unity_tcp_sender) {}

void DebugRosData(const char *title, const RosData& rosData) {
	std::cout << title << ", size = " << rosData.size() << " : ";
	for (const auto& i : rosData) {
		std::cout << (int)i << " ";
	}
	std::cout << std::endl;
}