#pragma once
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <utility>
#include <cstdint>
#include <regex>

class UnityTcpSender;

using RosData = std::vector<uint8_t>;

void DebugRosData(const char *title, const RosData& rosData);

class RosNode : public rclcpp::Node {	// Base class for ROS communication
public:
	virtual std::string get_message_type() const;

protected :
	RosNode(const std::string& node_name, const std::string &topic);

	std::string topic;

private :
	static std::string strip_topic_name(const std::string& topic);

	static std::regex strip_topic_name_regex;
};

class RosSender : public RosNode {	// Base class for ROS communication where data is sent to the ROS network.
public:
	RosSender(const std::string& node_name, const std::string &topic);
private:
};

class RosReceiver : public RosNode {	// Base class for ROS communication where data is being sent outside of the ROS network.
public:
	RosReceiver(UnityTcpSender* unity_tcp_sender, const std::string& node_name, const std::string& topic);
protected:
	UnityTcpSender* unity_tcp_sender;
};

