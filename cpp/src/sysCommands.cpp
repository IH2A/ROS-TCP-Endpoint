#include "sysCommands.hpp"
#include "json.hpp"
using json = nlohmann::json;

const std::string SysCommand::k_SysCommand_Handshake{ "__handshake" };
const std::string SysCommand::k_SysCommand_Log{ "__log" };
const std::string SysCommand::k_SysCommand_Warning{ "__warn" };
const std::string SysCommand::k_SysCommand_Error{ "__error" };
const std::string SysCommand::k_SysCommand_ServiceRequest{ "__request" };
const std::string SysCommand::k_SysCommand_ServiceResponse{ "__response" };
const std::string SysCommand::k_SysCommand_Subscribe{ "__subscribe" };
const std::string SysCommand::k_SysCommand_Publish{ "__publish" };
const std::string SysCommand::k_SysCommand_RosService{ "__ros_service" };
const std::string SysCommand::k_SysCommand_UnityService{ "__unity_service" };
const std::string SysCommand::k_SysCommand_TopicList{ "__topic_list" };
const std::string SysCommand::k_SysCommand_RemoveSubscriber{ "__remove_subscriber" };
const std::string SysCommand::k_SysCommand_RemovePublisher{ "__remove_publisher" };
const std::string SysCommand::k_SysCommand_RemoveRosService{ "__remove_ros_service" };
const std::string SysCommand::k_SysCommand_RemoveUnityService{ "__remove_unity_service" };

std::string SysCommand::serialize_Log(const std::string& text) {
	return json{ { "text", text } }.dump();
}

std::string SysCommand::serialize_Service(int srv_id) {
	return json{ { "srv_id", srv_id } }.dump();
}

std::string SysCommand::serialize_TopicsResponse(const TopicsResponse& topics_response) {
	return json{ {"topics", topics_response.topics}, {"types", topics_response.types} }.dump();
}

std::string SysCommand::serialize_Handshake() {
	std::string version{ "v0.7.0" };
	std::string protocol{ "ROS2" };
	std::string metadata = json{ { "protocol", protocol } }.dump();
	return json{ {"version", version}, {"metadata", metadata } }.dump();
}

bool SysCommand::deserialize(const std::string& data, TopicAndType& result) {
	json j = json::parse(data, nullptr, false);
	if (j.is_discarded())
		return false;
	j["topic"].get_to(result.topic);
	j["message_name"].get_to(result.message_name);
	return true;
}

bool SysCommand::deserialize(const std::string& data, PublisherRegistration& result) {
	json j = json::parse(data, nullptr, false);
	if (j.is_discarded())
		return false;
	j["topic"].get_to(result.topic);
	j["message_name"].get_to(result.message_name);
	j["queue_size"].get_to(result.queue_size);
	j["latch"].get_to(result.latch);
	return true;
}

bool SysCommand::deserialize(const std::string& data, Service& result) {
	json j = json::parse(data, nullptr, false);
	if (j.is_discarded())
		return false;
	j["srv_id"].get_to(result.srv_id);
	return true;
}

