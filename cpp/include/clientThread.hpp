#pragma once
#include <WinSock2.h>
#include <memory>
#include <string>
#include <map>
#include <regex>
#include <functional>
#include "unityTcpSender.hpp"
#include "rosSubscriber.hpp"
#include "rosPublisher.hpp"
#include "rosService.hpp"
#include "unityService.hpp"
#include "rosCommunication.hpp"

class TcpServerNode;

class ClientThread {
public:
	ClientThread(TcpServerNode *tcp_server, SOCKET socket, const sockaddr_in& remote);

	void start();
	void halt();
	void wait();
	bool is_finished() const;

	void recvall(char *buffer, int size);
	int32_t read_int32();
	std::string read_string();
	std::pair<std::string, std::string> read_message();

	void send_ros_service_request(int srv_id, const std::string& destination, const RosData&data);
	void service_call_thread(int srv_id, const std::string& destination, const RosData&data, std::shared_ptr<RosService>& rosService);
	std::shared_ptr<RosNode> get_node_for_topic(const std::string& topic);

private:
	void run();

	void register_subscriber(const std::string& topic, const std::string& message_type);
	void register_publisher(const std::string& topic, const std::string& message_type, int queue_size, bool latch);
	void register_ros_service(const std::string& topic, const std::string& request_message_type);
	void register_unity_service(const std::string& topic, const std::string& request_message_type);
	void unregister_all();

	void send_topic_list();

	std::string parse_message_name(const std::string& name);
	bool get_ros_data(const std::string& data, RosData &result);
	void handle_syscommand(const std::string& command, const std::string& data);
	void deserialize_error(const std::string& command, const std::string& data);
	std::string get_message_type(const std::string& message_name);

	TcpServerNode* tcp_server{ nullptr };
	UnityTcpSender unity_tcp_sender;
	SOCKET socket;
	sockaddr_in remote;
	std::thread thread;
	std::shared_ptr<StatusEvent> halt_event;
	std::regex parse_message_regex;

	std::map<std::string, std::shared_ptr<RosPublisher>> publishers_table;
	std::map<std::string, std::shared_ptr<RosSubscriber>> subscribers_table;
	std::map<std::string, std::shared_ptr<RosService>> ros_services_table;
	std::map<std::string, std::shared_ptr<UnityService>> unity_services_table;

	RosData rosData;
	int pending_srv_id{ 0 };
	bool pending_srv_is_request{ false };
	bool run_is_finished{ false };

	static constexpr int NO_PENDING_SERVICE_ID = -1;
	static const std::string ROS2_HEADER;
};
