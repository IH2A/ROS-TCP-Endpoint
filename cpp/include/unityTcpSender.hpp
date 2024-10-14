#pragma once
#include <string>
#include <queue>
#include <sstream>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <map>
#include <memory>
#include <chrono>
#include "statusEvent.hpp"
#include <WinSock2.h>
#include "rosCommunication.hpp"
#include "threadPauser.hpp"
#include "sysCommands.hpp"

class TcpServerNode;


class UnityTcpSender {
public:
	enum class Reliability {
		Reliable,
		BestEffort
	};

	UnityTcpSender(TcpServerNode *tcp_server);

    void send_unity_info(const std::string &text);
    void send_unity_warning(const std::string& text);
    void send_unity_error(const std::string& text);
    void send_ros_service_response(int srv_id, const std::string& destination, const RosData& response);
    void send_unity_message(const std::string& topic, const RosData& message, Reliability reliability);
	RosData send_unity_service_request(const std::string& topic, const RosData& request);
    void send_unity_service_response(int srv_id, const RosData& data);
    void send_topic_list(const SysCommand::TopicsResponse &topics_response);
	void start_sender(SOCKET socket, std::shared_ptr<StatusEvent>& event);

private:
	void sender_loop(SOCKET socket, std::shared_ptr<StatusEvent>& event);
	std::string serialize_message(const std::string& destination, const RosData& message);
	std::string serialize_command(const std::string& command, const std::string& params);
	void add_to_queue(const std::string &data, Reliability reliablity);
	bool try_remove_from_queue(std::string &data);

	static void append_size(std::ostringstream& oss, uint32_t size);
	static void append_string(std::ostringstream& oss, const std::string& data);
	static void append_ros_data(std::ostringstream& oss, const RosData& ros_data);

	TcpServerNode *tcp_server;
	std::chrono::seconds time_between_halt_checks;
	size_t best_effort_queue_max_size{ 50 };

	//Each sender thread has its own queue: this is always the queue for the currently active thread.
	std::queue<std::string> reliable_queue, best_effort_queue;
	std::mutex queue_lock;
	std::condition_variable queue_condition;

	// variables needed for matching up unity service requests with responses
	int next_srv_id;	
	std::mutex srv_lock;
	std::map<int, std::shared_ptr<ThreadPauser>> services_waiting;
};
