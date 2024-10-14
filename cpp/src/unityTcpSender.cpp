#include "unityTcpSender.hpp"
#include "tcpServerNode.hpp"
#include "clientThread.hpp"

UnityTcpSender::UnityTcpSender(TcpServerNode* tcp_server) : tcp_server(tcp_server), time_between_halt_checks(5), next_srv_id(1001) {
}

void UnityTcpSender::send_unity_info(const std::string& text) {
	add_to_queue(serialize_command(SysCommand::k_SysCommand_Log, SysCommand::serialize_Log(text)), Reliability::Reliable);
}

void UnityTcpSender::send_unity_warning(const std::string& text) {
	add_to_queue(serialize_command(SysCommand::k_SysCommand_Warning, SysCommand::serialize_Log(text)), Reliability::Reliable);
}

void UnityTcpSender::send_unity_error(const std::string& text) {
	add_to_queue(serialize_command(SysCommand::k_SysCommand_Error, SysCommand::serialize_Log(text)), Reliability::Reliable);
}

void UnityTcpSender::send_ros_service_response(int srv_id, const std::string& destination, const RosData& response) {
	add_to_queue(serialize_command(SysCommand::k_SysCommand_ServiceResponse, SysCommand::serialize_Service(srv_id)) + serialize_message(destination, response), Reliability::Reliable);
}

void UnityTcpSender::send_unity_message(const std::string& topic, const RosData& message, Reliability reliability) {
	add_to_queue(serialize_message(topic, message), reliability);
}

RosData UnityTcpSender::send_unity_service_request(const std::string& topic, const RosData &request) {
	std::shared_ptr<ThreadPauser> threadPauser = std::make_shared<ThreadPauser>();
	int service_id;
	{
		const std::lock_guard<std::mutex> slock(srv_lock);
		service_id = next_srv_id++;
		services_waiting[service_id] = threadPauser;
	}
	add_to_queue(serialize_command(SysCommand::k_SysCommand_ServiceRequest, SysCommand::serialize_Service(service_id)) + serialize_message(topic, request), Reliability::Reliable);
	RosData response = threadPauser->sleep_until_resumed();
	return response;
}

void UnityTcpSender::send_unity_service_response(int srv_id, const RosData& data) {
	std::shared_ptr<ThreadPauser> threadPauser;
	{
		const std::lock_guard<std::mutex> slock(srv_lock);
		auto service_data = services_waiting.find(srv_id);
		if (service_data != services_waiting.end()) {
			threadPauser = service_data->second;
			services_waiting.erase(srv_id);
		}
	}
	if (threadPauser) {
		threadPauser->resume_with_result(data);
	}
}

void UnityTcpSender::send_topic_list(const SysCommand::TopicsResponse& topics_response) {
	add_to_queue(serialize_command(SysCommand::k_SysCommand_TopicList, SysCommand::serialize_TopicsResponse(topics_response)), Reliability::Reliable);
}

void UnityTcpSender::start_sender(SOCKET socket, std::shared_ptr<StatusEvent>& halt_event) {
	// send a handshake message to confirm the connection and version number
	add_to_queue(serialize_command(SysCommand::k_SysCommand_Handshake, SysCommand::serialize_Handshake()), Reliability::Reliable);

	std::thread(&UnityTcpSender::sender_loop, this, socket, halt_event).detach();
}

void UnityTcpSender::sender_loop(SOCKET socket, std::shared_ptr<StatusEvent>& halt_event) {
	std::string data;

	try {
		while (!halt_event->is_set()) {
			if (try_remove_from_queue(data)) {
				if (SOCKET_ERROR == send(socket, data.c_str(), static_cast<int>(data.size()), 0)) {
					int err = WSAGetLastError();
					tcp_server->log_error("socket send error %d", err);
					if (WSAECONNABORTED == err || WSAECONNRESET == err)
						break;
				}
			}
		}
	} catch (std::exception ex) {
		tcp_server->log_error("exception occured in UnityTcpSender: %s", ex.what());
	}
	// make sure this one is called
	halt_event->set();
}

std::string UnityTcpSender::serialize_message(const std::string& destination, const RosData& message) {
	/*  Serialize a destination and message class.

		Args:
			destination: name of destination
			message:     message class to serialize

		Returns:
			serialized destination and message as a list of bytes
			*/
	std::ostringstream oss;
	append_string(oss, destination);
	append_ros_data(oss, message);

	return oss.str();
}

std::string UnityTcpSender::serialize_command(const std::string& command, const std::string& params) {
	//little endian.  uint32 taille de la command, puis les octets de la commande
	std::ostringstream oss;
	append_string(oss, command);
	//todo: encodage utf - 8 des params, à voir si nécessaire
	append_string(oss, params);

	return oss.str();
}

void UnityTcpSender::add_to_queue(const std::string& data, Reliability reliability) {
	{
		const std::lock_guard<std::mutex> qlock(queue_lock);
		switch (reliability) {
		case Reliability::Reliable:
			reliable_queue.push(data);
			break;
		case Reliability::BestEffort:
			if (best_effort_queue.size() == best_effort_queue_max_size) {
				// if queue is already full, remove oldest element
				best_effort_queue.pop();
			}
			best_effort_queue.push(data);
		}
	}
	queue_condition.notify_one();
}

bool UnityTcpSender::try_remove_from_queue(std::string& data) {
	std::unique_lock<std::mutex> qlock(queue_lock);

	queue_condition.wait_for(qlock, time_between_halt_checks, [this] { return !(reliable_queue.empty() && best_effort_queue.empty()); });
	if (!reliable_queue.empty()) {
		data = std::move(reliable_queue.front());
		reliable_queue.pop();
		return true;
	}
	if (!best_effort_queue.empty()) {
		data = std::move(best_effort_queue.front());
		best_effort_queue.pop();
		return true;
	}
	return false;
}

void UnityTcpSender::append_size(std::ostringstream& oss, uint32_t size) {
	oss.put(size & 0xff);
	oss.put(size >> 8 & 0xff);
	oss.put(size >> 16 & 0xff);
	oss.put(size >> 24 & 0xff);
}

void UnityTcpSender::append_string(std::ostringstream& oss, const std::string& data) {
	uint32_t size = static_cast<uint32_t>(data.size());
	append_size(oss, static_cast<uint32_t>(data.size()));
	oss << data;
}
void UnityTcpSender::append_ros_data(std::ostringstream& oss, const RosData& ros_data) {
	uint32_t size = static_cast<uint32_t>(ros_data.size());
	append_size(oss, size);
	oss.write(reinterpret_cast<const char*>(ros_data.data()), size);
}

