#include <stdexcept>
#include <thread>
#include <sstream>
#include <regex>
#include "clientThread.hpp"
#include "tcpServerNode.hpp"

const std::string ClientThread::ROS2_HEADER { 0, 1, 0, 0 };

ClientThread::ClientThread(TcpServerNode* tcp_server, SOCKET socket, const sockaddr_in &remote)
    : tcp_server(tcp_server), unity_tcp_sender(tcp_server), socket(socket), remote(remote) {
    try {
        parse_message_regex.assign(R"(<class '([^.]+)\.msg.*_([^_]+)'>)", std::regex_constants::ECMAScript | std::regex_constants::optimize);
    } catch (const std::regex_error& err) {
        std::cerr << "ClientThread - Unable to build regex :" << err.what() << std::endl;
    }
}

void ClientThread::start() {
    halt_event = std::make_shared<StatusEvent>();
    thread = std::thread(&ClientThread::run, this);
}

void ClientThread::halt() {
    halt_event->set();
}

void ClientThread::wait() {
	if (thread.joinable()) {
        halt_event->set();
		thread.join();
	}
    halt_event.reset();
}

bool ClientThread::is_finished() const {
    return run_is_finished;
}

void ClientThread::recvall(char *buffer, int size) {
    int pos = 0;
    while (pos < size) {
        int read = recv(socket, buffer + pos, size - pos, 0);
        if (SOCKET_ERROR == read) {
            tcp_server->log_error("recvall - Unable to read from socket: %d", WSAGetLastError());
            throw std::runtime_error{"socket error"};
        }
        pos += read;
    }
}

int32_t ClientThread::read_int32() {
    uint8_t buffer[4]{};
    recvall(reinterpret_cast<char*>(buffer), 4);
    return buffer[0] | buffer[1] << 8 | buffer[2] << 16 | buffer[3] << 24;
}

std::string ClientThread::read_string() {
    int32_t str_len = read_int32();
    std::string result{};
    if (str_len > 0) {
        result.resize(str_len);
        recvall(result.data(), str_len);
        // trim right to remove padding and null termination
        while (!result.empty() && result.back() == '\0') {
            result.pop_back();
        }
    }
    return result;
}

std::pair<std::string, std::string> ClientThread::read_message() {
    /*  Decode destination and full message size from socket connection.
        Grab bytes in chunks until full message has been read.
        */
    std::string destination = read_string();
    int32_t full_message_size = read_int32();
    std::string data{};
    if (full_message_size > 0) {
        data.resize(full_message_size);
        recvall(data.data(), full_message_size);
    }
    return std::make_pair(destination, data);
}

void ClientThread::send_ros_service_request(int srv_id, const std::string& destination, const RosData &data) {
    const auto& ros_communicator_iter = ros_services_table.find(destination);
    if (ros_communicator_iter != ros_services_table.cend()) {
        std::thread(&ClientThread::service_call_thread, this, srv_id, destination, data, ros_communicator_iter->second).detach();
    }
}

void ClientThread::service_call_thread(int srv_id, const std::string& destination, const RosData &data, std::shared_ptr<RosService>& rosService) {
    RosData response;
    if (!rosService->send(data, response)) {
        std::string error_msg = "No response data from service '" + destination + "'!";
        unity_tcp_sender.send_unity_error(error_msg);
        tcp_server->log_error(error_msg.c_str());
        return;
    }
    unity_tcp_sender.send_ros_service_response(srv_id, destination, response);
}

std::shared_ptr<RosNode> ClientThread::get_node_for_topic(const std::string& topic) {
    if (auto publisher = publishers_table.find(topic); publisher != publishers_table.end()) {
        return std::reinterpret_pointer_cast<RosNode>(publisher->second);
    }
    if (auto subscriber = subscribers_table.find(topic); subscriber != subscribers_table.end()) {
        return std::reinterpret_pointer_cast<RosNode>(subscriber->second);
    }

    if (auto ros_service = ros_services_table.find(topic); ros_service != ros_services_table.end()) {
        return std::reinterpret_pointer_cast<RosNode>(ros_service->second);
    }
    if (auto unity_service = unity_services_table.find(topic); unity_service != unity_services_table.end()) {
        return std::reinterpret_pointer_cast<RosNode>(unity_service->second);
    }
    return std::shared_ptr<RosNode>();
}


void ClientThread::run() {
    /*
        Receive a message from Unity and determine where to send it based on the publishers table
        and topic string.Then send the read message.

        If there is a response after sending the serialized data, assume it is a
        ROS service response.

        Message format is expected to arrive as
        int: length of destination bytes
        str : destination.Publisher topic, Subscriber topic, Service name, etc
        int : size of full message
        msg : the ROS msg type as bytes
        */

    pending_srv_id = NO_PENDING_SERVICE_ID;
    pending_srv_is_request = false;

    const auto& remote_addr = remote.sin_addr.S_un.S_un_b;
    tcp_server->log_info("Connection from %hhu.%hhu.%hhu.%hhu:%hu", remote_addr.s_b1, remote_addr.s_b2, remote_addr.s_b3, remote_addr.s_b4, ntohs(remote.sin_port));
    unity_tcp_sender.start_sender(socket, halt_event);

    try {
        RosData ros_data{};
        while (!halt_event->is_set()) {
            const auto [destination, data] = read_message();
            if (!destination.empty()) { // ignore keepalive message (empty destination), listen for more
                //tcp_server->log_debug("Received %d bytes for %s", data.size(), destination);
                // Process this message that was sent from Unity
                if (pending_srv_id != NO_PENDING_SERVICE_ID) {
                    // if we've been told that the next message will be a service request/response, process it as such
                    if (!get_ros_data(data, ros_data)) {
                        throw std::runtime_error{ "Invalid ros data, destination : " + destination };
                    }
                    if (pending_srv_is_request) {
                        send_ros_service_request(pending_srv_id, destination, ros_data);
                    } else {
                        unity_tcp_sender.send_unity_service_response(pending_srv_id, ros_data);
                    }
                    pending_srv_id = NO_PENDING_SERVICE_ID;
                } else if (destination.substr(0, 2) == "__") {
                    // handle a system command, such as registering new topics
                    handle_syscommand(destination, data);
                } else {
                    auto publisher = publishers_table.find(destination);
                    if (publisher != publishers_table.end()) {
                        if (get_ros_data(data, ros_data)) {
                            publisher->second->send(ros_data);
                        } else {
                            std::string error_msg = "Invalid ros data, destination : " + destination;
                            unity_tcp_sender.send_unity_error(error_msg);
                            tcp_server->log_error(error_msg.c_str());
                        }
                    } else {
                        std::ostringstream oss;
                        oss << "Not registered to publish topic '" << destination << "'! Valid publish topics are:";
                        for (const auto& iter : publishers_table) {
                            oss << iter.first << ". ";
                        }
                        std::string error_msg = oss.str();
                        unity_tcp_sender.send_unity_error(error_msg);
                        tcp_server->log_error(error_msg.c_str());
                    }
                }
            }
        }
    } catch (std::exception ex) {
        tcp_server->log_error("exception occured in ClientThread: %s", ex.what());
    }
    halt_event->set();
    closesocket(socket);
    unregister_all();
    tcp_server->log_info("Disconnected from %hhu.%hhu.%hhu.%hhu:%hu", remote_addr.s_b1, remote_addr.s_b2, remote_addr.s_b3, remote_addr.s_b4, ntohs(remote.sin_port));

    run_is_finished = true;
}

void ClientThread::register_subscriber(const std::string& topic, const std::string& message_type) {
    auto& old_node = subscribers_table.find(topic);
    if (old_node != subscribers_table.end()) {
        tcp_server->unregister_node(std::reinterpret_pointer_cast<RosNode>(old_node->second));
    }
    std::shared_ptr<RosSubscriber> new_subscriber = std::make_shared<RosSubscriber>(&unity_tcp_sender, topic, message_type);
    subscribers_table[topic] = new_subscriber;
    tcp_server->register_node(std::reinterpret_pointer_cast<RosNode>(new_subscriber));
    tcp_server->log_info("RegisterSubscriber(%s, %s)", topic.c_str(), message_type.c_str());
}

void ClientThread::register_publisher(const std::string& topic, const std::string& message_type, int queue_size, bool latch) {
    auto& old_node = publishers_table.find(topic);
    if (old_node != publishers_table.end()) {
        tcp_server->unregister_node(std::reinterpret_pointer_cast<RosNode>(old_node->second));
    }
    std::shared_ptr<RosPublisher> new_publisher = std::make_shared<RosPublisher>(topic, message_type, queue_size);
    publishers_table[topic] = new_publisher;
    tcp_server->register_node(std::reinterpret_pointer_cast<RosNode>(new_publisher));
    tcp_server->log_info("RegisterPublisher(%s, %s)", topic.c_str(), message_type.c_str());
}

void ClientThread::register_ros_service(const std::string& topic, const std::string& request_message_type) {
    auto& old_node = ros_services_table.find(topic);
    if (old_node != ros_services_table.end()) {
        tcp_server->unregister_node(std::reinterpret_pointer_cast<RosNode>(old_node->second));
    }
    std::shared_ptr<RosService> new_service = std::make_shared<RosService>(topic, request_message_type);
    ros_services_table[topic] = new_service;
    tcp_server->register_node(std::reinterpret_pointer_cast<RosNode>(new_service));
    tcp_server->log_info("RegisterRosService(%s, %s)", topic.c_str(), request_message_type.c_str());
}

void ClientThread::register_unity_service(const std::string& topic, const std::string& request_message_type) {
    auto& old_node = unity_services_table.find(topic);
    if (old_node != unity_services_table.end()) {
        tcp_server->unregister_node(std::reinterpret_pointer_cast<RosNode>(old_node->second));
    }
    std::shared_ptr<UnityService> new_service = std::make_shared<UnityService>(&unity_tcp_sender, topic, request_message_type);
    unity_services_table[topic] = new_service;
    tcp_server->register_node(std::reinterpret_pointer_cast<RosNode>(new_service));
    tcp_server->log_info("RegisterUnityService(%s, %s)", topic.c_str(), request_message_type.c_str());
}

void ClientThread::unregister_all() {
    for (auto& [topic, publisher] : publishers_table) {
        tcp_server->unregister_node(publisher);
    }
    publishers_table.clear();

    for (auto& [topic, subscriber] : subscribers_table) {
        tcp_server->unregister_node(subscriber);
    }
    subscribers_table.clear();

    for (auto& [topic, service] : ros_services_table) {
        tcp_server->unregister_node(service);
    }
    ros_services_table.clear();

    for (auto& [topic, service] : unity_services_table) {
        tcp_server->unregister_node(service);
    }
    unity_services_table.clear();
}

void ClientThread::send_topic_list() {
    SysCommand::TopicsResponse topics_response{};
    for (const auto& [topic, types] : tcp_server->get_topic_names_and_types()) {
        size_t count = types.size();
        if (count == 0) {
            // shall not happen
        } else if (count == 1) {
            topics_response.topics.push_back(topic);
            topics_response.types.push_back(types.front());
        } else {
            std::shared_ptr<RosNode> node = get_node_for_topic(topic);
            topics_response.topics.push_back(topic);
            if (node) {
                const std::string message_type = node->get_message_type();
                tcp_server->log_warning(
                    "Only one message type per topic is supported, but found multiple types for topic %s; maintaining %s as the subscribed type.",
                    topic, parse_message_name(message_type));
                topics_response.types.push_back(message_type);
            } else {
                tcp_server->log_warning(
                    "Only one message type per topic is supported, but found multiple types for topic %s; and none is currently subscribed. Using first type %s.",
                    topic, types.front());
                topics_response.types.push_back(types.front());
            }
        }
    }

    // strip the '/msg/' part from types
    for (auto& type : topics_response.types) {
        auto msg_index = type.find("/msg/");
        if (msg_index != std::string::npos) {
            type = type.substr(0, msg_index) + type.substr(msg_index + 4);
        }
    }

    unity_tcp_sender.send_topic_list(topics_response);
}

std::string ClientThread::parse_message_name(const std::string& name) {
    // Example input string: <class 'std_msgs.msg._string.Metaclass_String'>
    std::smatch match;

    if (std::regex_match(name, match, parse_message_regex)) {
        if (match.size() == 3) {
            return match[1].str() + "/" + match[2].str();
        } else {
            tcp_server->log_error("UnityTcpSender::parse_message_name : unexpected match size of %d for %s", match.size(), name);
        }
    } else {
        tcp_server->log_error("UnityTcpSender::parse_message_name : unable to match regex for %s", name);
    }
    return std::string{};
}


bool ClientThread::get_ros_data(const std::string& data, RosData& result) {
    if (0 != data.compare(0, ROS2_HEADER.size(), ROS2_HEADER)) {
        return false;
    }

    size_t ros_data_size = data.size();
    result.resize(ros_data_size);
    std::copy_n(data.begin(), ros_data_size, result.begin());
    return true;
}

void ClientThread::handle_syscommand(const std::string& command, const std::string& data) {
    tcp_server->log_debug("Handling syscommand %s", command.c_str());
    if (SysCommand::k_SysCommand_Subscribe == command) {
        SysCommand::TopicAndType topic_and_type{};
        if (SysCommand::deserialize(data, topic_and_type)) {
            if (topic_and_type.topic.empty()) {
                unity_tcp_sender.send_unity_error("Can't subscribe to a blank topic name! SysCommand.subscribe(" + topic_and_type.message_name + ")");
            } else {
                register_subscriber(topic_and_type.topic, topic_and_type.message_name);
            }
        } else {
            deserialize_error(command, data);
        }
    } else if (SysCommand::k_SysCommand_Publish == command) {
        SysCommand::PublisherRegistration publisher_registration{};
        if (SysCommand::deserialize(data, publisher_registration)) {
            if (publisher_registration.topic.empty()) {
                unity_tcp_sender.send_unity_error("Can't publish to a blank topic name! SysCommand.publish(" + publisher_registration.message_name + ")");
            } else {
                register_publisher(publisher_registration.topic, publisher_registration.message_name, publisher_registration.queue_size, publisher_registration.latch);
            }
        } else {
            deserialize_error(command, data);
        }
    } else if (SysCommand::k_SysCommand_RosService == command) {
        SysCommand::TopicAndType topic_and_type{};
        if (SysCommand::deserialize(data, topic_and_type)) {
            if (topic_and_type.topic.empty()) {
                unity_tcp_sender.send_unity_error("Can't register a blank topic name! SysCommand.ros_service(" + topic_and_type.message_name + ")");
            } else {
                register_ros_service(topic_and_type.topic, topic_and_type.message_name);
            }
        } else {
            deserialize_error(command, data);
        }
    } else if (SysCommand::k_SysCommand_UnityService == command) {
        SysCommand::TopicAndType topic_and_type{};
        if (SysCommand::deserialize(data, topic_and_type)) {
            if (topic_and_type.topic.empty()) {
                unity_tcp_sender.send_unity_error("Can't register a blank topic name! SysCommand.unity_service(" + topic_and_type.message_name + ")");
            } else {
                register_unity_service(topic_and_type.topic, topic_and_type.message_name);
            }
        } else {
            deserialize_error(command, data);
        }
    } else if (SysCommand::k_SysCommand_ServiceResponse == command) {
        SysCommand::Service service{};
        if (SysCommand::deserialize(data, service)) {
            pending_srv_id = service.srv_id;
            pending_srv_is_request = false;
        } else {
            deserialize_error(command, data);
        }
    } else if (SysCommand::k_SysCommand_ServiceRequest == command) {
        SysCommand::Service service{};
        if (SysCommand::deserialize(data, service)) {
            pending_srv_id = service.srv_id;
            pending_srv_is_request = true;
        } else {
            deserialize_error(command, data);
        }
    } else if (SysCommand::k_SysCommand_TopicList == command) {
        send_topic_list();
    } else {
        const std::string error_msg = "Don't understand SysCommand.'" + command + "'";
        tcp_server->log_error(error_msg.c_str());
        unity_tcp_sender.send_unity_error(error_msg);
    }
}

void ClientThread::deserialize_error(const std::string& command, const std::string& data) {
    std::string error_msg = "Error parsing data for SysCommand " + command + " : " + data;
    unity_tcp_sender.send_unity_error(error_msg);
    tcp_server->log_error(error_msg.c_str());
}

std::string ClientThread::get_message_type(const std::string& message_name) {
    auto slash_index = message_name.find_first_of('/');
    if (slash_index != std::string::npos && message_name.find_first_of('/', slash_index + 1) == std::string::npos) {
        return message_name.substr(0, slash_index) + "/msg" + message_name.substr(slash_index);
    }
    unity_tcp_sender.send_unity_error("Error getting message type for " + message_name);
    return message_name;
}
