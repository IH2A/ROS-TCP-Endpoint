#pragma once
#include <rclcpp/rclcpp.hpp>
#include "rosPublisher.hpp"
#include "rosSubscriber.hpp"
#include "rosService.hpp"
#include "unityService.hpp"
#include "clientThread.hpp"
#include <vector>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <cstdarg>
#include <WinSock2.h>

class TcpServerNode : public rclcpp::Node {
public:
    TcpServerNode(const std::string& name);

    bool init(int connections = 10, const std::string& tcp_ip = "", int tcp_port = 0);
    void shutdown();

    void start();
    void spin_executor();
    void register_node(std::shared_ptr<RosNode> node);
    void unregister_node(std::shared_ptr<RosNode> old_node);
    
    void log_debug(const char* msg, ...);
    void log_info(const char* msg, ...);
    void log_warning(const char* msg, ...);
    void log_error(const char* msg, ...);
    
private:
    static const std::string rosIPParameter;
    static const std::string rosPortParameter;

    void listen_loop();

    bool get_log_string(const char* format, std::va_list args);

    std::string tcp_ip{};
    int tcp_port{0};
    int connections{0};
    SOCKET tcp_server{INVALID_SOCKET};

    bool stop_server_thread{ false };
    std::thread server_thread{};
    std::set<std::shared_ptr<ClientThread>> client_threads;
    std::mutex client_threads_mutex;
    std::mutex log_mutex;
    char log_buffer[256]{};
    rclcpp::executors::MultiThreadedExecutor::UniquePtr executor;
};
