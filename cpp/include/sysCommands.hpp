#pragma once
#include <string>
#include <vector>

class SysCommand {
public:
    static const std::string k_SysCommand_Handshake;
    static const std::string k_SysCommand_Log;
    static const std::string k_SysCommand_Warning;
    static const std::string k_SysCommand_Error;
    static const std::string k_SysCommand_ServiceRequest;
    static const std::string k_SysCommand_ServiceResponse;
    static const std::string k_SysCommand_Subscribe;
    static const std::string k_SysCommand_Publish;
    static const std::string k_SysCommand_RosService;
    static const std::string k_SysCommand_UnityService;
    static const std::string k_SysCommand_TopicList;
    static const std::string k_SysCommand_RemoveSubscriber;
    static const std::string k_SysCommand_RemovePublisher;
    static const std::string k_SysCommand_RemoveRosService;
    static const std::string k_SysCommand_RemoveUnityService;

    struct TopicAndType {
        std::string topic;
        std::string message_name;
    };

    struct PublisherRegistration {
        std::string topic;
        std::string message_name;
        int queue_size;
        bool latch;
    };

    struct Service {
        int srv_id;
    };

    struct TopicsResponse {
        std::vector<std::string> topics;
        std::vector<std::string> types;
    };

    static std::string serialize_Log(const std::string& text);
    static std::string serialize_Service(int srv_id);
    static std::string serialize_TopicsResponse(const TopicsResponse &topics_response);
    static std::string serialize_Handshake();

    static bool deserialize(const std::string& data, TopicAndType& result);
    static bool deserialize(const std::string& data, PublisherRegistration& result);
    static bool deserialize(const std::string& data, Service& result);
};
