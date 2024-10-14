#include "unityService.hpp"
#include "tcpServerNode.hpp"
#include "rosSerialization.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include <functional>

UnityService::UnityService(UnityTcpSender* unity_tcp_sender, const std::string& service_topic, const std::string &service_type, int queue_size)
	: RosReceiver(unity_tcp_sender, service_topic + "_service", service_topic) {

    // get typesupport informations for request and response types
    ts_lib = rclcpp::get_typesupport_library(service_type, "rosidl_typesupport_cpp");
    service_ts = rclcpp::get_service_typesupport_handle(service_type, "rosidl_typesupport_cpp", *ts_lib);
    const auto service_request_ts_introspection = get_message_typesupport_handle(service_ts->request_typesupport, rosidl_typesupport_introspection_cpp::typesupport_identifier);
    service_request_members = reinterpret_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(service_request_ts_introspection->data);
    const auto service_response_ts_introspection = get_message_typesupport_handle(service_ts->response_typesupport, rosidl_typesupport_introspection_cpp::typesupport_identifier);
    service_response_members = reinterpret_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(service_response_ts_introspection->data);

    service = create_generic_service(
        *this, 
        service_topic, 
        service_ts, 
        service_request_members,
        [this] (GenericService::SharedRequest request, GenericService::SharedResponse &response) { return handle_request(request, response); }
    );
}

bool UnityService::handle_request(GenericService::SharedRequest request, GenericService::SharedResponse &response) {
    RosData request_data;

    // deserialize message and send it as service request
    if (RosSerialization::serialize_message(request.get(), request_data, service_request_members, service_ts->request_typesupport)) {
        RosData response_data = unity_tcp_sender->send_unity_service_request(topic, request_data);

        auto data = RosSerialization::deserialize_message(response_data, service_response_members, service_ts->response_typesupport);
        if (data) {
            response = std::reinterpret_pointer_cast<GenericService::Response>(data);
            return true;
        }
    }

    return false;
}
