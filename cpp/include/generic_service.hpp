#pragma once

#include <rclcpp/rclcpp.hpp>
#include "rclcpp/service.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include <functional>

class GenericService : public rclcpp::ServiceBase {
public:
	using Request = void*;   // Serialized data pointer of request message
	using Response = void*;  // Serialized data pointer of response message

    using SharedRequest = std::shared_ptr<void>;
    using SharedResponse = std::shared_ptr<void>;

//    typedef void (*ServiceCallback)(SharedRequest request, SharedRequest response);

    using ServiceCallback = std::function<bool(SharedRequest, SharedResponse&)>;

	RCLCPP_SMART_PTR_DEFINITIONS(GenericService)

  GenericService(
    std::shared_ptr<rcl_node_t> node_handle,
    const std::string & service_name,
    const rosidl_service_type_support_t* service_type_support,
    const rosidl_typesupport_introspection_cpp::MessageMembers* request_members,
    ServiceCallback service_callback,
    rcl_service_options_t& service_options);

    std::shared_ptr<void> create_request() override;
    std::shared_ptr<rmw_request_id_t> create_request_header() override;
    void handle_request(std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<void> request) override;

private:
    RCLCPP_DISABLE_COPY(GenericService)

    void send_response(rmw_request_id_t& req_id, Response response);

    ServiceCallback service_callback_;

    const rosidl_service_type_support_t* srv_type_support_handle_;
    const rosidl_typesupport_introspection_cpp::MessageMembers* request_members_;
};

typename GenericService::SharedPtr
create_generic_service(
    rclcpp::Node &node,
    const std::string& service_name,
    const rosidl_service_type_support_t* service_type_support,
    const rosidl_typesupport_introspection_cpp::MessageMembers* request_members,
    GenericService::ServiceCallback service_callback,
    const rclcpp::QoS& qos = rclcpp::ServicesQoS(),
    rclcpp::CallbackGroup::SharedPtr group = nullptr);
