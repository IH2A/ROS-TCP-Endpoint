#pragma once
#include "rosCommunication.hpp"
#include "rclcpp/generic_client.hpp"
#include <memory>

class RosService : public RosSender {
public:
	RosService(const std::string& service_topic, const std::string& service_type);

	bool send(const RosData& request, RosData& response);

private:
	rclcpp::GenericClient::SharedPtr client;

	std::shared_ptr<rcpputils::SharedLibrary> ts_lib;
	const rosidl_service_type_support_t* service_ts;
	const rosidl_typesupport_introspection_cpp::MessageMembers* service_request_members;
	const rosidl_typesupport_introspection_cpp::MessageMembers* service_response_members;
};
