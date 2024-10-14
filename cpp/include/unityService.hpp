#pragma once
#include "rosCommunication.hpp"
#include "generic_service.hpp"
#include <memory>

class UnityService : public RosReceiver {
public:
	UnityService(UnityTcpSender* unity_tcp_sender, const std::string& service_topic, const std::string & service_type, int queue_size = 10);

	bool handle_request(GenericService::SharedRequest request, GenericService::SharedResponse &response);

private:
	GenericService::SharedPtr service;

	std::shared_ptr<rcpputils::SharedLibrary> ts_lib;
	const rosidl_service_type_support_t* service_ts;
	const rosidl_typesupport_introspection_cpp::MessageMembers* service_request_members;
	const rosidl_typesupport_introspection_cpp::MessageMembers* service_response_members;
};
