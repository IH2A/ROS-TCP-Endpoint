#include "rosService.hpp"
#include "tcpServerNode.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosSerialization.hpp"


// Class to send messages to a ROS service.
RosService::RosService(const std::string& service_topic, const std::string& service_type)
	: RosSender(service_topic + "_RosService", service_topic) {

    // get typesupport informations for request and response types
    ts_lib = rclcpp::get_typesupport_library(service_type, "rosidl_typesupport_cpp");
    service_ts = rclcpp::get_service_typesupport_handle(service_type, "rosidl_typesupport_cpp", *ts_lib);
    const auto service_request_ts_introspection = get_message_typesupport_handle(service_ts->request_typesupport, rosidl_typesupport_introspection_cpp::typesupport_identifier);
    service_request_members = reinterpret_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(service_request_ts_introspection->data);
    const auto service_response_ts_introspection = get_message_typesupport_handle(service_ts->response_typesupport, rosidl_typesupport_introspection_cpp::typesupport_identifier);
    service_response_members = reinterpret_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(service_response_ts_introspection->data);

    client = create_generic_client(service_topic, service_type);
}

/*
    Takes in serialized message data from source outside of the ROS network,
    deserializes it into it's class, calls the service with the message, and returns
    the service's response.

    Args:
        data: The already serialized message_class data coming from outside of ROS

    Returns:
        service response
*/
bool RosService::send(const RosData &request, RosData &response) {
    // deserialize message and send it as service request
    auto data = RosSerialization::deserialize_message(request, service_request_members, service_ts->request_typesupport);
    if (!data) {
        return false;
    }
    rclcpp::GenericClient::Request ros_request = data.get();
    auto future = client->async_send_request(ros_request);

    // wait for result and build RosData
    auto future_result = future.get();  //$$ TODO : handle timeout ?
    /*
        while (rclcpp::ok()) {
            if (future.wait)
        }
        if (timeout) {
            client->remove_pending_request(future);
        }
        */

    if (!RosSerialization::serialize_message(future_result.get(), response, service_response_members, service_ts->response_typesupport))
        return false;

	return true;
}
