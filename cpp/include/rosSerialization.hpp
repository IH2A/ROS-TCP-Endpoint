#pragma once
#include <rclcpp/rclcpp.hpp>
#include "rosCommunication.hpp"

class RosSerialization {
public :
	static std::shared_ptr<uint8_t[]> deserialize_message(
		const RosData& message, 
		const rosidl_typesupport_introspection_cpp::MessageMembers* message_members, 
		const rosidl_message_type_support_t* message_ts);

	static bool serialize_message(
		const void* data, RosData& message, 
		const rosidl_typesupport_introspection_cpp::MessageMembers* message_members, 
		const rosidl_message_type_support_t* message_ts);
};