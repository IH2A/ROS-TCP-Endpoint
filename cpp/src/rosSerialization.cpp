#include "rosSerialization.hpp"

std::shared_ptr<uint8_t[]> RosSerialization::deserialize_message(const RosData& message, const rosidl_typesupport_introspection_cpp::MessageMembers* message_members, const rosidl_message_type_support_t* message_ts) {
    rclcpp::SerializedMessage serializedMessage{ message.size() };
    auto& rcl_serializedMessage = serializedMessage.get_rcl_serialized_message();
    rcl_serializedMessage.buffer_length = message.size();
    std::copy_n(message.cbegin(), rcl_serializedMessage.buffer_length, rcl_serializedMessage.buffer);

    auto result = std::shared_ptr<uint8_t[]>(
        new uint8_t[message_members->size_of_],
        [fini_function = message_members->fini_function](uint8_t* msg) {
            fini_function(msg);
            delete[] msg;
        });

    message_members->init_function(result.get(), rosidl_runtime_cpp::MessageInitialization::ZERO);

    rmw_ret_t ret = rmw_deserialize(&rcl_serializedMessage, message_ts, result.get());
    if (ret != RMW_RET_OK) {  // Failed to deserialize service event message
        result.reset();
        std::cerr << "deserialize_message failed !" << std::endl;
    }
    return result;
}

bool RosSerialization::serialize_message(const void* data, RosData& message, const rosidl_typesupport_introspection_cpp::MessageMembers* message_members, const rosidl_message_type_support_t* message_ts) {
    message.clear();

    rclcpp::SerializedMessage serializedMessage{ message_members->size_of_ };
    auto& rcl_serializedMessage = serializedMessage.get_rcl_serialized_message();

    rmw_ret_t ret = rmw_serialize(data, message_ts, &rcl_serializedMessage);
    if (ret != RMW_RET_OK) {  // Failed to deserialize service event message
        std::cerr << "serialize_message failed !" << std::endl;
        return false;
    }

    message.resize(rcl_serializedMessage.buffer_length);
    std::copy_n(rcl_serializedMessage.buffer, rcl_serializedMessage.buffer_length, message.begin());
    return true;
}

