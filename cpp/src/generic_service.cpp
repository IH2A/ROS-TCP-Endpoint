#include "generic_service.hpp"
#include "rclcpp/rclcpp.hpp"


typename GenericService::SharedPtr create_generic_service(
    rclcpp::Node& node,
    const std::string& service_name,
    const rosidl_service_type_support_t* service_type_support,
    const rosidl_typesupport_introspection_cpp::MessageMembers* request_members,
    GenericService::ServiceCallback service_callback,
    const rclcpp::QoS& qos,
    rclcpp::CallbackGroup::SharedPtr group) {

    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base = node.get_node_base_interface();
    std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services = node.get_node_services_interface();
    std::string extended_service_name = rclcpp::extend_name_with_sub_namespace(service_name, node.get_sub_namespace());
    const rmw_qos_profile_t& qos_profile = qos.get_rmw_qos_profile();

    rcl_service_options_t service_options = rcl_service_get_default_options();
    service_options.qos = qos_profile;

    auto serv = GenericService::make_shared(
        node_base->get_shared_rcl_node_handle(),
        service_name,
        service_type_support,
        request_members,
        service_callback,
        service_options);
    auto serv_base_ptr = std::dynamic_pointer_cast<rclcpp::ServiceBase>(serv);
    node_services->add_service(serv_base_ptr, group);
    return serv;
}

GenericService::GenericService(
    std::shared_ptr<rcl_node_t> node_handle,
    const std::string& service_name,
    const rosidl_service_type_support_t* service_type_support,
    const rosidl_typesupport_introspection_cpp::MessageMembers* request_members,
    ServiceCallback service_callback,
    rcl_service_options_t& service_options)
    : ServiceBase(node_handle), service_callback_(service_callback), srv_type_support_handle_(service_type_support), request_members_(request_members) {
    // rcl does the static memory allocation here
    service_handle_ = std::shared_ptr<rcl_service_t>(
        new rcl_service_t, [handle = node_handle_, service_name](rcl_service_t* service) {
            if (rcl_service_fini(service, handle.get()) != RCL_RET_OK) {
                RCLCPP_ERROR(
                    rclcpp::get_node_logger(handle.get()).get_child("rclcpp"),
                    "Error in destruction of rcl service handle: %s",
                    rcl_get_error_string().str);
                rcl_reset_error();
            }
            delete service;
        });
    *service_handle_.get() = rcl_get_zero_initialized_service();

    rcl_ret_t ret = rcl_service_init(
        service_handle_.get(),
        node_handle.get(),
        srv_type_support_handle_,
        service_name.c_str(),
        &service_options);
    if (ret != RCL_RET_OK) {
        if (ret == RCL_RET_SERVICE_NAME_INVALID) {
            auto rcl_node_handle = get_rcl_node_handle();
            // this will throw on any validation problem
            rcl_reset_error();
            rclcpp::expand_topic_or_service_name(
                service_name,
                rcl_node_get_name(rcl_node_handle),
                rcl_node_get_namespace(rcl_node_handle),
                true);
        }

        rclcpp::exceptions::throw_from_rcl_error(ret, "could not create service");
    }
}

std::shared_ptr<void> GenericService::create_request() {
    void* request = new uint8_t[request_members_->size_of_];
    request_members_->init_function(request, rosidl_runtime_cpp::MessageInitialization::ZERO);
    return std::shared_ptr<void>(
        request,
        [this](void* p) {
            request_members_->fini_function(p);
            delete[] reinterpret_cast<uint8_t*>(p);
        });
}

std::shared_ptr<rmw_request_id_t> GenericService::create_request_header() {
    return std::make_shared<rmw_request_id_t>();
}

void GenericService::handle_request(std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<void> request) {
    SharedResponse response;
    if (service_callback_(request, response)) {
        send_response(*request_header, response.get());
    }
}

void GenericService::send_response(rmw_request_id_t& req_id, Response response) {
    rcl_ret_t ret = rcl_send_response(get_service_handle().get(), &req_id, response);

    if (ret == RCL_RET_TIMEOUT) {
        RCLCPP_WARN(
            node_logger_.get_child("rclcpp"),
            "failed to send response to %s (timeout): %s",
            this->get_service_name(), rcl_get_error_string().str);
        rcl_reset_error();
        return;
    }
    if (ret != RCL_RET_OK) {
        rclcpp::exceptions::throw_from_rcl_error(ret, "failed to send response");
    }
}
