#include "tcpServerNode.hpp"

using namespace std::chrono_literals;

int main(int argc, char** argv) {
	//Ros Initialisation
	rclcpp::init(argc, argv);

	std::shared_ptr<TcpServerNode> tcpServerNode = std::make_shared<TcpServerNode>("UnityEndpoint");
	if (!tcpServerNode->init())
		return -1;

	tcpServerNode->start();
	tcpServerNode->spin_executor();

	tcpServerNode->shutdown();
    rclcpp::shutdown();

    return 0;
}

