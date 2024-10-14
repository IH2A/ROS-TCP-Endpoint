#pragma once
#include <string>
#include <mutex>
#include <condition_variable>
#include "rosCommunication.hpp"

class ThreadPauser {
public:
	ThreadPauser();

	RosData sleep_until_resumed();
	void resume_with_result(const RosData &result);

private:
	bool has_result;
	RosData result;
	std::mutex m;
	std::condition_variable cv;
};