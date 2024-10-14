#include "threadPauser.hpp"

ThreadPauser::ThreadPauser() : has_result(false) {}

RosData ThreadPauser::sleep_until_resumed() {
	std::unique_lock ul(m);
	cv.wait(ul, [this] { return has_result; });
	has_result = false;
	return std::move(result);
}

void ThreadPauser::resume_with_result(const RosData& result) {
	std::unique_lock ul(m);
	this->result = result;
	has_result = true;
	ul.unlock();
	cv.notify_one();
}
