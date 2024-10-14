#include "statusEvent.hpp"

StatusEvent::StatusEvent() : status{false} { }

bool StatusEvent::is_set() const {
	return status;
}

void StatusEvent::set() {
	status = true;
}
