#pragma once

class StatusEvent {
public:
	StatusEvent();

	bool is_set() const;
	void set();

private:
	bool status;
};