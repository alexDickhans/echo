#pragma once

#include "pros/serial.hpp"
#include <iostream>

class Telemetry {
private:
	pros::Serial* serial = nullptr;
public:
	Telemetry() = default;

	void setSerial(pros::Serial* serial);

	void send(const std::string& string);
};

