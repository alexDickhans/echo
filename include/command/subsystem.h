#pragma once

class Subsystem {
public:
	virtual void periodic() = 0;
	virtual ~Subsystem() = default;
};