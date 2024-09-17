#pragma once

/**
 * @brief Abstract class for subsystem behaviors. Look at the [annotated intake example](../tutorials/intakeExample.md)
 * for a more in depth solution
 */
class Subsystem {
public:
	/**
	 * Period is run every frame by the \refitem CommandScheduler useful for debugging tasks and feedback controllers
	 * that need to run every frame
	 */
	virtual void periodic() = 0;
	virtual ~Subsystem() = default;
};
