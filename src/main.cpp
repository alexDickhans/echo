#include "main.h"

// BEZIER_MP_ASSET(skills);
BEZIER_MIRRORED_MP_ASSET(test);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
	} else {
		pros::lcd::clear_line(2);
	}
}

[[noreturn]] void update_loop() {
	while (true) {
		auto start_time = pros::millis();

		CommandScheduler::run();

		pros::c::task_delay_until(&start_time, 10);
	}
}


[[noreturn]] void screen_update_loop() {
	std::default_random_engine de;

	std::uniform_int_distribution<size_t> particle_dist(0, 10);

	while (true) {
		auto start_time = pros::millis();

		auto pose = drivetrain->getPose();

		pros::lcd::set_text(2, std::to_string(pose.x() * metre.Convert(inch)) + ", " + std::to_string(pose.y() * metre.Convert(inch)) + ", " + std::to_string(pose.z() * radian.Convert(degree)));

		auto particles = drivetrain->getParticles();
		TELEMETRY.send("[");
		for (size_t i = particle_dist(de); i < particles.size(); i += 10) {
			TELEMETRY.send("[");
			TELEMETRY.send(std::to_string(particles[i].x()));
			TELEMETRY.send(",");
			TELEMETRY.send(std::to_string(particles[i].y()));
			TELEMETRY.send(",");
			TELEMETRY.send(std::to_string(particles[i].z()));
			if (i <= particles.size()-11) TELEMETRY.send("],");
			else TELEMETRY.send("]]\n");
		}

		pros::c::task_delay_until(&start_time, 100);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");  // Prints "Hello PROS User!" to line 1 of the LCD

	subsystemInit();

	pros::Task commandScheduler(update_loop, "Command Scheduler");
	pros::Task screenUpdate(screen_update_loop, "Screen Updater");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	CommandScheduler::schedule(
		new Sequence({
			drivetrain->setNorm(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Matrix3d::Identity() * 0.01, false),
			new Ramsete(drivetrain, 0.0, 3.0, &test_red),
		})
	);
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {

}