#include "main.h"

Command* autonCommand;

[[noreturn]] void update_loop() {
    while (true) {
        auto start_time = pros::millis();

        CommandScheduler::run();

        pros::c::task_delay_until(&start_time, 10);
    }
}


[[noreturn]] void screen_update_loop() {

    lv_theme_t *th = lv_theme_default_init(lv_disp_get_default(),  /*Use the DPI, size, etc from this display*/
                                           lv_color_hex(0xff7d26),
                                           lv_color_hex(0x303236),   /*Primary and secondary palette*/
                                           false,    /*Light or dark mode*/
                                           &lv_font_montserrat_14); /*Small, normal, large fonts*/

    lv_disp_set_theme(lv_disp_get_default(), th); /*Assign the theme to the display*/
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0xffffff), LV_PART_ANY | LV_STATE_ANY);

    LV_IMG_DECLARE(echo_logo);

    lv_obj_t * logo = lv_img_create(lv_scr_act());
    lv_img_set_src(logo, &echo_logo);
    lv_obj_align(logo, LV_ALIGN_CENTER, 0, 0);

    std::default_random_engine de;

    std::uniform_int_distribution<size_t> particle_dist(0, 49);

    while (true) {
        auto start_time = pros::millis();

        auto pose = drivetrain->getPose();

        // pros::lcd::set_text(1, std::to_string(pose.x() * metre.Convert(inch)) + ", " +
        //                                std::to_string(pose.y() * metre.Convert(inch)) + ", " +
        //                                std::to_string(pose.z() * radian.Convert(degree)));

        switch (ALLIANCE) {
            case RED:
                partner.set_text(1, 1, "RED");
                break;
            case BLUE:
                partner.set_text(1, 1, "BLUE");
                break;
        }

        // TELEMETRY.send("[[" + std::to_string(pose.x()) + "," + std::to_string(pose.y()) + "," +
        //                std::to_string(pose.z()) + "]]\n");
        pros::c::task_delay_until(&start_time, 20);
        // TELEMETRY.send("{\"time\": " + std::to_string(pros::millis()/1000.0) + ", \"data\":[");
        // TELEMETRY.send("[");
        // TELEMETRY.send(std::to_string(pose.x()));
        // TELEMETRY.send(",");
        // TELEMETRY.send(std::to_string(pose.y()));
        // TELEMETRY.send(",");
        // TELEMETRY.send(std::to_string(pose.z()));
        // TELEMETRY.send("],");
        // for (size_t i = particle_dist(de); i < CONFIG::NUM_PARTICLES; i += 50) {
        //     auto particle = drivetrain->getParticle(i);
        //     TELEMETRY.send("[");
        //     TELEMETRY.send(std::to_string(particle.x()));
        //     TELEMETRY.send(",");
        //     TELEMETRY.send(std::to_string(particle.y()));
        //     TELEMETRY.send(",");
        //     TELEMETRY.send(std::to_string(particle.z()));
        //     if (i <= CONFIG::NUM_PARTICLES - 52)
        //         TELEMETRY.send("],");
        //     else
        //         TELEMETRY.send("]]}\n");
        // }

    }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    subsystemInit();

    pros::Task commandScheduler(update_loop, "Command Scheduler");
    // pros::Task screenUpdate(screen_update_loop, "Screen Updater");

    autonCommand = AutonomousCommands::getAuton();
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
    CommandScheduler::schedule(autonCommand);
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
    if (AUTON == Auton::SKILLS) {
        CommandScheduler::schedule(autonCommand->until([&]() {return primary.get_digital(DIGITAL_A);}));
    }
}
