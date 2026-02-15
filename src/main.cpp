#include "main.h"
#include "localization/mcl_chassis.hpp"
#include "robodash/api.h"

using namespace localization;

// ─────────────────────────────────────────────────────────
// CHASSIS SETUP — Configure your robot here!
// ─────────────────────────────────────────────────────────

MCLChassis chassis({1, 2},   // Left motor ports
                   {-3, -4}, // Right motor ports (negative = reversed)
                   5,        // IMU port
                   4.0,      // Wheel diameter (inches)
                   12.5,     // Track width (inches)
                   1.0,      // Gear ratio (1.0 = direct drive)
                   600       // Motor RPM (600=blue, 200=green, 100=red)
);

// ─────────────────────────────────────────────────────────
// AUTONOMOUS ROUTINES — Add your autons here!
// ─────────────────────────────────────────────────────────

void auton_left() {
  chassis.set_pose(24, 24, 0);
  chassis.drive_to(48, 48);
  chassis.turn_to(90);
  chassis.pose_to(72, 72, 180);
  chassis.stop();
}

void auton_right() {
  chassis.set_pose(120, 24, 180);
  chassis.drive_to(96, 48);
  chassis.turn_to(90);
  chassis.pose_to(72, 72, 0);
  chassis.stop();
}

void auton_skills() {
  chassis.set_pose(24, 24, 0);
  chassis.drive_to(48, 48);
  chassis.drive_to(72, 72);
  chassis.drive_to(96, 96);
  chassis.drive_to(24, 24);
  chassis.stop();
}

void auton_do_nothing() {
  // Safety auton — does nothing
}

// ─────────────────────────────────────────────────────────
// ROBODASH SELECTOR — Pick your auton on the brain screen!
// ─────────────────────────────────────────────────────────

rd::Selector selector({
    {"Left Side", auton_left},
    {"Right Side", auton_right},
    {"Skills", auton_skills},
    {"Do Nothing", auton_do_nothing},
});

// ─────────────────────────────────────────────────────────
// INITIALIZE
// ─────────────────────────────────────────────────────────

void initialize() {
  // Add distance sensors: (port, forward", left", facing°)
  chassis.add_sensor(6, 5, 0, 0);    // Front sensor
  chassis.add_sensor(7, 0, 5, 90);   // Left sensor
  chassis.add_sensor(8, 0, -5, -90); // Right sensor
}

void disabled() {}
void competition_initialize() {}

// ─────────────────────────────────────────────────────────
// AUTONOMOUS — Runs whichever auton you selected!
// ─────────────────────────────────────────────────────────

void autonomous() { selector.run_auton(); }

// ─────────────────────────────────────────────────────────
// OPCONTROL — Arcade drive + telemetry
// ─────────────────────────────────────────────────────────

void opcontrol() {
  pros::Controller master(pros::E_CONTROLLER_MASTER);

  while (true) {
    // Arcade drive
    int dir = master.get_analog(ANALOG_LEFT_Y);
    int turn = master.get_analog(ANALOG_RIGHT_X);
    // Note: for opcontrol driving, use the motor groups directly
    // The chassis object handles localization in the background
    pros::delay(20);
  }
}