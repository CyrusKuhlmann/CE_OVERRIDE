#include "subsystems/brainDisplay.h"

#include "config.h"
#include "subsystems/subsystems.h"

#include "pros/llemu.hpp"

void BrainDisplay::periodic() {
    const auto pose = localizationSubsystem->getPose();
    const double headingDeg = CONFIG::radToDeg(CONFIG::wrapPi(pose.z()));
    pros::lcd::print(0, "x: %.2f y: %.2f h: %.2f deg", pose.x(), pose.y(), headingDeg);
}
