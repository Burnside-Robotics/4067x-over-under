#include "main.h"

std::shared_ptr<OdomChassisController> driveTrain = ChassisControllerBuilder()
	.withMotors({-14, -15, -16}, {11, 12, 13})
	.withDimensions(AbstractMotor::gearset::blue, {{4_in, 11.5_in}, imev5BlueTPR})
	.withGains({0.001, 0, 0.0001}, {0.001, 0, 0.0001}, {0.001, 0, 0.0001})
	.withSensors(RotationSensor(9), RotationSensor(10), RotationSensor(8))
	.withOdometry({{2.75_in, 6.25_in, 3.075_in }, 360})
	.buildOdometry();

Controller controller;

void opcontrol() {
	while (true) {
		float left = controller.getAnalog(ControllerAnalog::leftY);
		float right = controller.getAnalog(ControllerAnalog::rightY);

		driveTrain->getModel()->tank(left, right);

		pros::delay(20);
	}
}
