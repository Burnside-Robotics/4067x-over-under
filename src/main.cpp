#include "main.h"

class Dampener {
	private:
		float currentValue = 0;
		float accelerationLimit;

	public:
		Dampener(float accelerationLimit) {
			this->accelerationLimit = accelerationLimit;
		}

		float cycle(float target) {
			if (target > currentValue) {
				if (target - currentValue < accelerationLimit) {
					currentValue = target;
				} else {
					currentValue += accelerationLimit;
				}
			} else if (target < currentValue) {
				if (currentValue - target < accelerationLimit ){
					currentValue = target;
				} else {
					currentValue -= accelerationLimit;
				}
			}
			return currentValue;
		}
};

class Debouncer {
	private:
		bool value = false;

	public:
		bool test(bool target) {
			bool previousValue = this->value;
			this->value = target;
			return !previousValue && target;
		}
};

MotorGroup leftDriveGroup = {-14, -15, -16};
MotorGroup rightDriveGroup = {11, 12, 13};

std::shared_ptr<OdomChassisController> driveTrain = ChassisControllerBuilder()
	.withMotors(leftDriveGroup, rightDriveGroup)
	.withDimensions(AbstractMotor::gearset::blue, {{4_in, 11.5_in}, imev5BlueTPR})
	.withGains({0.0007, 0, 0}, {0.001, 0, 0}, {0.001, 0, 0})
	.withSensors(RotationSensor(9), RotationSensor(10), RotationSensor(8))
	.withOdometry({{2.75_in, 6.25_in, 3.075_in }, 360}, StateMode::CARTESIAN)
	.buildOdometry();

Controller controller;

void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	leftDriveGroup.setBrakeMode(AbstractMotor::brakeMode::brake);
	rightDriveGroup.setBrakeMode(AbstractMotor::brakeMode::brake);
}

void opcontrol() {
	Dampener leftInputDampener(0.4);
	Dampener rightInputDampener(0.4);

	Debouncer driveDirectionDebouncer;

	bool driveReversed = false;

	while (true) {
		if (driveDirectionDebouncer.test(controller.getDigital(ControllerDigital::Y))) {
			driveReversed = !driveReversed;
			controller.rumble(".");
		}

		float leftInput = controller.getAnalog(ControllerAnalog::leftY);
		float rightInput = controller.getAnalog(ControllerAnalog::rightY);

		if (driveReversed) {
			std::swap(leftInput, rightInput);
			leftInput *= -1;
			rightInput *= -1;
		}

		driveTrain->getModel()->tank(leftInputDampener.cycle(leftInput), rightInputDampener.cycle(rightInput));

		OdomState state = driveTrain->getState();

		pros::lcd::print(2, "X: %f", state.x.convert(inch));
		pros::lcd::print(3, "Y: %f", state.y.convert(inch));
		pros::lcd::print(4, "theta: %f", state.theta.convert(degree));

		pros::delay(20);
	}
}
