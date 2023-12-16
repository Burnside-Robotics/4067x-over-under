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

sylib::Addrled backLights = sylib::Addrled(22, 3, 45);

Motor catapultMotor(18, false, AbstractMotor::gearset::red, AbstractMotor::encoderUnits::degrees);

ADIButton catapultLimitSwitch('H');

pros::ADIDigitalOut leftWingSolenoid('F');
pros::ADIDigitalOut rightWingSolenoid('G');

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

	sylib::initialize();
	backLights.gradient(0x990000, 0x990003, 45, 0, false, true);
	backLights.cycle(*backLights, 5);


	leftDriveGroup.setBrakeMode(AbstractMotor::brakeMode::brake);
	rightDriveGroup.setBrakeMode(AbstractMotor::brakeMode::brake);

	leftDriveGroup.setGearing(AbstractMotor::gearset::blue);
	leftDriveGroup.setGearing(AbstractMotor::gearset::blue);
}

void opcontrol() {
	ControllerButton catapultManualButton(ControllerDigital::right);

	bool catapultManual = false;


	bool leftWingDeployed = false;
	bool rightWingDeployed = false;

	ControllerButton leftWingButton(ControllerDigital::down);
	ControllerButton rightWingButton(ControllerDigital::B);


	Dampener leftInputDampener(0.4);
	Dampener rightInputDampener(0.4);

	ControllerButton driveDirectionButton(ControllerDigital::Y);

	bool driveReversed = false;

	while (true) {
		/** -------- **/
		/** Catapult **/
		/** -------- **/

		if(catapultManualButton.changedToPressed()) {
			catapultManual = !catapultManual;
		}

		float catapultVoltage;

		if (catapultManual) {
			if(controller.getDigital(ControllerDigital::L1)) {
				catapultVoltage = 9000;
			} else if(controller.getDigital(ControllerDigital::L2)) {
				catapultVoltage = -9000;
			} else {
				catapultVoltage = 0;
			}
		} else {
			if(catapultLimitSwitch.isPressed() && !controller.getDigital(ControllerDigital::L1)) {
				catapultVoltage = 1800;
			} else {
				catapultVoltage = 9000;
			}
		}

		catapultMotor.moveVoltage(catapultVoltage);

		/** ----- **/
		/** Wings **/
		/** ----- **/

		if(leftWingButton.changedToPressed()) {
			leftWingDeployed = !leftWingDeployed;
		}

		if(rightWingButton.changedToPressed()) {
			rightWingDeployed = !rightWingDeployed;
		}

		leftWingSolenoid.set_value(leftWingDeployed);
		rightWingSolenoid.set_value(rightWingDeployed);

		/** ----------- **/
		/** Drive Train **/
		/** ----------- **/

		if (driveDirectionButton.changedToPressed()) {
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
