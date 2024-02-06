#include "main.h"

/**
 * A dampener limits the rate at which a value can change. This is useful for adding sensitivity limits to joysticks.
 */
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

Motor catapultMotor(15, false, AbstractMotor::gearset::red, AbstractMotor::encoderUnits::degrees);

ADIButton catapultLimitSwitch('H');

pros::ADIDigitalOut leftWingSolenoid('A');
pros::ADIDigitalOut rightWingSolenoid('F');

pros::ADIDigitalOut elevationSolenoid('H');

MotorGroup leftDriveGroup = {-1, -11, -12};
MotorGroup rightDriveGroup = {10, 20, 19};

std::shared_ptr<OdomChassisController> driveTrain = ChassisControllerBuilder()
	.withMotors(leftDriveGroup, rightDriveGroup)
	.withDimensions(AbstractMotor::gearset::blue, {{4_in, 11.5_in}, imev5BlueTPR})
	.withGains({0.0009, 0, 0}, {0.002, 0, 0}, {0.001, 0, 0})
	.withSensors(RotationSensor(2), RotationSensor(9), RotationSensor(13))
	.withOdometry({{2.75_in, 6.25_in, 3.075_in }, 360}, StateMode::CARTESIAN)
	.buildOdometry();

Controller controller;

void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	sylib::initialize();
	// Set the LEDs on the back of the robot to cycle through a rainbow gradient
	backLights.gradient(0x990000, 0x990003, 45, 0, false, true);
	backLights.cycle(*backLights, 5);


	// Tell motors to apply brake force when commanded to apply zero voltage
	leftDriveGroup.setBrakeMode(AbstractMotor::brakeMode::brake);
	rightDriveGroup.setBrakeMode(AbstractMotor::brakeMode::brake);

	// Declare that the drive train uses Blue gearsets in the motors
	leftDriveGroup.setGearing(AbstractMotor::gearset::blue);
	leftDriveGroup.setGearing(AbstractMotor::gearset::blue);
}

void autonomous() {
	// driveTrain->setState(OdomState{0_m, 0_m, 90_deg});
	// driveTrain->driveToPoint(Point{-2.1_ft, -1.1_ft}, true);
	// driveTrain->driveToPoint(Point{0_m, 0.6_m}, false);
	// driveTrain->driveToPoint(Point{0.15_m, 1.05_m}, true);
	// driveTrain->driveToPoint(Point{0.15_m, 0.5_m}, false);
	driveTrain->setState(OdomState{0_m, 0_m, 180_deg});
	driveTrain->driveToPoint(Point{0_m, 0.5_m}, true);
}

void opcontrol() {
	bool leftWingDeployed = false;
	bool rightWingDeployed = false;

	// Outer rear buttons on controller paddles
	ControllerButton leftWingButton(ControllerDigital::down);
	ControllerButton rightWingButton(ControllerDigital::B);


	Dampener leftInputDampener(0.4);
	Dampener rightInputDampener(0.4);

	// Inner right button on rear controller paddles
	ControllerButton driveDirectionButton(ControllerDigital::Y);

	bool driveReversed = false;
	

	bool elevationDeployed = false;
	ControllerButton elevationToggle(ControllerDigital::R1);

	while (true) {
		/** -------- **/
		/** Catapult **/
		/** -------- **/

		float catapultVoltage;

		if(controller.getDigital(ControllerDigital::L1)) {
			// Drive catapult at full speed
			catapultVoltage = 12000;
		} else if(controller.getDigital(ControllerDigital::L2)) {
			// Drive catapult at reduced speed (suitable for match loading)
			catapultVoltage = 9000;
		} else if (controller.getDigital(ControllerDigital::right)) {
			// Drive catapult backwards (should not be run  for more than a few seconds or it will strip)
			catapultVoltage = -9000;
		} else {
			catapultVoltage = 0;
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

		if(elevationToggle.changedToPressed()) {
			elevationDeployed = !elevationDeployed;
		}

		elevationSolenoid.set_value(elevationDeployed);
		
		/** ----------- **/
		/** Drive Train **/
		/** ----------- **/

		if (driveDirectionButton.changedToPressed()) {
			// Switch the direction of the drive train and rumble the controller
			driveReversed = !driveReversed;
			controller.rumble(".");
		}

		float leftInput = controller.getAnalog(ControllerAnalog::leftY);
		float rightInput = controller.getAnalog(ControllerAnalog::rightY);

		if (driveReversed) {
			// When the drive train is reversed, we swap the inputs, and reverse their polarity
			std::swap(leftInput, rightInput);

			leftInput *= -1;
			rightInput *= -1;
		}

		driveTrain->getModel()->tank(leftInputDampener.cycle(leftInput), rightInputDampener.cycle(rightInput));

		OdomState state = driveTrain->getState();

		// Print the odometry position to the LCD screen
		pros::lcd::print(2, "X: %f", state.x.convert(inch));
		pros::lcd::print(3, "Y: %f", state.y.convert(inch));
		pros::lcd::print(4, "theta: %f", state.theta.convert(degree));

		pros::delay(20);
	}
}
