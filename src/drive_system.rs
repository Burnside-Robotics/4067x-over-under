use uom::si::{
    angle::{degree, radian},
    angular_velocity::{revolution_per_minute, AngularVelocity},
    f64::{Angle, Length, Ratio},
    length::inch,
    ratio::percent,
};
use vex_rs_lib::{gains, pure_pursuit::PurePursuitCommands, ratio, tank_drive::TankDrive};
use vex_rt::prelude::*;

use crate::{
    utils::{Dampener, Debouncer},
    RobotSystem,
};

// The drive ratio is the gearing ratio from the motors to the wheels.
// Our drive ratio is 36:60, making the wheels turn at 0.6 times the speed of the motors
// Since our drive train motors are 600rpm, this makes the wheels spin at 360rpm

pub struct DriveSystem {
    pub drive_train: TankDrive<3>,

    reversed_drive_state: bool,

    left_dampener: Dampener<Ratio>,

    right_dampener: Dampener<Ratio>,

    debouncer: Debouncer,
}

impl DriveSystem {
    pub fn new(
        port_l1: SmartPort,
        port_l2: SmartPort,
        port_l3: SmartPort,
        port_r1: SmartPort,
        port_r2: SmartPort,
        port_r3: SmartPort,
    ) -> Self {
        let mut left_motors = [
            port_l1.into_motor(Gearset::SixToOne, EncoderUnits::Degrees, true),
            port_l2.into_motor(Gearset::SixToOne, EncoderUnits::Degrees, true),
            port_l3.into_motor(Gearset::SixToOne, EncoderUnits::Degrees, true),
        ];
        let mut right_motors = [
            port_r1.into_motor(Gearset::SixToOne, EncoderUnits::Degrees, false),
            port_r2.into_motor(Gearset::SixToOne, EncoderUnits::Degrees, false),
            port_r3.into_motor(Gearset::SixToOne, EncoderUnits::Degrees, false),
        ];

        for motor in left_motors.iter_mut() {
            motor.set_brake_mode(BrakeMode::Brake);
        }

        for motor in right_motors.iter_mut() {
            motor.set_brake_mode(BrakeMode::Brake);
        }

        let wheel_diameter = Length::new::<inch>(3.25);

        Self {
            drive_train: TankDrive {
                left_motors,
                right_motors,
                drive_ratio: ratio!(0.6),
                wheel_diameter,
                track_width: Length::new::<inch>(12.5),
                distance_gains: gains!(0.05, 15.7e-4, 0.0),
                turn_gains: gains!(0.05, 8e-5, 0.0),
                left_velocity_gains: gains!(0.05, 8e-5, 0.0),
                right_velocity_gains: gains!(0.05, 8e-5, 0.0),
                position_threshold: Angle::new::<degree>(5.0),
                velocity_threshold: AngularVelocity::new::<revolution_per_minute>(5.0),
            },

            reversed_drive_state: false,
            left_dampener: Dampener::new(ratio!(0.4)),
            right_dampener: Dampener::new(ratio!(0.4)),

            debouncer: Debouncer::new(),
        }
    }

    pub fn run_pure_pursuit_commands(&mut self, commands: Option<PurePursuitCommands>) {
        if let Some(PurePursuitCommands(length, angle)) = commands {
            let max_speed: Ratio = Ratio::new::<percent>(50.0);

            let linear_vel: Ratio = (ratio!(0.05) * length.get::<inch>()).min(max_speed).max(-max_speed);
            let turn_vel: Ratio = (ratio!(1.0) * angle.get::<radian>()).min(max_speed).max(-max_speed);

            self.drive_train
                .drive_tank(linear_vel + turn_vel, linear_vel - turn_vel);
        }
    }
}

impl RobotSystem for DriveSystem {
    fn driver_control_cycle(&mut self, controller: &mut Controller) -> Result<(), ControllerError> {
        // Our drive train can be reversed by pressing controller rear paddle levers
        // This is to make it easy to drive when intaking, and when aiming to shoot by flipping the drive train to suit
        // whichever current side of the robot is used for reference

        // Use outside paddle levers to switch drive direction
        if self.debouncer.test(controller.y.is_pressed()?) {
            self.reversed_drive_state = !self.reversed_drive_state;
            controller.screen.rumble(".");
        }

        let mut left_input: Ratio = controller.left_stick.get_y()?;
        let mut right_input: Ratio = controller.right_stick.get_y()?;

        if self.reversed_drive_state {
            (left_input, right_input) = (-right_input, -left_input);
        }

        self.drive_train.drive_tank(
            self.left_dampener.cycle(left_input),
            self.right_dampener.cycle(right_input),
        );

        Ok(())
    }
}
