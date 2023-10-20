use colorsys::Hsl;
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

    led_strip_left: AddressableLed<56>,
    led_strip_right: AddressableLed<56>,
    led_strip_back: AddressableLed<45>,

    i: u128,
}

impl DriveSystem {
    pub fn new(
        port_l1: SmartPort,
        port_l2: SmartPort,
        port_l3: SmartPort,
        port_r1: SmartPort,
        port_r2: SmartPort,
        port_r3: SmartPort,
        led_port_left: AdiPort,
        led_port_right: AdiPort,
        led_port_back: AdiPort,
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

            led_strip_right: led_port_right.into_adressable_led().unwrap(),
            led_strip_left: led_port_left.into_adressable_led().unwrap(),
            led_strip_back: led_port_back.into_adressable_led().unwrap(),

            i: 0,
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
        let red = Hsl::new(0.0, 100.0, 30.0, None);
        let white = Hsl::new(0.0, 0.0, 60.0, None);
        let red_end = Hsl::new(360.0, 100.0, 30.0, None);

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

        // // Only one LED strip can be set per cycle... Dunno why
        if self.i == 0 {
            self.led_strip_right.set_gradient(&red, &red_end, 8..48, false);
        } else if self.i == 1 {
            self.led_strip_left.set_gradient(&red, &red_end, 8..48, false);
        } else if self.i == 2 {
            self.led_strip_back.set_gradient(&red, &red_end, 0..45, false);
        } else if self.i % 3 == 0 {
            self.led_strip_left.rotate(1, 8..48);
        } else if self.i % 4 == 0 {
            self.led_strip_right.rotate(1, 8..48);
        } else {
            self.led_strip_back.rotate(1, 0..45);
        }

        self.i += 1;

        // // Reverse Driver inputs if neccesary
        if self.reversed_drive_state {
            (left_input, right_input) = (-right_input, -left_input);
            self.led_strip_left.set_range(&white, 0..8);
            self.led_strip_left.set_range(&red, 48..56);

            self.led_strip_right.set_range(&white, 0..8);
            self.led_strip_right.set_range(&red, 48..56);
        } else {
            self.led_strip_left.set_range(&red, 0..8);
            self.led_strip_left.set_range(&white, 48..56);

            self.led_strip_right.set_range(&red, 0..8);
            self.led_strip_right.set_range(&white, 48..56);
        }

        self.drive_train.drive_tank(
            self.left_dampener.cycle(left_input),
            self.right_dampener.cycle(right_input),
        );

        Ok(())
    }
}
