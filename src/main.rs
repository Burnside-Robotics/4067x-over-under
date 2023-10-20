#![no_std]
#![no_main]
#![allow(unused_must_use)]

extern crate alloc;

use alloc::{format, vec};
use core::time::Duration;

use uom::si::{
    angle::degree,
    f64::Length,
    length::{inch, meter},
};
use vex_rs_lib::{
    coordinates::{Coordinates, Position},
    odometry::OdometrySystem,
    pure_pursuit::{Command, Direction, PurePursuitSystem},
};
use vex_rt::prelude::*;

use crate::{drive_system::DriveSystem, shooter_system::ShooterSystem, wing_system::WingSystem};

mod drive_system;
mod shooter_system;
mod utils;
mod wing_system;

pub trait RobotSystem {
    fn initialize(_value: &'static Mutex<Self>) {}
    fn driver_control_cycle(&mut self, controller: &mut Controller) -> Result<(), ControllerError>;
}

pub struct Bot {
    drive_system: Mutex<DriveSystem>,
    shooter_system: Mutex<ShooterSystem>,
    wing_system: Mutex<WingSystem>,

    controller: Mutex<Controller>,

    odometry_system: Mutex<OdometrySystem>,

    pure_pursuit: Mutex<PurePursuitSystem>,
}

impl Robot for Bot {
    fn new(p: Peripherals) -> Self {
        Bot {
            drive_system: DriveSystem::new(
                p.port14, p.port15, p.port16, p.port11, p.port12, p.port13, p.port_b, p.port_a, p.port_c,
            )
            .into(),
            shooter_system: ShooterSystem::new(p.port18, p.port_h).into(),
            wing_system: WingSystem::new(p.port_f, p.port_g).into(),

            controller: p.master_controller.into(),

            odometry_system: OdometrySystem::new(
                p.port09.into_rotation(false).unwrap(),
                p.port10.into_rotation(false).unwrap(),
                p.port08.into_rotation(false).unwrap(),
                Length::new::<inch>(2.75),
                Length::new::<inch>(3.125),
                Length::new::<inch>(3.125),
                Length::new::<inch>(3.075),
            )
            .into(),

            pure_pursuit: PurePursuitSystem::new(Length::new::<inch>(1.0), Length::new::<inch>(15.0)).into(),
        }
    }

    fn initialize(&'static self, _ctx: Context) {
        println!("initialised");

        Task::spawn(move || {
            let mut pause = Loop::new(Duration::from_millis(10));

            loop {
                let mut odom_system = self.odometry_system.lock();
                odom_system.cycle();

                let odom_position = odom_system.get_position();

                let commands = self.pure_pursuit.lock().cycle(odom_position);

                self.drive_system.lock().run_pure_pursuit_commands(commands);

                select! {
                    _ = pause.select() => continue
                }
            }
        });

        DriveSystem::initialize(&self.drive_system);
        ShooterSystem::initialize(&self.shooter_system);
        WingSystem::initialize(&self.wing_system);
    }

    fn autonomous(&self, ctx: Context) {
        self.pure_pursuit.lock().enabled = true;

        self.shooter_system.lock().shoot_once(&ctx);
        self.pure_pursuit.lock().set_sequence([Command::DriveTo(
            Coordinates::new(Length::new::<meter>(0.45), Length::new::<meter>(0.7)),
            Direction::Forward,
            false,
        )]);
    }

    fn opcontrol(&'static self, ctx: Context) {
        let mut pause = Loop::new(Duration::from_millis(10));

        self.pure_pursuit.lock().enabled = false;
        self.pure_pursuit.lock().clear_sequence();

        loop {
            let mut controller = self.controller.lock();

            self.drive_system.lock().driver_control_cycle(&mut controller);
            self.shooter_system.lock().driver_control_cycle(&mut controller);
            self.wing_system.lock().driver_control_cycle(&mut controller);

            select! {
                _ = ctx.done() => break,
                _ = pause.select() => continue
            }
        }
    }
}

entry!(Bot);
