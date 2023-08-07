#![no_std]
#![no_main]
#![allow(unused_must_use)]

use core::time::Duration;

use vex_rt::prelude::*;

use crate::{drive_system::DriveSystem, shooter_system::ShooterSystem, wing_system::WingSystem};

mod drive_system;
mod shooter_system;
mod utils;
mod wing_system;

pub trait DriverControlHandler {
    fn driver_control_initialize(&mut self) {}
    fn driver_control_cycle(&mut self, controller: &mut Controller) -> Result<(), ControllerError>;
}

pub struct Bot {
    drive_system: Mutex<DriveSystem>,
    shooter_system: Mutex<ShooterSystem>,
    wing_system: Mutex<WingSystem>,

    controller: Mutex<Controller>,
}

impl Robot for Bot {
    fn new(p: Peripherals) -> Self {
        println!("initialised");

        Bot {
            drive_system: Mutex::new(DriveSystem::new(
                p.port15, p.port14, p.port13, p.port18, p.port17, p.port16,
            )),
            shooter_system: Mutex::new(ShooterSystem::new(p.port11, p.port_h)),
            wing_system: Mutex::new(WingSystem::new(p.port_g)),

            controller: p.master_controller.into(),
        }
    }

    fn autonomous(&self, ctx: Context) {}

    fn opcontrol(&'static self, ctx: Context) {
        let mut pause = Loop::new(Duration::from_millis(50));

        self.drive_system.lock().driver_control_initialize();
        self.shooter_system.lock().driver_control_initialize();
        self.wing_system.lock().driver_control_initialize();

        loop {
            self.drive_system
                .lock()
                .driver_control_cycle(&mut *self.controller.lock());
            self.shooter_system
                .lock()
                .driver_control_cycle(&mut *self.controller.lock());
            self.wing_system
                .lock()
                .driver_control_cycle(&mut *self.controller.lock());

            select! {
                _ = ctx.done() => break,
                _ = pause.select() => continue
            }
        }
    }
}

entry!(Bot);
