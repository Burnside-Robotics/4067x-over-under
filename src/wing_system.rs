use vex_rt::prelude::*;

use crate::{utils::Debouncer, RobotSystem};

pub struct WingSystem {
    left_wing_solenoid: AdiDigitalOutput,
    right_wing_solenoid: AdiDigitalOutput,

    left_wing_deployed: bool,
    right_wing_deployed: bool,

    left_switch_debouncer: Debouncer,
    right_switch_debouncer: Debouncer,
}

impl WingSystem {
    pub fn new(left_solenoid_port: AdiPort, right_solenoid_port: AdiPort) -> Self {
        Self {
            left_wing_solenoid: left_solenoid_port.into_adi_digital_output().unwrap(),
            right_wing_solenoid: right_solenoid_port.into_adi_digital_output().unwrap(),

            left_wing_deployed: false,
            right_wing_deployed: false,

            left_switch_debouncer: Debouncer::new(),
            right_switch_debouncer: Debouncer::new(),
        }
    }

    pub fn set_left_state(&mut self, state: bool) {
        self.left_wing_deployed = state;
        self.left_wing_solenoid.write(state);
    }

    pub fn set_right_state(&mut self, state: bool) {
        self.right_wing_deployed = state;
        self.right_wing_solenoid.write(state);
    }
}

impl RobotSystem for WingSystem {
    fn driver_control_cycle(&mut self, controller: &mut Controller) -> Result<(), ControllerError> {
        if self.left_switch_debouncer.test(controller.down.is_pressed()?) {
            self.left_wing_deployed = !self.left_wing_deployed;
        }

        if self.right_switch_debouncer.test(controller.b.is_pressed()?) {
            self.right_wing_deployed = !self.right_wing_deployed;
        }

        self.left_wing_solenoid.write(self.left_wing_deployed);
        self.right_wing_solenoid.write(self.right_wing_deployed);

        Ok(())
    }
}
