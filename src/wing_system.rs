use crate::{utils::Debouncer, DriverControlHandler};
use vex_rt::prelude::*;

pub struct WingSystem {
    wing_solenoid: AdiDigitalOutput,

    wings_deployed: bool,

    wing_switch_debouncer: Debouncer,
}

impl WingSystem {
    pub fn new(solenoid_port: AdiPort) -> Self {
        Self {
            wing_solenoid: solenoid_port.into_adi_digital_output().unwrap(),

            wings_deployed: false,

            wing_switch_debouncer: Debouncer::new(),
        }
    }
}

impl DriverControlHandler for WingSystem {
    fn driver_control_cycle(&mut self, controller: &Controller) -> Result<(), ControllerError> {
        if self
            .wing_switch_debouncer
            .test(controller.right.is_pressed()?)
        {
            self.wings_deployed = !self.wings_deployed;
        }

        self.wing_solenoid.write(self.wings_deployed);

        Ok(())
    }
}
