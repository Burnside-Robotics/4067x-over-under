use crate::{utils::Debouncer, DriverControlHandler};
use uom::si::{f64::Ratio, ratio::percent};
use vex_rt::{prelude::*, smart_port::SmartPort};

pub struct ShooterSystem {
    catapult_motor: Motor,
    limit_switch: AdiDigitalInput,

    manual_enabled: bool,

    manual_switch_debouncer: Debouncer,
}

impl ShooterSystem {
    pub fn new(motor_port: SmartPort, switch_port: AdiPort) -> Self {
        Self {
            catapult_motor: motor_port.into_motor(
                Gearset::ThirtySixToOne,
                EncoderUnits::Degrees,
                false,
            ),
            limit_switch: switch_port.into_adi_digital_input().unwrap(),
            manual_enabled: false,
            manual_switch_debouncer: Debouncer::new(),
        }
    }
}

impl DriverControlHandler for ShooterSystem {
    fn driver_control_cycle(&mut self, controller: &Controller) -> Result<(), ControllerError> {
        if self
            .manual_switch_debouncer
            .test(controller.y.is_pressed()?)
        {
            self.manual_enabled = !self.manual_enabled;
        }

        let velocity = if self.manual_enabled {
            if controller.r1.is_pressed()? {
                75.0
            } else if controller.r2.is_pressed()? {
                -75.0
            } else {
                0.0
            }
        } else {
            if self.limit_switch.read().unwrap() && !controller.r1.is_pressed()? {
                10.0
            } else {
                75.0
            }
        };

        self.catapult_motor
            .move_ratio(Ratio::new::<percent>(velocity));

        Ok(())
    }
}
