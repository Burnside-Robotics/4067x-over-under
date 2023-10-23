use core::time::Duration;

use colorsys::Hsl;
use vex_rt::prelude::*;

pub struct LedSystem {
    led_strip_left: AddressableLed<100>,
    led_strip_right: AddressableLed<100>,
    led_strip_back: AddressableLed<100>,
}

impl LedSystem {
    pub fn new(led_port_left: AdiPort, led_port_right: AdiPort, led_port_back: AdiPort) -> Self {
        Self {
            led_strip_right: led_port_right.into_adressable_led().unwrap(),
            led_strip_left: led_port_left.into_adressable_led().unwrap(),
            led_strip_back: led_port_back.into_adressable_led().unwrap(),
        }
    }

    pub fn initialize(ownself: &'static Mutex<Self>) {
        Task::spawn(move || {
            let mut pause = Loop::new(Duration::from_millis(5));

            let red = Hsl::new(0.0, 100.0, 30.0, None);
            let white = Hsl::new(0.0, 0.0, 60.0, None);
            let red_end = Hsl::new(360.0, 100.0, 30.0, None);

            let mut i = 0;
            loop {
                let mut self_ = ownself.lock();

                if i == 0 {
                    self_.led_strip_right.set_gradient(&red, &red_end, 0..100, false);
                } else if i == 1 {
                    self_.led_strip_left.set_gradient(&red, &red_end, 0..100, false);
                } else if i == 2 {
                    self_.led_strip_back.set_gradient(&red, &red_end, 0..100, false);
                } else if i % 3 == 0 {
                    self_.led_strip_left.rotate(1, 0..100);
                } else if i % 3 == 1 {
                    self_.led_strip_right.rotate(1, 0..100);
                } else {
                    self_.led_strip_back.rotate(1, 0..100);
                }

                i += 1;

                select! {
                    _ = pause.select() => continue
                }
            }
        });
    }
}
