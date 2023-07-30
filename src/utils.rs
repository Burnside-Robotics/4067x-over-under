use core::ops::{AddAssign, Sub, SubAssign};

pub struct Dampener<T> {
    current_value: T,

    acceleration_limit: T,
}

impl<T: AddAssign + SubAssign + PartialOrd + Copy + Sub<Output = T> + Default> Dampener<T> {
    pub fn new(acceleration_limit: T) -> Self {
        Self {
            current_value: Default::default(),
            acceleration_limit,
        }
    }

    pub fn cycle(&mut self, target: T) -> T {
        if target > self.current_value {
            if target - self.current_value < self.acceleration_limit {
                self.current_value = target;
            } else {
                self.current_value += self.acceleration_limit;
            }
        } else if target < self.current_value {
            if self.current_value - target < self.acceleration_limit {
                self.current_value = target;
            } else {
                self.current_value -= self.acceleration_limit;
            }
        }
        return self.current_value;
    }
}

pub struct Debouncer {
    value: bool,
}

impl Debouncer {
    pub fn new() -> Self {
        Self { value: false }
    }

    pub fn test(&mut self, value: bool) -> bool {
        let previous_value = self.value;
        self.value = value;
        !previous_value && value
    }
}
