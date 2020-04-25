use crate::MotorError;
use hal::I2cdev;
use lazy_static::lazy_static;
use linux_embedded_hal as hal;
use pwm_pca9685::{Channel, Pca9685};
use std::collections::HashMap;
use std::f32::consts::PI;

lazy_static! {
    pub static ref STEP_CHANNEL_MAP: HashMap<StepMotor, StepChannels> = {
        let mut map = HashMap::new();
        map.insert(
            StepMotor::Stepper1,
            StepChannels {
                ref_channel1: Channel::C8,
                ref_channel2: Channel::C13,
                ain1: Channel::C10,
                ain2: Channel::C9,
                bin1: Channel::C11,
                bin2: Channel::C12,
            },
        );
        map.insert(
            StepMotor::Stepper2,
            StepChannels {
                ref_channel1: Channel::C2,
                ref_channel2: Channel::C7,
                ain1: Channel::C4,
                ain2: Channel::C3,
                bin1: Channel::C5,
                bin2: Channel::C6,
            },
        );
        map
    };
}

#[derive(Clone)]
pub struct StepChannels {
    pub ref_channel1: Channel,
    pub ref_channel2: Channel,
    pub ain1: Channel,
    pub ain2: Channel,
    pub bin1: Channel,
    pub bin2: Channel,
}
#[derive(PartialEq, Debug)]
pub enum StepDirection {
    Forward,
    Backward,
}

#[derive(Debug, PartialEq)]
pub enum StepStyle {
    Single,
    Double,
    Interleave,
    Microstep,
}

#[derive(Hash, PartialEq, Eq)]
pub enum StepMotor {
    Stepper1,
    Stepper2,
}

pub struct Stepper {
    microsteps: i32,
    pub channels: StepChannels,
    curve: Vec<i32>,
    current_step: i32,
}

impl Stepper {
    /// Initializes the stepper motor. If `microsteps` is not specified, it
    /// defaults to 16.
    pub fn try_new(
        pwm: &mut Pca9685<I2cdev>,
        step_motor: StepMotor,
        microsteps: Option<i32>,
    ) -> Result<Self, MotorError> {
        let channels = STEP_CHANNEL_MAP.get(&step_motor).unwrap();
        let microsteps = microsteps.unwrap_or(16);
        let curve: Vec<i32> = (0..microsteps + 1)
            .map(|i| {
                let value = (65535.0
                    * (PI / (2.0 * microsteps as f32) * i as f32).sin())
                .round() as i32
                    + 1
                    >> 4;
                value.min(4095)
            })
            .collect();

        pwm.set_channel_off(channels.ref_channel1, 4095)
            .map_err(|_| MotorError::ChannelError)?;
        pwm.set_channel_off(channels.ref_channel2, 4095)
            .map_err(|_| MotorError::ChannelError)?;

        let mut stepper = Self {
            microsteps,
            current_step: 0,
            channels: (*channels).clone(),
            curve,
        };
        stepper.update_coils(pwm, [0; 4])?;
        Ok(stepper)
    }

    pub fn calc_step(
        &mut self,
        step_dir: StepDirection,
        step_style: StepStyle,
    ) -> Result<[i32; 4], MotorError> {
        let step_size = self.calc_step_size(&step_dir, &step_style);
        println!("step size: {}", step_size);
        match step_dir {
            StepDirection::Forward => self.current_step += step_size,
            StepDirection::Backward => self.current_step -= step_size,
        }
        println!("Current microstep: {}", self.current_step);
        let duty_cycles = self.calc_duty_cycle(step_style);
        Ok(duty_cycles)
    }

    /// Steps one of the two stepper motors once.
    pub fn step_once(
        &mut self,
        pwm: &mut Pca9685<I2cdev>,
        step_dir: StepDirection,
        step_style: StepStyle,
    ) -> Result<(), MotorError> {
        // Set the reference channels to run at full blast.
        let duty_cycle = self.calc_step(step_dir, step_style)?;
        println!("duty_cycle: {:?}", duty_cycle);
        self.update_coils(pwm, duty_cycle)?;
        Ok(())
    }

    fn update_coils(
        &mut self,
        pwm: &mut Pca9685<I2cdev>,
        duty_cycle: [i32; 4],
    ) -> Result<(), MotorError> {
        pwm.set_channel_off(self.channels.ain2, duty_cycle[0] as u16)
            .map_err(|_| MotorError::ChannelError)?;
        pwm.set_channel_off(self.channels.bin1, duty_cycle[1] as u16)
            .map_err(|_| MotorError::ChannelError)?;
        pwm.set_channel_off(self.channels.ain1, duty_cycle[2] as u16)
            .map_err(|_| MotorError::ChannelError)?;
        pwm.set_channel_off(self.channels.bin2, duty_cycle[3] as u16)
            .map_err(|_| MotorError::ChannelError)?;

        Ok(())
    }

    fn calc_duty_cycle(&mut self, step_style: StepStyle) -> [i32; 4] {
        let mut duty_cycles = [0; 4];
        let trailing_coil =
            ((self.current_step / self.microsteps) % 4) as usize;
        let leading_coil = ((trailing_coil + 1) % 4) as usize;
        let microstep = (self.current_step % self.microsteps) as usize;
        duty_cycles[leading_coil] = self.curve[microstep];
        duty_cycles[trailing_coil] =
            self.curve[self.microsteps as usize - microstep];
        if step_style != StepStyle::Microstep
            && duty_cycles[leading_coil] == duty_cycles[trailing_coil]
            && duty_cycles[leading_coil] > 0
        {
            duty_cycles[leading_coil] = 4095;
            duty_cycles[trailing_coil] = 4095;
        }
        duty_cycles
    }

    fn calc_step_size(
        &mut self,
        step_dir: &StepDirection,
        step_style: &StepStyle,
    ) -> i32 {
        if step_style == &StepStyle::Microstep {
            return 1;
        }

        let half_step = self.microsteps / 2;
        let additional_microsteps = self.current_step % half_step;
        if additional_microsteps != 0 {
            if step_dir == &StepDirection::Forward {
                self.current_step += half_step - additional_microsteps;
            } else {
                self.current_step -= additional_microsteps;
            }
            return 0;
        } else if step_style == &StepStyle::Interleave {
            return half_step;
        } else {
            let curr_interleave = self.current_step / half_step;
            if (step_style == &StepStyle::Single && curr_interleave % 2 == 1)
                || (step_style == &StepStyle::Double
                    && curr_interleave % 2 == 0)
            {
                return half_step;
            } else {
                return self.microsteps;
            }
        }
    }
}
