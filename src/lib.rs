use hal::I2cdev;
use linux_embedded_hal as hal;
use pwm_pca9685::{Channel, Pca9685, SlaveAddr};
use std::cmp::Ordering;
use std::error::Error;
use std::fmt;

pub mod dc;
use dc::{DcMotor, DC_CHANNEL_MAP};

pub mod stepper;

#[derive(Debug)]
pub enum MotorError {
    /// An error occurred initializing the I2C bus.
    I2cError,
    /// An error occurred configuring the PCA9685.
    PwmError,
    /// An error occurred setting a channel.
    ChannelError,
    /// The value for throttle is not in the bounds of [-1.0, 1.0].
    ThrottleError,
}

impl fmt::Display for MotorError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:?}", self)
    }
}

pub struct MotorControl {
    pub pwm: Pca9685<I2cdev>,
}

impl Error for MotorError {}

impl MotorControl {
    /// Attempt to create a new interface to the Motor HAT. This assumes
    /// several defaults for ease in establishing a connection:
    /// - Assumes only one Motor HAT as 0x96.
    /// - Assumes only a pre-scale of 100 so the HAT is running at 60 Hz.
    ///
    /// If no I2C bus is provided to the function, it will attempt to
    /// connect to /dev/i2c-1 which will work in most cases.
    pub fn try_new(i2c: Option<I2cdev>) -> Result<Self, MotorError> {
        let i2c = if let Some(i2c) = i2c {
            i2c
        } else {
            I2cdev::new("/dev/i2c-1").map_err(|_| MotorError::I2cError)?
        };

        // The default address for the motor hat is 0x96.
        let address =
            SlaveAddr::Alternative(true, false, false, false, false, false);

        let mut pwm = Pca9685::new(i2c, address);
        pwm.enable().map_err(|_| MotorError::PwmError)?;
        pwm.set_channel_on(Channel::All, 0)
            .map_err(|_| MotorError::PwmError)?;
        pwm.set_prescale(100).map_err(|_| MotorError::PwmError)?;
        Ok(MotorControl { pwm })
    }

    /// Sets the throttle for one of the four DC motors.
    pub fn set_dc_motor(
        &mut self,
        motor: DcMotor,
        throttle: f32,
    ) -> Result<(), MotorError> {
        if throttle > 1.0 || throttle < -1.0 {
            return Err(MotorError::ThrottleError);
        }
        let channels = DC_CHANNEL_MAP.get(&motor).unwrap();

        // Set the reference channel to run at full blast.
        self.pwm
            .set_channel_off(channels.ref_channel, 4095)
            .map_err(|_| MotorError::ChannelError)?;

        let duty_cycle = (4095.0 * throttle.abs()) as u16;

        match throttle.partial_cmp(&0.0) {
            Some(Ordering::Greater) => {
                self.pwm
                    .set_channel_off(channels.forward_channel, duty_cycle)
                    .map_err(|_| MotorError::ChannelError)?;
            }
            Some(Ordering::Less) => {
                self.pwm
                    .set_channel_off(channels.backward_channel, duty_cycle)
                    .map_err(|_| MotorError::ChannelError)?;
            }
            _ => {
                self.pwm
                    .set_channel_full_off(channels.forward_channel)
                    .map_err(|_| MotorError::ChannelError)?;
                self.pwm
                    .set_channel_full_off(channels.backward_channel)
                    .map_err(|_| MotorError::ChannelError)?;
            }
        }
        Ok(())
    }
}
