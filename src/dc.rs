use crate::{Motor, MotorError};
use hal::I2cdev;
use linux_embedded_hal as hal;
use pwm_pca9685::{Channel, Pca9685};
use std::cmp::Ordering;

/// Return DC channel map for the given motor type.
/// Returns None if given motor is no DC motor.
pub fn get_dc_channels_map(motor: Motor) -> Option<DcChannels> {
    let channels = match motor {
        Motor::Motor1 => DcChannels {
            ref_channel: Channel::C8,
            forward_channel: Channel::C9,
            backward_channel: Channel::C10,
        },
        Motor::Motor2 => DcChannels {
            ref_channel: Channel::C13,
            forward_channel: Channel::C11,
            backward_channel: Channel::C12,
        },
        Motor::Motor3 => DcChannels {
            ref_channel: Channel::C2,
            forward_channel: Channel::C3,
            backward_channel: Channel::C4,
        },
        Motor::Motor4 => DcChannels {
            ref_channel: Channel::C7,
            forward_channel: Channel::C5,
            backward_channel: Channel::C6,
        },
        _ => return None,
    };

    Some(channels)
}

#[derive(Clone)]
/// A structure encapsulating the channels used to control a DC motor.
pub struct DcChannels {
    ref_channel: Channel,
    forward_channel: Channel,
    backward_channel: Channel,
}

/// A structure to initialize and control a DC motor.
pub struct DcMotor {
    channels: DcChannels,
}

impl DcMotor {
    /// Attempts to initialize a DC motor.
    pub fn try_new(
        pwm: &mut Pca9685<I2cdev>,
        motor: Motor,
    ) -> Result<Self, MotorError> {
        let channels =
            get_dc_channels_map(motor).ok_or(MotorError::InvalidMotorError)?;

        // Set the channels we'll be using to on at 0.
        pwm.set_channel_on(channels.ref_channel, 0)
            .map_err(|_| MotorError::ChannelError)?;
        pwm.set_channel_on(channels.forward_channel, 0)
            .map_err(|_| MotorError::ChannelError)?;
        pwm.set_channel_on(channels.backward_channel, 0)
            .map_err(|_| MotorError::ChannelError)?;

        // Set the reference channel to run at full blast.
        pwm.set_channel_off(channels.ref_channel, 4095)
            .map_err(|_| MotorError::ChannelError)?;
        Ok(Self { channels })
    }

    /// Sets the throttle for the motor. Valid throttle values are in the
    /// range [-1.0, 1.0].
    pub fn set_throttle(
        &mut self,
        pwm: &mut Pca9685<I2cdev>,
        throttle: f32,
    ) -> Result<(), MotorError> {
        if throttle > 1.0 || throttle < -1.0 {
            return Err(MotorError::ThrottleError);
        }
        let duty_cycle = (4095.0 * throttle.abs()) as u16;

        match throttle.partial_cmp(&0.0) {
            Some(Ordering::Greater) => {
                pwm.set_channel_off(self.channels.forward_channel, duty_cycle)
                    .map_err(|_| MotorError::ChannelError)?;
            }
            Some(Ordering::Less) => {
                pwm.set_channel_off(self.channels.backward_channel, duty_cycle)
                    .map_err(|_| MotorError::ChannelError)?;
            }
            _ => {
                pwm.set_channel_full_off(self.channels.forward_channel)
                    .map_err(|_| MotorError::ChannelError)?;
                pwm.set_channel_full_off(self.channels.backward_channel)
                    .map_err(|_| MotorError::ChannelError)?;
            }
        }
        Ok(())
    }

    /// Stops energizing the PWMs for this motor.
    pub fn stop(
        &mut self,
        pwm: &mut Pca9685<I2cdev>,
    ) -> Result<(), MotorError> {
        // Set the reference channel to run at full blast.
        pwm.set_channel_full_off(self.channels.ref_channel)
            .map_err(|_| MotorError::ChannelError)?;
        pwm.set_channel_full_off(self.channels.forward_channel)
            .map_err(|_| MotorError::ChannelError)?;
        pwm.set_channel_full_off(self.channels.backward_channel)
            .map_err(|_| MotorError::ChannelError)?;
        Ok(())
    }
}
