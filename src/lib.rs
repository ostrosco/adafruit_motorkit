use hal::I2cdev;
use linux_embedded_hal as hal;
use pwm_pca9685::{Channel, Pca9685, SlaveAddr};
use std::error::Error;

pub struct Motor {
    pwm: Pca9685<I2cdev>,
}

impl Motor {
    pub fn try_new() -> Result<Self, Box<dyn Error>> {
        let i2c = I2cdev::new("/dev/i2c-1")?;
        // The default address for the motor hat is 0x96.
        let address =
            SlaveAddr::Alternative(true, false, false, false, false, false);
        let mut pwm = Pca9685::new(i2c, address);
        pwm.enable().map_err(|_| "Couldn't enable the PWM")?;
        pwm.set_channel_on(Channel::All, 0)
            .expect("Couldn't set channel on");
        pwm.set_prescale(100)
            .map_err(|_| "Couldn't set the prescale")?;
        Ok(Motor { pwm })
    }

    pub fn set_motor1(&mut self, throttle: f32) {
        let pwm_channel = Channel::C8;
        let forward_channel = Channel::C9;
        let backward_channel = Channel::C10;
        self.set_motor(
            pwm_channel,
            forward_channel,
            backward_channel,
            throttle,
        );
    }

    pub fn set_motor2(&mut self, throttle: f32) {
        let pwm_channel = Channel::C13;
        let forward_channel = Channel::C11;
        let backward_channel = Channel::C12;
        self.set_motor(
            pwm_channel,
            forward_channel,
            backward_channel,
            throttle,
        );
    }

    pub fn set_motor3(&mut self, throttle: f32) {
        let pwm_channel = Channel::C2;
        let forward_channel = Channel::C3;
        let backward_channel = Channel::C4;
        self.set_motor(
            pwm_channel,
            forward_channel,
            backward_channel,
            throttle,
        );
    }

    pub fn set_motor4(&mut self, throttle: f32) {
        let pwm_channel = Channel::C7;
        let forward_channel = Channel::C5;
        let backward_channel = Channel::C6;
        self.set_motor(
            pwm_channel,
            forward_channel,
            backward_channel,
            throttle,
        );
    }

    fn set_motor(
        &mut self,
        pwm_channel: Channel,
        forward_channel: Channel,
        backward_channel: Channel,
        throttle: f32,
    ) {
        self.pwm
            .set_channel_off(pwm_channel, 4095)
            .expect("Couldn't set PWM input");
        let duty_cycle = (4095.0 * throttle.abs()) as u16;
        if throttle > 0.0 {
            println!("Setting forward channel");
            self.pwm
                .set_channel_off(forward_channel, duty_cycle)
                .expect("Couldn't set forward channel");
        } else if throttle < 0.0 {
            self.pwm
                .set_channel_off(backward_channel, duty_cycle)
                .expect("Couldn't set forward channel");
        } else {
            self.pwm
                .set_channel_full_off(forward_channel)
                .expect("Couldn't turn off forward channel");
            self.pwm
                .set_channel_full_off(backward_channel)
                .expect("Couldn't turn off forward channel");
        }
    }
}
