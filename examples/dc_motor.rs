use adafruit_motorkit::{dc::DcMotor, MotorControl};
use std::error::Error;
use std::thread;
use std::time::Duration;

fn main() -> Result<(), Box<dyn Error>> {
    let mut motor_ctrl = MotorControl::try_new(None)?;
    motor_ctrl.set_dc_motor(DcMotor::Motor1, 0.5)?;
    thread::sleep(Duration::from_secs(5));
    motor_ctrl.set_dc_motor(DcMotor::Motor1, 0.0)?;
    Ok(())
}
