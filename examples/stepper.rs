use adafruit_motorkit::{
    stepper::{StepDirection, StepMotor, StepStyle, Stepper},
    MotorControl,
};
use std::error::Error;
use std::thread;
use std::time::Duration;

fn main() -> Result<(), Box<dyn Error>> {
    let mut motor_ctrl = MotorControl::try_new(None)?;
    let mut stepper =
        Stepper::try_new(&mut motor_ctrl.pwm, StepMotor::Stepper1, None)?;
    for _ in 1..100 {
        stepper.step_once(
            &mut motor_ctrl.pwm,
            StepDirection::Forward,
            StepStyle::Single,
        )?;
    }
    Ok(())
}
