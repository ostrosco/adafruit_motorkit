use adafruit_motorkit::{
    init_pwm,
    stepper::{StepDirection, StepStyle, StepperMotor},
    Motor,
};
use std::error::Error;

fn main() -> Result<(), Box<dyn Error>> {
    let mut pwm = init_pwm(None)?;
    let mut stepper = StepperMotor::try_new(&mut pwm, Motor::Stepper1, None)?;
    for _ in 1..100 {
        stepper.step_once(
            &mut pwm,
            StepDirection::Forward,
            StepStyle::Single,
        )?;
    }
    stepper.stop(&mut pwm)?;
    Ok(())
}
