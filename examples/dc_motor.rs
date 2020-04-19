use adafruit_motorhat::Motor;
use std::error::Error;

fn main() -> Result<(), Box<dyn Error>> {
    let mut motor = Motor::try_new()?;
    motor.set_motor1(1.0);
    loop {}
}
