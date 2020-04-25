use lazy_static::lazy_static;
use pwm_pca9685::Channel;
use std::collections::HashMap;

lazy_static! {
    // A map of which motor uses which channels on the Motor HAT.
    pub static ref DC_CHANNEL_MAP: HashMap<DcMotor, DcChannels> = {
        let mut map = HashMap::new();
        map.insert(
            DcMotor::Motor1,
            DcChannels {
                ref_channel: Channel::C8,
                forward_channel: Channel::C9,
                backward_channel: Channel::C10,
            },
        );
        map.insert(
            DcMotor::Motor2,
            DcChannels {
                ref_channel: Channel::C13,
                forward_channel: Channel::C11,
                backward_channel: Channel::C12,
            },
        );
        map.insert(
            DcMotor::Motor3,
            DcChannels {
                ref_channel: Channel::C2,
                forward_channel: Channel::C3,
                backward_channel: Channel::C4,
            },
        );
        map.insert(
            DcMotor::Motor4,
            DcChannels {
                ref_channel: Channel::C7,
                forward_channel: Channel::C5,
                backward_channel: Channel::C6,
            },
        );
        map
    };
}

pub struct DcChannels {
    pub ref_channel: Channel,
    pub forward_channel: Channel,
    pub backward_channel: Channel,
}

#[derive(Hash, PartialEq, Eq)]
pub enum DcMotor {
    Motor1,
    Motor2,
    Motor3,
    Motor4,
}
