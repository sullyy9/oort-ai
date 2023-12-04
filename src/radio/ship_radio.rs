use oort_api::prelude::*;

use super::message::RadioMessage;

#[derive(Clone, PartialEq, Debug)]
pub struct Radio {}

impl Radio {
    pub fn new() -> Self {
        return Self {};
    }

    pub fn set_channel(&self, channel: usize) {
        set_radio_channel(channel);
    }

    pub fn get_channel(&self) -> usize {
        return get_radio_channel();
    }

    pub fn send(&self, message: RadioMessage) {
        send_bytes(&message.to_bytes());
    }

    pub fn receive(&self) -> Option<RadioMessage> {
        return receive_bytes().map(|bytes| RadioMessage::from_bytes(&bytes));
    }
}
