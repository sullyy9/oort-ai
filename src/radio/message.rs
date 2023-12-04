use std::mem::MaybeUninit;

use oort_api::prelude::*;

#[derive(Clone, PartialEq, Debug)]
pub enum RadioMessage {
    Position(Vec2),
    Unknown,
}

impl RadioMessage {
    pub fn to_bytes(&self) -> Vec<u8> {
        match self {
            Self::Position(position) => [&[1], &serialise_vec2(position)[..]].concat(),
            Self::Unknown => Vec::new(),
        }
    }

    pub fn from_bytes(bytes: &[u8]) -> Self {
        let type_byte = bytes[0];

        match type_byte {
            1 => Self::Position(deserialise_vec2(bytes[1..17].try_into().unwrap())),
            _ => Self::Unknown,
        }
    }
}

////////////////////////////////////////////////////////////////

fn serialise_vec2(vec: &Vec2) -> [u8; 16] {
    let mut bytes: [MaybeUninit<u8>; 16] = unsafe { MaybeUninit::uninit().assume_init() };

    bytes[..8].copy_from_slice(&vec.x.to_le_bytes().map(MaybeUninit::new));
    bytes[8..].copy_from_slice(&vec.y.to_le_bytes().map(MaybeUninit::new));

    return unsafe { std::mem::transmute(bytes) };
}

fn deserialise_vec2(bytes: [u8; 16]) -> Vec2 {
    let bytes: [[u8; 8]; 2] = unsafe { std::mem::transmute(bytes) };

    return vec2(f64::from_le_bytes(bytes[0]), f64::from_le_bytes(bytes[1]));
}

////////////////////////////////////////////////////////////////
