use defmt::Format;
use enum_map::enum_map;

use crate::{
    key::{
        Action::{Control as C, Key as K, LayerModifier as LM, Pass as ___________},
        Control, Key, LayerIndex,
    },
    processor::mapper::InputMap,
};

pub const LAYER_COUNT: usize = 2;

#[derive(Clone, Copy, Default, Format, PartialEq, PartialOrd)]
pub enum Layer {
    #[default]
    Base,
    Function1,
}

impl LayerIndex for Layer {}

impl From<Layer> for usize {
    fn from(value: Layer) -> usize {
        value as usize
    }
}

pub const KEY_MATRIX_ROW_COUNT: usize = 2;
pub const KEY_MATRIX_COL_COUNT: usize = 2;

#[rustfmt::skip]
pub fn get_input_map() -> InputMap<{LAYER_COUNT}, { KEY_MATRIX_ROW_COUNT }, { KEY_MATRIX_COL_COUNT }, Layer> {
    InputMap::new(
        [
            [
                [K(Key::A),                    K(Key::B)],
                [___________,                  LM(Layer::Function1)],
            ],
            [
                [K(Key::C),                    K(Key::D)],
                [C(Control::RGBAnimationNext), ___________],
            ],
        ],
        [
            enum_map! {
                _ => ___________,
            },
            enum_map! {
                _ => ___________,
            },
        ],
    )
}
