use alloc::{boxed::Box, rc::Rc, vec::Vec};
use async_trait::async_trait;
use core::cell::RefCell;
use defmt::Format;
use embedded_hal::digital::{InputPin, OutputPin};
use rp2040_hal::gpio;
use rtic_monotonics::rp2040::prelude::*;
use rtic_sync::arbiter::Arbiter;
use serde::{de, ser::SerializeStruct, Deserialize, Serialize};

use crate::{
    kb::Mono,
    key::Edge,
    remote::{MethodId, RemoteInvoker, Service, ServiceId},
    split, util,
};

#[derive(Clone, Copy, Debug, Format)]
pub struct Result<const ROW_COUNT: usize, const COL_COUNT: usize> {
    pub scan_time_ticks: u64,
    pub matrix: [[Bit; COL_COUNT]; ROW_COUNT],
}

impl<const ROW_COUNT: usize, const COL_COUNT: usize> Serialize for Result<ROW_COUNT, COL_COUNT> {
    fn serialize<S>(&self, serializer: S) -> core::result::Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        let mut state = serializer.serialize_struct("Result", 2)?;
        state.serialize_field("scan_time_ticks", &self.scan_time_ticks)?;
        state.serialize_field(
            "matrix",
            &self
                .matrix
                .iter()
                .map(|row| row.to_vec())
                .collect::<Vec<Vec<Bit>>>(),
        )?;
        state.end()
    }
}

impl<'de, const ROW_COUNT: usize, const COL_COUNT: usize> Deserialize<'de>
    for Result<ROW_COUNT, COL_COUNT>
{
    fn deserialize<D>(deserializer: D) -> core::result::Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct ResultOwned {
            scan_time_ticks: u64,
            matrix: Vec<Vec<Bit>>,
        }

        let temp = ResultOwned::deserialize(deserializer)?;
        if temp.matrix.len() != ROW_COUNT || temp.matrix.iter().any(|row| row.len() != COL_COUNT) {
            return Err(de::Error::custom(
                "matrix dimensions do not match expectation",
            ));
        }

        let mut result = Result::<ROW_COUNT, COL_COUNT> {
            scan_time_ticks: temp.scan_time_ticks,
            ..Default::default()
        };
        for (i, row) in temp.matrix.into_iter().enumerate() {
            for (j, bit) in row.into_iter().enumerate() {
                result.matrix[i][j] = bit;
            }
        }

        Ok(result)
    }
}

#[derive(Clone, Copy, Debug, Deserialize, Format, Serialize)]
pub struct Bit {
    pub edge: Edge,
    pub pressed: bool,
}

impl<const ROW_COUNT: usize, const COL_COUNT: usize> Default for Result<ROW_COUNT, COL_COUNT> {
    fn default() -> Self {
        Result {
            scan_time_ticks: 0,
            matrix: [[Bit {
                edge: Edge::None,
                pressed: false,
            }; COL_COUNT]; ROW_COUNT],
        }
    }
}

pub trait Scanner<const ROW_COUNT: usize, const COL_COUNT: usize> {
    async fn scan(&mut self) -> Result<ROW_COUNT, COL_COUNT>;
}

pub trait SplitScanner<const ROW_COUNT: usize, const COL_COUNT: usize>:
    Scanner<ROW_COUNT, { COL_COUNT / 2 }>
where
    [(); COL_COUNT / 2]:,
{
    async fn scan<I>(&mut self, client: &Arbiter<Rc<RefCell<I>>>) -> Result<ROW_COUNT, COL_COUNT>
    where
        I: RemoteInvoker;
}

pub struct SplitSwitchMatrix<const ROW_COUNT: usize, const COL_COUNT: usize>
where
    [(); COL_COUNT / 2]:,
{
    local_matrix: BasicVerticalSwitchMatrix<{ ROW_COUNT }, { COL_COUNT / 2 }>, // TODO: use boxed scanner
}

#[allow(dead_code)]
impl<const ROW_COUNT: usize, const COL_COUNT: usize> SplitSwitchMatrix<ROW_COUNT, COL_COUNT>
where
    [(); COL_COUNT / 2]:,
{
    pub fn new(local_matrix: BasicVerticalSwitchMatrix<{ ROW_COUNT }, { COL_COUNT / 2 }>) -> Self {
        SplitSwitchMatrix { local_matrix }
    }
}

const SERVICE_ID_KEY_MATRIX: ServiceId = 0x10;
const METHOD_ID_KEY_MATRIX_SCAN: MethodId = 0x11;

impl<const ROW_COUNT: usize, const COL_COUNT: usize> Scanner<ROW_COUNT, { COL_COUNT / 2 }>
    for SplitSwitchMatrix<ROW_COUNT, COL_COUNT>
where
    [(); COL_COUNT / 2]:,
{
    async fn scan(&mut self) -> Result<ROW_COUNT, { COL_COUNT / 2 }> {
        let start_time = Mono::now();
        let result = self.local_matrix.scan().await;
        let end_time = Mono::now();
        util::log_duration(util::LogDurationTag::KeyMatrixScan, start_time, end_time);
        result
    }
}

impl<const ROW_COUNT: usize, const COL_COUNT: usize> SplitScanner<ROW_COUNT, COL_COUNT>
    for SplitSwitchMatrix<ROW_COUNT, COL_COUNT>
where
    [(); COL_COUNT / 2]:,
{
    async fn scan<I>(&mut self, client: &Arbiter<Rc<RefCell<I>>>) -> Result<ROW_COUNT, COL_COUNT>
    where
        I: RemoteInvoker,
    {
        let remote_result = client
            .access()
            .await
            .borrow_mut()
            .invoke::<SwitchMatrixScanRequest, SwitchMatrixScanResponse<{ROW_COUNT}, {COL_COUNT/2}>>(
                SERVICE_ID_KEY_MATRIX,
                METHOD_ID_KEY_MATRIX_SCAN,
                SwitchMatrixScanRequest {},
            )
            .await.result;
        let local_result: Result<{ ROW_COUNT }, { COL_COUNT / 2 }> = Scanner::scan(self).await;

        // merge
        let mut merged_matrix = [[Bit {
            edge: Edge::None,
            pressed: false,
        }; COL_COUNT]; ROW_COUNT];

        let (left_matrix, right_matrix) = match split::get_self_side() {
            split::Side::Left => (local_result.matrix, remote_result.matrix),
            split::Side::Right => (remote_result.matrix, local_result.matrix),
        };
        #[allow(clippy::needless_range_loop)]
        for i in 0..ROW_COUNT {
            for j in 0..(COL_COUNT / 2) {
                merged_matrix[i][j] = left_matrix[i][j];
                merged_matrix[i][j + COL_COUNT / 2] = right_matrix[i][j];
            }
        }

        Result {
            scan_time_ticks: local_result.scan_time_ticks,
            matrix: merged_matrix,
        }
    }
}

#[async_trait]
impl<const ROW_COUNT: usize, const COL_COUNT: usize> Service
    for SplitSwitchMatrix<ROW_COUNT, COL_COUNT>
where
    [(); COL_COUNT / 2]:,
{
    fn get_service_id(&self) -> ServiceId {
        SERVICE_ID_KEY_MATRIX
    }

    async fn dispatch(&mut self, method_id: MethodId, _request_buffer: &[u8]) -> Vec<u8> {
        // info!("Ping::dispatch()");
        match method_id {
            METHOD_ID_KEY_MATRIX_SCAN => {
                let result = Scanner::scan(self).await;
                postcard::to_allocvec(&SwitchMatrixScanResponse { result }).unwrap()
            }
            _ => {
                defmt::error!("unimplemented");
                Vec::new()
            }
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize, Format, Serialize)]
pub struct SwitchMatrixScanRequest {}

#[derive(Clone, Copy, Debug, Deserialize, Format, Serialize)]
pub struct SwitchMatrixScanResponse<const ROW_COUNT: usize, const COL_COUNT: usize> {
    result: Result<{ ROW_COUNT }, { COL_COUNT }>,
}

pub struct BasicVerticalSwitchMatrix<const ROW_COUNT: usize, const COL_COUNT: usize> {
    pub rows: [Box<dyn InputPin<Error = gpio::Error> + Sync + Send>; ROW_COUNT],
    pub cols: [Box<dyn OutputPin<Error = gpio::Error> + Sync + Send>; COL_COUNT],
    previous_result: Result<{ ROW_COUNT }, { COL_COUNT }>,
}

impl<const ROW_COUNT: usize, const COL_COUNT: usize>
    BasicVerticalSwitchMatrix<ROW_COUNT, COL_COUNT>
{
    pub fn new(
        rows: [Box<dyn InputPin<Error = gpio::Error> + Sync + Send>; ROW_COUNT],
        cols: [Box<dyn OutputPin<Error = gpio::Error> + Sync + Send>; COL_COUNT],
    ) -> Self {
        BasicVerticalSwitchMatrix {
            rows,
            cols,
            previous_result: Result::default(),
        }
    }
}

impl<const ROW_COUNT: usize, const COL_COUNT: usize> Scanner<ROW_COUNT, COL_COUNT>
    for BasicVerticalSwitchMatrix<ROW_COUNT, COL_COUNT>
{
    async fn scan(&mut self) -> Result<ROW_COUNT, COL_COUNT> {
        let mut result = Result::default();
        for (j, col) in self.cols.iter_mut().enumerate() {
            col.set_high().unwrap();
            for (i, row) in self.rows.iter_mut().enumerate() {
                let pressed = row.is_high().unwrap();
                result.matrix[i][j] = Bit {
                    edge: Edge::from((self.previous_result.matrix[i][j].pressed, pressed)),
                    pressed,
                }
            }
            col.set_low().unwrap();
            Mono::delay(1.micros()).await;
        }
        result.scan_time_ticks = Mono::now().ticks();
        self.previous_result = result;
        result
    }
}

pub struct BasicHorizontalSwitchMatrix<const ROW_COUNT: usize, const COL_COUNT: usize> {
    pub rows: [Box<dyn OutputPin<Error = gpio::Error> + Sync + Send>; ROW_COUNT],
    pub cols: [Box<dyn InputPin<Error = gpio::Error> + Sync + Send>; COL_COUNT],
    previous_result: Result<{ ROW_COUNT }, { COL_COUNT }>,
}

#[allow(dead_code)]
impl<const ROW_COUNT: usize, const COL_COUNT: usize>
    BasicHorizontalSwitchMatrix<ROW_COUNT, COL_COUNT>
{
    pub fn new(
        rows: [Box<dyn OutputPin<Error = gpio::Error> + Sync + Send>; ROW_COUNT],
        cols: [Box<dyn InputPin<Error = gpio::Error> + Sync + Send>; COL_COUNT],
    ) -> Self {
        BasicHorizontalSwitchMatrix {
            rows,
            cols,
            previous_result: Result::default(),
        }
    }
}

impl<const ROW_COUNT: usize, const COL_COUNT: usize> Scanner<ROW_COUNT, COL_COUNT>
    for BasicHorizontalSwitchMatrix<ROW_COUNT, COL_COUNT>
{
    async fn scan(&mut self) -> Result<ROW_COUNT, COL_COUNT> {
        let mut result = Result::default();
        for (i, row) in self.rows.iter_mut().enumerate() {
            row.set_high().unwrap();
            for (j, col) in self.cols.iter_mut().enumerate() {
                let pressed = col.is_high().unwrap();
                result.matrix[i][j] = Bit {
                    edge: Edge::from((self.previous_result.matrix[i][j].pressed, pressed)),
                    pressed,
                }
            }
            row.set_low().unwrap();
            Mono::delay(1.micros()).await;
        }
        self.previous_result = result;
        result
    }
}
