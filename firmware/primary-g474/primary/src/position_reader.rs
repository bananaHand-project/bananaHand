use embassy_stm32::Peri;
use embassy_stm32::adc::{Adc, AdcChannel, AnyAdcChannel, SampleTime};
use embassy_stm32::peripherals::{ADC1, DMA1_CH2, PA0, PA1, PA2, PA3, PB0, PB1, PB11, PF0};
use embassy_time::{Duration, Ticker};

use crate::config::POSITION_COUNT;
use crate::shared::SharedData;

// Enable/disable each potentiometer reading.
pub const POT_ENABLED: [bool; POSITION_COUNT] = [true; POSITION_COUNT];

pub struct PosAdcPins {
    pub pinky: Peri<'static, PF0>,
    pub ring: Peri<'static, PB0>,
    pub middle: Peri<'static, PB1>,
    pub index1: Peri<'static, PA3>,
    pub index2: Peri<'static, PB11>,
    pub thumb_flex: Peri<'static, PA2>,
    pub thumb_opp: Peri<'static, PA0>,
    pub thumb_aux: Peri<'static, PA1>,
}

impl PosAdcPins {
    fn into_ordered_channels(self) -> [AnyAdcChannel<'static, ADC1>; POSITION_COUNT] {
        [
            self.index1.degrade_adc(),     // [0]
            self.middle.degrade_adc(),     // [1]
            self.ring.degrade_adc(),       // [2]
            self.pinky.degrade_adc(),      // [3]
            self.thumb_flex.degrade_adc(), // [4]
            self.thumb_opp.degrade_adc(),  // [5]
            self.index2.degrade_adc(),     // [6]
            self.thumb_aux.degrade_adc(),  // [7]
        ]
    }
}

#[embassy_executor::task]
pub async fn position_reader_task(
    mut adc: Adc<'static, ADC1>,
    mut dma: Peri<'static, DMA1_CH2>,
    pos_pins: PosAdcPins,
    shared: &'static SharedData<POSITION_COUNT>,
) -> ! {
    let mut pos_ch = pos_pins.into_ordered_channels();
    let mut buf = [0u16; POSITION_COUNT];
    let mut ticker = Ticker::every(Duration::from_hz(200));

    loop {
        adc.read(
            dma.reborrow(),
            pos_ch.iter_mut().map(|ch| (ch, SampleTime::CYCLES12_5)),
            &mut buf,
        )
        .await;

        for (idx, enabled) in POT_ENABLED.iter().enumerate() {
            if !*enabled {
                buf[idx] = 0;
            }
        }

        shared.write_frame(&buf);
        ticker.next().await;
    }
}
