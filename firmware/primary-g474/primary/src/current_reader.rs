use embassy_stm32::Peri;
use embassy_stm32::adc::{Adc, AdcChannel, AnyAdcChannel, SampleTime};
use embassy_stm32::peripherals::{ADC2, DMA2_CH1, PA4, PA5, PA6, PA7, PB2, PC4, PC5, PF1};
use embassy_time::{Duration, Ticker};

use crate::config::CURRENT_COUNT;
use crate::shared::SharedData;

pub const CURRENT_ENABLED: [bool; CURRENT_COUNT] = [true; CURRENT_COUNT];

pub struct CurAdcPins {
    pub index1: Peri<'static, PB2>,
    pub middle: Peri<'static, PC5>,
    pub ring: Peri<'static, PA7>,
    pub pinky: Peri<'static, PA6>,
    pub thumb_flex: Peri<'static, PA5>,
    pub thumb_revolve: Peri<'static, PF1>,
    pub index2: Peri<'static, PC4>,
    pub thumb_aux: Peri<'static, PA4>,
}

impl CurAdcPins {
    fn into_ordered_channels(self) -> [AnyAdcChannel<'static, ADC2>; CURRENT_COUNT] {
        [
            self.index1.degrade_adc(),        // [0]
            self.middle.degrade_adc(),        // [1]
            self.ring.degrade_adc(),          // [2]
            self.pinky.degrade_adc(),         // [3]
            self.thumb_flex.degrade_adc(),    // [4]
            self.thumb_revolve.degrade_adc(), // [5]
            self.index2.degrade_adc(),        // [6]
            self.thumb_aux.degrade_adc(),     // [7]
        ]
    }
}

#[embassy_executor::task]
pub async fn current_reader_task(
    mut adc: Adc<'static, ADC2>,
    mut dma: Peri<'static, DMA2_CH1>,
    current_pins: CurAdcPins,
    shared: &'static SharedData<CURRENT_COUNT>,
) -> ! {
    let mut current_ch = current_pins.into_ordered_channels();
    let mut buf = [0u16; CURRENT_COUNT];
    // Currently no support in Embassy for triggering ADC conversion from HRTIM clock.
    let mut ticker = Ticker::every(Duration::from_hz(40000));

    loop {
        adc.read(
            dma.reborrow(),
            current_ch.iter_mut().map(|ch| (ch, SampleTime::CYCLES12_5)),
            &mut buf,
        )
        .await;

        for (idx, enabled) in CURRENT_ENABLED.iter().enumerate() {
            if !*enabled {
                buf[idx] = 0;
            }
        }

        shared.write_frame(&buf);
        ticker.next().await;
    }
}
