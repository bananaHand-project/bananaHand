use embassy_stm32::Peri;
use embassy_stm32::adc::{Adc, SampleTime};
use embassy_stm32::peripherals::{ADC2, DMA2_CH1};
use embassy_time::{Duration, Ticker};

use crate::board_map::CurrentAdcPins;
use crate::config::CURRENT_COUNT;
use crate::shared::SharedData;

pub const CURRENT_ENABLED: [bool; CURRENT_COUNT] = [true; CURRENT_COUNT];

#[embassy_executor::task]
pub async fn current_reader_task(
    mut adc: Adc<'static, ADC2>,
    mut dma: Peri<'static, DMA2_CH1>,
    current_pins: CurrentAdcPins,
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
