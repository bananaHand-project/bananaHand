use embassy_stm32::Peri;
use embassy_stm32::adc::{Adc, AnyAdcChannel, SampleTime};
use embassy_stm32::peripherals::{ADC1, DMA1_CH2};
use embassy_time::{Duration, Ticker};

use crate::config::POSITION_COUNT;
use crate::shared::SharedData;

// Enable/disable each potentiometer reading.
pub const POT_ENABLED: [bool; POSITION_COUNT] = [true; POSITION_COUNT];

#[embassy_executor::task]
pub async fn position_reader_task(
    mut adc: Adc<'static, ADC1>,
    mut dma: Peri<'static, DMA1_CH2>,
    mut pos_ch: [AnyAdcChannel<ADC1>; POSITION_COUNT],
    shared: &'static SharedData<POSITION_COUNT>,
) -> ! {
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
