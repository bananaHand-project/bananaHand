use embassy_stm32::Peri;
use embassy_stm32::adc::{Adc, AnyAdcChannel, SampleTime};
use embassy_stm32::peripherals::{ADC1, DMA1_CH2};
use embassy_time::{Duration, Ticker};

use crate::shared_data::SharedData;

pub const NUM_POTS: usize = 8;

// Enable/disable each potentiometer reading.
pub const POT_ENABLED: [bool; NUM_POTS] = [true; NUM_POTS];

#[embassy_executor::task]
pub async fn position_reader_task(
    mut adc: Adc<'static, ADC1>,
    mut dma: Peri<'static, DMA1_CH2>,
    mut pos_ch: [AnyAdcChannel<ADC1>; NUM_POTS],
    shared: &'static SharedData<NUM_POTS>,
) -> ! {
    let mut buf = [0u16; NUM_POTS];
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
