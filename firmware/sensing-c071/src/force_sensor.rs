use embassy_stm32::adc::{Adc, AnyAdcChannel, Instance};

pub struct ForceSensor<C: Instance> {
    channel: AnyAdcChannel<C>,
    last_raw: u16,
}

impl<C: Instance> ForceSensor<C> {
    pub const ADC_MAX_RAW: u16 = 4095;
    pub const ADC_VREF: f32 = 3.3;
    pub const FSR_FORCE_MIN: f32 = 0.0;
    pub const FSR_FORCE_MAX: f32 = 500.0;
    pub const FSR_BITS_MIN: u16 = 0;
    pub const FSR_BITS_MAX: u16 = 4095;

    pub fn new(channel: AnyAdcChannel<C>) -> Self {
        Self {
            channel,
            last_raw: 0,
        }
    }

    pub fn read_raw_blocking(&mut self, adc: &mut Adc<'_, C>) -> u16 {
        let raw = adc.blocking_read(&mut self.channel);
        self.last_raw = raw;
        raw
    }

    pub fn read_voltage_blocking(&mut self, adc: &mut Adc<'_, C>) -> f32 {
        let raw = self.read_raw_blocking(adc);
        (raw as f32 / Self::ADC_MAX_RAW as f32) * Self::ADC_VREF
    }

    pub fn read_force_blocking(&mut self, adc: &mut Adc<'_, C>) -> f32 {
        let raw = self.read_raw_blocking(adc);
        let t = (raw.saturating_sub(Self::FSR_BITS_MIN)) as f32
            / (Self::FSR_BITS_MAX - Self::FSR_BITS_MIN) as f32;
        Self::FSR_FORCE_MIN + t * (Self::FSR_FORCE_MAX - Self::FSR_FORCE_MIN)
    }

    pub fn last_raw(&self) -> u16 {
        self.last_raw
    }

    pub fn last_voltage(&self) -> f32 {
        (self.last_raw as f32 / Self::ADC_MAX_RAW as f32) * Self::ADC_VREF
    }
}
