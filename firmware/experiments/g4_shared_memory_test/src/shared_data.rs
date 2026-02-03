use core::sync::atomic::{AtomicU16, AtomicU32, Ordering};

pub struct SharedData<const N: usize> {
    seq: AtomicU32,
    values: [AtomicU16; N],
}

impl<const N: usize> SharedData<N> {
    pub const fn new() -> Self {
        Self {
            seq: AtomicU32::new(0),
            values: [const { AtomicU16::new(0) }; N],
        }
    }

    pub fn write_frame(&self, data: &[u16; N]) {
        self.seq.fetch_add(1, Ordering::AcqRel);
        for (idx, value) in data.iter().enumerate() {
            self.values[idx].store(*value, Ordering::Relaxed);
        }
        self.seq.fetch_add(1, Ordering::Release);
    }

    pub fn read_frame(&self, out: &mut [u16; N]) -> bool {
        let start = self.seq.load(Ordering::Acquire);
        if start & 1 == 1 {
            return false;
        }
        for (idx, slot) in self.values.iter().enumerate() {
            out[idx] = slot.load(Ordering::Relaxed);
        }
        let end = self.seq.load(Ordering::Acquire);
        start == end && (end & 1 == 0)
    }

    pub fn seq(&self) -> u32 {
        self.seq.load(Ordering::Acquire)
    }
}
