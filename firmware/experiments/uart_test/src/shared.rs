use core::sync::atomic::{AtomicU16, AtomicU32, Ordering};

pub struct SharedU16x8 {
    seq: AtomicU32,
    values: [AtomicU16; 8],
}

impl SharedU16x8 {
    pub const fn new() -> Self {
        Self {
            seq: AtomicU32::new(0),
            values: [
                AtomicU16::new(0),
                AtomicU16::new(0),
                AtomicU16::new(0),
                AtomicU16::new(0),
                AtomicU16::new(0),
                AtomicU16::new(0),
                AtomicU16::new(0),
                AtomicU16::new(0),
            ],
        }
    }

    pub fn store(&self, values: [u16; 8]) {
        // Seqlock write: odd while writing, even when stable.
        self.seq.fetch_add(1, Ordering::AcqRel);
        for (slot, value) in self.values.iter().zip(values.iter()) {
            slot.store(*value, Ordering::Relaxed);
        }
        self.seq.fetch_add(1, Ordering::Release);
    }

    pub fn read_snapshot(&self) -> [u16; 8] {
        loop {
            let start = self.seq.load(Ordering::Acquire);
            if start & 1 != 0 {
                continue;
            }

            let mut out = [0u16; 8];
            for (dst, src) in out.iter_mut().zip(self.values.iter()) {
                *dst = src.load(Ordering::Relaxed);
            }

            let end = self.seq.load(Ordering::Acquire);
            if start == end {
                return out;
            }
        }
    }
}

pub static POSITIONS: SharedU16x8 = SharedU16x8::new();
pub static COMMANDS: SharedU16x8 = SharedU16x8::new();
