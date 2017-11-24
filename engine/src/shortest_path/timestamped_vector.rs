use std::ops::Index;
use std::sync::atomic::compiler_fence;
use std::sync::atomic::Ordering::SeqCst;

#[derive(Debug)]
pub struct TimestampedVector<T: Copy> {
    data: Vec<T>,
    // choose something small, as overflows are not a problem
    current: u16,
    timestamps: Vec<u16>,
    default: T
}

impl<T: Copy> TimestampedVector<T> {
    pub fn new(size: usize, default: T) -> TimestampedVector<T> {
        TimestampedVector {
            data: vec![default; size],
            current: 0,
            timestamps: vec![0; size],
            default: default
        }
    }

    pub fn reset(&mut self) {
        let old = self.current;
        self.current += 1;

        if old > self.current {
            // overflow, need to reinit
            for element in self.data.iter_mut() {
                *element = self.default;
            }
        }
    }

    pub fn set(&mut self, index: usize, value: T) {
        self.data[index] = value;
        // we are going to access this container from two different threads
        // unsyncrhnoized and with one thread modifying it - completely unsafe
        // but fine for this case, we only need to make sure, that the timestamp
        // is always modified after the value - which is the cause for this memory fence
        compiler_fence(SeqCst);
        self.timestamps[index] = self.current;
    }
}

impl<T: Copy> Index<usize> for TimestampedVector<T> {
    type Output = T;

    fn index(&self, index: usize) -> &T {
        if self.timestamps[index] == self.current {
            &self.data[index]
        } else {
            &self.default
        }
    }
}