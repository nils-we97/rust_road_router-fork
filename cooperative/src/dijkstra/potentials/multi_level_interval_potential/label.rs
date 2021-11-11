use rust_road_router::datastr::graph::Weight;
use rust_road_router::datastr::timestamped_vector::Reset;
use std::ops::{Index, IndexMut};

#[derive(Clone, Debug)]
pub struct MultiLevelBucketLabel {
    pub data: Vec<Weight>,
}

impl MultiLevelBucketLabel {
    pub fn new(num_levels: usize) -> Self {
        Self {
            data: vec![Weight::DEFAULT; num_levels],
        }
    }

    pub fn set(&mut self, index: usize, value: Weight) {
        self.data[index] = value;
    }

    pub fn set_all(&mut self, value: Weight) {
        self.data.iter_mut().for_each(|val| *val = value);
    }
}

impl Reset for MultiLevelBucketLabel {
    const DEFAULT: Self = Self { data: vec![] }; // not used anyway
    fn reset(&mut self) {
        self.data.iter_mut().for_each(|entry| *entry = Weight::DEFAULT)
    }
}

impl Index<usize> for MultiLevelBucketLabel {
    type Output = Weight;

    fn index(&self, index: usize) -> &Self::Output {
        &self.data[index]
    }
}

impl IndexMut<usize> for MultiLevelBucketLabel {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.data[index]
    }
}
