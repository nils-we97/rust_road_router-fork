//! Data structure to efficiently map a large noncensecutive global id space into a smaller consecutive id space from 0 to n where the order is preserved

use std::cmp::min;
use std::mem::size_of;
use std::ops::Deref;

use std::alloc::{alloc_zeroed, Layout};

use crate::io::*;

/// the number of bytes in one L1 cache line
/// hardcoded for now, no idea, if we can retreave it during compilation
/// if calculated at compile time we must ensure its >= 8, otherwise the alignment of the 64bit vec will be broken.
const CACHE_LINE_WIDTH: usize = 64; // bytes

/// number of bits (not bytes) in one 64 bit uint
const STORAGE_BITS: usize = size_of::<u64>() * 8;

/// Bitvector or rather Bitarray (size can't be changed after creation) with a few extras:
/// - we need access to the ints containing the actualy bits, then do popcount on them
/// - the memory needs to be aligned to the cache line width
/// -> create a thin wrapper around a Vec<u64>, where we do the first allocation ourselves, and have operations to set individual bits
#[derive(Debug)]
pub struct BitVec {
    data: Vec<u64>,
    size: usize,
}

impl BitVec {
    /// Create a new `BitVec` with at least the specified number of bits
    pub fn new(size: usize) -> BitVec {
        // ceiling to the right number of u64s
        let num_ints = (size + STORAGE_BITS - 1) / STORAGE_BITS;
        let data = unsafe {
            let pointer = alloc_zeroed(Layout::from_size_align(num_ints * size_of::<usize>(), CACHE_LINE_WIDTH).unwrap());
            // TODO: freeing will supply a different alignment (the one of u64)
            // appearently this is not a problem, but it could be some day
            // so we probably should also do the dropping ourselves
            #[allow(clippy::cast_ptr_alignment)] // was actually allocated with greater alignment
            Vec::from_raw_parts(pointer as *mut u64, num_ints, num_ints)
        };

        BitVec { data, size }
    }

    /// Get individual bit value of position `index`.
    pub fn get(&self, index: usize) -> bool {
        assert!(index < self.size, "index: {} size: {}", index, self.size);
        // shifting a 1 bit to the right place and masking
        self.data[index / STORAGE_BITS] & (1 << (index % STORAGE_BITS)) != 0
    }

    /// Set individual bit value of position `index` to 1.
    pub fn set(&mut self, index: usize) {
        assert!(index < self.size, "index: {} size: {}", index, self.size);
        // shifting a 1 bit to the right place and then eighter set through | or negate and unset with &
        self.data[index / STORAGE_BITS] |= 1 << (index % STORAGE_BITS);
    }

    /// Set individual bit value of position `index` to 0.
    pub fn unset(&mut self, index: usize) {
        assert!(index < self.size, "index: {} size: {}", index, self.size);
        // shifting a 1 bit to the right place and then eighter set through | or negate and unset with &
        self.data[index / STORAGE_BITS] &= !(1 << (index % STORAGE_BITS));
    }

    /// Set all bits in the same 64bit int as the supplied index to zero.
    pub fn unset_all_around(&mut self, index: usize) {
        assert!(index < self.size, "index: {} size: {}", index, self.size);
        self.data[index / STORAGE_BITS] = 0;
    }

    /// Number of bits in the `BitVec`
    pub fn len(&self) -> usize {
        self.size
    }

    /// Are there no bits in this `BitVec`? Would be slightly pointless though.
    pub fn is_empty(&self) -> bool {
        self.size == 0
    }

    /// Zero all bits.
    pub fn clear(&mut self) {
        for val in &mut self.data {
            *val = 0;
        }
    }

    /// Set all bits to 1.
    pub fn set_all(&mut self) {
        for val in &mut self.data {
            *val = !0;
        }
    }

    /// The total number of bits set to 1.
    pub fn count_ones(&self) -> usize {
        self.data.iter().map(|v| v.count_ones() as usize).sum()
    }
}

impl DataBytes for BitVec {
    fn data_bytes(&self) -> &[u8] {
        self.data.data_bytes()
    }
}

impl DataBytesMut for BitVec {
    fn data_bytes_mut(&mut self) -> &mut [u8] {
        self.data.data_bytes_mut()
    }
}

impl Load for BitVec {
    fn new_with_bytes(num_bytes: usize) -> Self {
        BitVec::new(num_bytes * 8)
    }
}

/// A Bitarray with a clearlist for fast resetting.
/// Effective, when few bits (compared to the total number) will be set in each round before reset.
#[derive(Clone)]
pub struct FastClearBitVec {
    data: Vec<u64>,
    to_clear: Vec<usize>,
}

impl FastClearBitVec {
    /// Create a new `FastClearBitVec` with at least the specified number of bits
    pub fn new(size: usize) -> FastClearBitVec {
        // ceiling to the right number of u64s
        let num_ints = (size + STORAGE_BITS - 1) / STORAGE_BITS;
        let data = vec![0; num_ints];
        FastClearBitVec { data, to_clear: Vec::new() }
    }

    /// Get an individual bit.
    pub fn get(&self, index: usize) -> bool {
        // shifting a 1 bit to the right place and masking
        self.data[index / STORAGE_BITS] & (1 << (index % STORAGE_BITS)) != 0
    }

    /// Set an individual bit to 1.
    pub fn set(&mut self, index: usize) {
        if self.data[index / STORAGE_BITS] == 0 {
            self.to_clear.push(index / STORAGE_BITS);
        }
        // shifting a 1 bit to the right place and then eighter set through | or negate and unset with &
        self.data[index / STORAGE_BITS] |= 1 << (index % STORAGE_BITS);
    }

    /// Reset all bits to 0.
    pub fn clear(&mut self) {
        for &idx in &self.to_clear {
            self.data[idx] = 0;
        }
        self.to_clear.clear();
    }
}

/// the actual map data structure.
/// made up of the bitvec with one bit for each global id
/// and an vec containing prefix sums of the number of elements
/// for each global id in our id space we set the corresponding bit in the bitvector
/// the local id is then the number of bits set in the bitvector before the id itself
/// the prefix sum array is there so we do not need to always count everything before
/// but just the ones in the current cache line, since everything before is
/// already counted in the prefix sum. Conveniently couting ones in the ids cache line
/// is super fast. Since updates require us to update the prefixes we use the data structure
/// in two phases, first insert, and then after compile access.
#[derive(Debug)]
pub struct RankSelectMap {
    contained_keys_flags: BitVec,
    prefix_sum: Vec<usize>,
}

const BITS_PER_PREFIX: usize = CACHE_LINE_WIDTH * 8;
const INTS_PER_PREFIX: usize = BITS_PER_PREFIX / STORAGE_BITS;

impl RankSelectMap {
    /// Create a new `RankSelectMap` from a `BitVec`
    pub fn new(bit_vec: BitVec) -> RankSelectMap {
        let max_index = bit_vec.len();
        let mut this = RankSelectMap {
            contained_keys_flags: bit_vec,
            // the number of elements in the prefix vector is ceiled and one extra element
            // is added in the back containing the total number of elements
            prefix_sum: vec![0; (max_index + BITS_PER_PREFIX - 1) / BITS_PER_PREFIX + 1],
        };
        this.compile();
        this
    }

    /// Total number of entries in the map.
    pub fn len(&self) -> usize {
        match self.prefix_sum.last() {
            Some(&len) => len,
            None => 0,
        }
    }

    /// True if there are no entries in the map.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    fn compile(&mut self) {
        let mut previous = 0;
        // we start from one here, since the first prefix is always zero, and it saves some corner case handling
        for index in 1..self.prefix_sum.len() {
            self.prefix_sum[index] = self.bit_count_entire_range(index - 1) + previous;
            previous = self.prefix_sum[index];
        }
    }

    fn bit_count_entire_range(&self, range_index: usize) -> usize {
        // make sure to not go over the edge
        let range = (range_index * INTS_PER_PREFIX)..min((range_index + 1) * INTS_PER_PREFIX, self.contained_keys_flags.data.len());
        // count_ones will use POPCNT when -C target-cpu=native is set, so this should be crazy fast, since it's all in the cache
        // TODO investigate assembler
        self.contained_keys_flags.data[range].iter().map(|num| num.count_ones() as usize).sum()
    }

    /// Get the local id for global id `key`.
    /// Will panic of `key` is not in the map.
    pub fn at(&self, key: usize) -> usize {
        assert!(self.contained_keys_flags.get(key));
        self.prefix_sum[key / BITS_PER_PREFIX] + self.bit_count_partial_range(key)
    }

    /// Get the local id for global id `key` or the next lower id if key is not in the map.
    /// Will underflow when there is no lower next entry.
    pub fn at_or_next_lower(&self, key: usize) -> usize {
        let value = self.prefix_sum[key / BITS_PER_PREFIX] + self.bit_count_partial_range(key);
        if self.contained_keys_flags.get(key) {
            value
        } else {
            value - 1
        }
    }

    /// Get the local id for global id `key` if it exists.
    pub fn get(&self, key: usize) -> Option<usize> {
        if key < self.contained_keys_flags.size && self.contained_keys_flags.get(key) {
            Some(self.prefix_sum[key / BITS_PER_PREFIX] + self.bit_count_partial_range(key))
        } else {
            None
        }
    }

    fn bit_count_partial_range(&self, key: usize) -> usize {
        let index = key / STORAGE_BITS; // the index of the number containing the bit
        let num = (self.contained_keys_flags.data[index] % (1 << (key % STORAGE_BITS))).count_ones() as usize; // num ones in the number

        let range = ((index / INTS_PER_PREFIX) * INTS_PER_PREFIX)..index; // the range over the numbers before our number
        let sum: usize = self.contained_keys_flags.data[range].iter().map(|num| num.count_ones() as usize).sum(); // num ones in that range
        sum + num
    }
}

impl DataBytes for RankSelectMap {
    fn data_bytes(&self) -> &[u8] {
        self.contained_keys_flags.data_bytes()
    }
}

const GROUPED_LOCALS: usize = 256;

/// Wrapper around `RankSelectMap` if the reverse mapping (local to global) is required.
/// Implements `Deref` to `RankSelectMap`, so the original map is still ergonomically accessible
#[derive(Debug)]
pub struct InvertableRankSelectMap {
    map: RankSelectMap,
    blocks: Vec<usize>,
}

impl InvertableRankSelectMap {
    /// Create wrapper around `RankSelectMap` for inversion.
    pub fn new(map: RankSelectMap) -> InvertableRankSelectMap {
        let num_groups = 1 + (map.len() + GROUPED_LOCALS - 1) / GROUPED_LOCALS;
        let mut blocks = vec![0; num_groups];
        *blocks.last_mut().unwrap() = map.prefix_sum.len() - 1;

        let mut value = 0;
        for (block_index, prefix_sums) in map.prefix_sum.windows(2).enumerate() {
            while prefix_sums[0] <= value && value < prefix_sums[1] {
                blocks[value / GROUPED_LOCALS] = block_index;
                value += GROUPED_LOCALS;
            }
        }

        InvertableRankSelectMap { map, blocks }
    }

    /// Get global id for local one.
    pub fn inverse(&self, value: usize) -> usize {
        debug_assert!(value < self.map.len());
        let group = value / GROUPED_LOCALS;
        let block_index = match self.map.prefix_sum[self.blocks[group]..=self.blocks[group + 1]].binary_search(&value) {
            Ok(block_index) => block_index,
            Err(block_index) => block_index - 1,
        } + self.blocks[group];

        let block_local_rank = value - self.map.prefix_sum[block_index];
        let block_local_index = self.block_local_index(block_index, block_local_rank);

        block_index * BITS_PER_PREFIX + block_local_index
    }

    fn block_local_index(&self, block: usize, rank: usize) -> usize {
        let data_begin = block * INTS_PER_PREFIX;
        let data_end = min((block + 1) * INTS_PER_PREFIX, self.map.contained_keys_flags.data.len());
        let data = &self.map.contained_keys_flags.data[data_begin..data_end];

        let mut popcount_sum = 0;
        let mut num_skipped_bits = 0;
        let mut iter = data.iter();
        let bits = loop {
            let bits = iter.next().unwrap();
            let popcount = bits.count_ones() as usize;

            if popcount_sum + popcount > rank {
                break bits;
            }
            popcount_sum += popcount;
            num_skipped_bits += STORAGE_BITS;
        };

        num_skipped_bits + InvertableRankSelectMap::int_local_index(*bits, rank - popcount_sum)
    }

    fn int_local_index(int: u64, rank: usize) -> usize {
        debug_assert!(rank < int.count_ones() as usize);
        let mut word = int;
        let mut adjusted_rank = rank;
        let mut shifted = 0;

        debug_assert!(adjusted_rank < 64);
        let lower_ones = (word as u32).count_ones() as usize;
        if lower_ones <= adjusted_rank {
            word >>= 32;
            adjusted_rank -= lower_ones;
            shifted += 32;
        }

        debug_assert!(adjusted_rank < 32);
        let lower_ones = (word as u16).count_ones() as usize;
        if lower_ones <= adjusted_rank {
            word >>= 16;
            adjusted_rank -= lower_ones;
            shifted += 16;
        }

        debug_assert!(adjusted_rank < 16);
        let lower_ones = (word as u8).count_ones() as usize;
        if lower_ones <= adjusted_rank {
            word >>= 8;
            adjusted_rank -= lower_ones;
            shifted += 8;
        }

        debug_assert!(adjusted_rank < 8);
        while adjusted_rank > 0 {
            word &= word - 1;
            adjusted_rank -= 1;
        }

        debug_assert_ne!(word.trailing_zeros(), 64);
        shifted + word.trailing_zeros() as usize
    }
}

impl Deref for InvertableRankSelectMap {
    type Target = RankSelectMap;

    fn deref(&self) -> &Self::Target {
        &self.map
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_and_fill_map() -> RankSelectMap {
        let mut bits = BitVec::new(1000);
        bits.set(31);
        bits.set(52);
        bits.set(2);
        bits.set(130);
        bits.set(0);
        bits.set(149);
        bits.set(999);
        RankSelectMap::new(bits)
    }

    #[test]
    fn at_or_next_lower() {
        let map = create_and_fill_map();
        assert_eq!(map.at_or_next_lower(0), 0);
        assert_eq!(map.at_or_next_lower(1), 0);
        assert_eq!(map.at_or_next_lower(2), 1);
        assert_eq!(map.at_or_next_lower(3), 1);
        assert_eq!(map.at_or_next_lower(52), 3);
    }

    #[test]
    fn test_at() {
        let map = create_and_fill_map();
        assert_eq!(map.at(0), 0);
        assert_eq!(map.at(2), 1);
        assert_eq!(map.at(52), 3);
        assert_eq!(map.at(130), 4);
        assert_eq!(map.at(149), 5);
        assert_eq!(map.at(999), 6);
    }

    #[test]
    fn test_get() {
        let map = create_and_fill_map();
        assert_eq!(map.get(31), Some(2));
        assert_eq!(map.get(32), None);
    }

    #[test]
    fn test_len() {
        let map = create_and_fill_map();
        assert_eq!(map.len(), 7);
    }

    #[test]
    fn test_bug() {
        let mut bits = BitVec::new(1000);
        bits.set(0);
        bits.set(64);
        let map = RankSelectMap::new(bits);
        assert_eq!(map.at(0), 0);
        assert_eq!(map.at(64), 1);
    }

    #[test]
    fn test_inverse() {
        let map = InvertableRankSelectMap::new(create_and_fill_map());
        assert_eq!(map.at(0), 0);
        assert_eq!(map.inverse(0), 0);
        assert_eq!(map.inverse(1), 2);
        assert_eq!(map.inverse(3), 52);
        assert_eq!(map.inverse(4), 130);
        assert_eq!(map.inverse(5), 149);
        assert_eq!(map.inverse(6), 999);
    }

    #[test]
    fn test_with_bigger_map() {
        let mut bits = BitVec::new(1 << 16);
        for i in 0..(1 << 16) {
            if i % 7 == 3 {
                bits.set(i);
            }
        }
        let map = InvertableRankSelectMap::new(RankSelectMap::new(bits));

        let mut counter = 0;
        for i in 0..(1 << 16) {
            if i % 7 == 3 {
                assert_eq!(map.at(i), counter);
                assert_eq!(map.inverse(counter), i);
                counter += 1;
            }
        }
    }
}
