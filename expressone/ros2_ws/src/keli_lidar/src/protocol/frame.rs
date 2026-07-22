

use std::io::{Cursor, Read};

/// A parsed frame, consisting of raw uint16_t measurements.
///
/// The C++ driver uses a placement-new trick: it creates a `SensData`
/// struct in a separate buffer, hardcodes `range_start=0, range_end=810`,
/// then reads data from the ORIGINAL input buffer as a flat uint16_t array.
/// There is NO header structure in the wire data itself — just raw
/// big-endian 16-bit values.
///
/// Given `FRAME_LENGTH = 1622` bytes per sub-packet group, one logical
/// frame without intensity = 1622/2 = 811 points.
/// With intensity = 1622 points (811 ranges + 811 intensities).
pub struct SensFrame {
    pub data: Vec<u16>,
}

impl SensFrame {
    /// Parse raw big-endian uint16_t data from a byte buffer.
    pub fn from_buf(buf: &[u8]) -> Option<SensFrame> {
        if buf.len() < 2 {
            return None;
        }

        let mut cursor = Cursor::new(buf);
        let mut data = Vec::with_capacity(buf.len() / 2);

        while cursor.position() < buf.len() as u64 {
            let mut pair = [0u8; 2];
            if cursor.read_exact(&mut pair).is_err() {
                break;
            }
            let value = u16::from_be_bytes(pair);
            data.push(value);
        }

        Some(SensFrame { data })
    }

    /// Expected number of range points (hardcoded 811, mirroring C++).
    /// This matches `range_end(810) - range_start(0) + 1`.
    pub const RANGE_COUNT: usize = 811;

    /// Total number of data points (range + optional intensity).
    pub fn data_count(&self) -> usize {
        Self::RANGE_COUNT
    }

    /// Does this frame include intensity data?
    /// Each range point = 2 bytes → RANGE_COUNT*2 = 1622 bytes for range only.
    /// With intensity = RANGE_COUNT*4 = 3244 bytes.
    ///
    /// The check: if `buf.len() == RANGE_COUNT * 4`, we have intensity.
    pub fn has_intensity(&self, total_bytes: usize) -> bool {
        total_bytes >= Self::RANGE_COUNT * 4
    }

    /// Get a single range value (in mm).
    pub fn get_range(&self, index: usize) -> u16 {
        if index < self.data.len() {
            self.data[index]
        } else {
            0
        }
    }

    /// Get intensity value at `index`. Intensity data follows range data.
    pub fn get_intensity(&self, index: usize) -> u16 {
        let offset = self.data_count() + index;
        if offset < self.data.len() {
            self.data[offset]
        } else {
            0
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_frame_from_empty() {
        assert!(SensFrame::from_buf(&[]).is_none());
    }

    #[test]
    fn test_frame_one_value() {
        let buf = vec![0x00, 0x64]; // 100 in big-endian
        let frame = SensFrame::from_buf(&buf).unwrap();
        assert_eq!(frame.data, vec![100]);
    }

    #[test]
    fn test_frame_multi_value() {
        let buf = vec![0x00, 0x64, 0x01, 0x2C]; // 100, 300
        let frame = SensFrame::from_buf(&buf).unwrap();
        assert_eq!(frame.data, vec![100, 300]);
    }

    #[test]
    fn test_range_count() {
        assert_eq!(SensFrame::RANGE_COUNT, 811);
    }

    #[test]
    fn test_has_intensity() {
        let frame = SensFrame::from_buf(&[0u8; 3244]).unwrap();
        assert!(frame.has_intensity(3244));
    }
}
