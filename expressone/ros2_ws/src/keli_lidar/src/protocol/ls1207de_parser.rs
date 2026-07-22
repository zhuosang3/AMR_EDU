use crate::protocol::frame::SensFrame;
use r2r::sensor_msgs::msg::LaserScan;
use std::f64::consts::PI;

/// Configuration passed into the parser.
#[derive(Debug, Clone)]
pub struct ParserConfig {
    pub frame_id: String,
    pub range_min: f64,
    pub range_max: f64,
    pub time_increment: f64,
    pub min_ang: f64,
    pub max_ang: f64,
    pub intensity: bool,
    pub time_offset: f64,
    pub debug_mode: bool,
}

impl Default for ParserConfig {
    fn default() -> Self {
        ParserConfig {
            frame_id: "laser".to_string(),
            range_min: 0.05,
            range_max: 10.0,
            time_increment: 0.000040,
            min_ang: -std::f64::consts::PI,
            max_ang: std::f64::consts::PI,
            intensity: true,
            time_offset: 0.0,
            debug_mode: false,
        }
    }
}

/// Parse raw reassembled frame data into a `LaserScan` message.
///
/// This is the Rust translation of `CSDKeliLs1207DEParser::Parse()`.
/// `data` is the full reassembled payload (after sub-packet reassembly).
/// `config` controls options.
///
/// Returns `Some(LaserScan)` on success, `None` on parse failure.
pub fn parse_scan(data: &[u8], config: &ParserConfig) -> Option<LaserScan> {
    let sens_frame = SensFrame::from_buf(data)?;
    let data_count = sens_frame.data_count();

    // --- Time / angle setup (same magic numbers as C++) ---
    // scanning_freq = 1000 / 43 * 100 = 2325 (in 0.01 Hz)
    let scanning_freq = (1000u32 / 43) * 100;
    let scan_time = 1.0 / (scanning_freq as f64 / 100.0);

    let time_increment = config.time_increment;

    // Starting angle = 0xFFF92230 (Sick TiM legacy)
    let starting_angle: i32 = 0xFFF92230u32 as i32; // -28112 → decoded as signed
    let angle_min = (starting_angle as f64 / 10000.0) / 180.0 * PI - PI / 2.0;

    // Angular step width = 0xD05 (3333 → 0.3333°)
    let angular_step_width: u16 = 0xD05;
    let angle_increment = (angular_step_width as f64 / 10000.0) / 180.0 * PI;

    let angle_max = angle_min + (data_count.saturating_sub(1) as f64) * angle_increment;

    // Clip angle range to [config.min_ang, config.max_ang]
    let mut clipped_angle_min = angle_min;
    let mut clipped_angle_max = angle_max;
    let mut index_min = 0usize;
    let mut index_max = data_count.saturating_sub(1);

    while (index_min + 1) < data_count && (clipped_angle_min + angle_increment) < config.min_ang {
        clipped_angle_min += angle_increment;
        index_min += 1;
    }

    while index_max > 0 && (clipped_angle_max - angle_increment) > config.max_ang {
        clipped_angle_max -= angle_increment;
        index_max = index_max.saturating_sub(1);
    }

    if index_min > index_max {
        return None;
    }

    let num_points = index_max - index_min + 1;

    // Build ranges (f32, per ROS2 msg type)
    let mut ranges = Vec::with_capacity(num_points);
    let mut intensities = Vec::new();

    let mut check_all = 0u32;
    let mut check_fault = 0u32;
    let mut check_fault2 = 0u32;

    let has_intensity = config.intensity && sens_frame.has_intensity(data.len());

    for j in index_min..=index_max {
        check_all += 1;

        let range_raw = sens_frame.get_range(j);
        let meter_value = range_raw as f64 / 1000.0;

        if range_raw == 1 {
            check_fault += 1;
        }

        if meter_value > config.range_min && meter_value < config.range_max {
            ranges.push(meter_value as f32);
        } else {
            check_fault2 += 1;
            ranges.push(f32::INFINITY);
        }

        // Intensity
        if has_intensity {
            let raw_intensity = sens_frame.get_intensity(j);
            let intensity_val = normalize_intensity(raw_intensity) as f32;
            intensities.push(intensity_val);
        }
    }

    // If ALL points are fault or out-of-range, signal bad frame
    if check_all == check_fault || check_all == check_fault2 {
        return None;
    }

    // Build LaserScan message
    let mut msg = LaserScan::default();

    msg.header.frame_id = config.frame_id.clone();
    msg.angle_min = clipped_angle_min as f32;
    msg.angle_max = clipped_angle_max as f32;
    msg.angle_increment = angle_increment as f32;
    msg.time_increment = time_increment as f32;
    msg.scan_time = scan_time as f32;
    msg.range_min = config.range_min as f32;
    msg.range_max = config.range_max as f32;
    msg.ranges = ranges;
    if has_intensity {
        msg.intensities = intensities;
    }

    Some(msg)
}

/// Normalize intensity values (same mapping as C++).
fn normalize_intensity(raw: u16) -> f64 {
    if raw > 55000 {
        600.0
    } else if raw > 5000 {
        200.0 + (raw as f64 - 5000.0) / 1200.0
    } else {
        raw as f64 / 25.0
    }
}
