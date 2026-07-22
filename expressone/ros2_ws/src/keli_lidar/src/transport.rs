use crate::protocol::constants;
use log::{debug, error, info, warn};
use std::net::UdpSocket;
use std::time::Duration;

/// UDP transport for the Keli LiDAR.
///
/// Handles:
/// - Sending commands (start/stop stream)
/// - Receiving UDP datagrams
/// - Sub-packet reassembly (2-4 sub-packets per frame)
/// - Checksum verification
pub struct LidarTransport {
    socket: Option<UdpSocket>,
    remote_addr: String,
    port: u16,
    timeout_secs: u32,
    /// Sub-packet storage for reassembly
    sub_packets: [Option<SubPacket>; constants::CMD_FRAME_MAX_SUB_PKG_NUM],
    /// Reassembled data buffer
    store_buffer: Vec<u8>,
    /// Frame sequence tracking
    last_frame_total_index: u32,
    /// Loop iteration counter (for skip logic)
    iteration_count: u64,
    /// Skip every N frames (0 = no skip)
    skip: u32,
}

#[derive(Clone)]
struct SubPacket {
    total_index: u32,
    sub_pkg_num: u8,
    sub_pkg_index: u8,
    raw_data_len: usize,
    sens_data: Vec<u8>,
}

impl LidarTransport {
    pub fn new(host: &str, port: u16, timeout_secs: u32) -> Self {
        LidarTransport {
            socket: None,
            remote_addr: host.to_string(),
            port,
            timeout_secs,
            sub_packets: Default::default(),
            store_buffer: vec![0u8; constants::RECV_BUFFER_SIZE],
            last_frame_total_index: 0xFFFFFFFF,
            iteration_count: 0,
            skip: 0,
        }
    }

    pub fn set_skip(&mut self, skip: u32) {
        self.skip = skip;
    }

    /// Create and bind the UDP socket.
    pub fn connect(&mut self) -> Result<(), String> {
        let socket = UdpSocket::bind("0.0.0.0:0")
            .map_err(|e| format!("bind failed: {}", e))?;

        socket
            .set_read_timeout(Some(Duration::from_secs(self.timeout_secs as u64)))
            .map_err(|e| format!("set timeout failed: {}", e))?;

        info!(
            "UDP socket bound, target {}:{}",
            self.remote_addr, self.port
        );
        self.socket = Some(socket);
        self.last_frame_total_index = 0xFFFFFFFF;
        self.iteration_count = 0;
        Ok(())
    }

    /// Send an 8-byte command to the LiDAR.
    pub fn send_command(&self, cmd: &[u8; 8]) -> Result<(), String> {
        let sock = self.socket.as_ref().ok_or("socket not open")?;
        let sent = sock
            .send_to(cmd, format!("{}:{}", self.remote_addr, self.port))
            .map_err(|e| format!("sendto failed: {}", e))?;
        if sent != constants::CMD_SIZE {
            return Err(format!("sent {} bytes, expected {}", sent, constants::CMD_SIZE));
        }
        Ok(())
    }

    /// Read one datagram, reassemble sub-packets, and if a complete frame
    /// is available, return the reassembled payload.
    ///
    /// Returns `Ok(Some(payload))` when a full frame is ready.
    /// Returns `Ok(None)` when data is incomplete (keep going).
    /// Returns `Err("...")` on unrecoverable errors.
    pub fn read_and_reassemble(&mut self) -> Result<Option<Vec<u8>>, String> {
        let sock = self.socket.as_ref().ok_or("socket not open")?;
        let mut recv_buf = [0u8; constants::RECV_BUFFER_SIZE];

        // 1. Receive datagram
        let data_len = match sock.recv(&mut recv_buf) {
            Ok(n) => n,
            Err(e) => {
                // Timeout is expected — return None to continue looping
                return Err(format!("recv error: {}", e));
            }
        };

        // 2. Check frame header (FA 5A A5 AA)
        if recv_buf[..4] != constants::FRAME_HEADER {
            warn!("command header error");
            return Ok(None);
        }

        // 3. Parse sub-packet header fields
        let raw_data_len = ((recv_buf[constants::HDR_LENGTH_H] as u16) << 8
            | recv_buf[constants::HDR_LENGTH_L] as u16)
            .saturating_sub((constants::HDR_DATA_START - constants::HDR_CHECKSUM) as u16)
            as usize;

        let frame_total_index = ((recv_buf[constants::HDR_TOTAL_IDX_H] as u32) << 8)
            | recv_buf[constants::HDR_TOTAL_IDX_L] as u32;

        let sub_pkg_num = recv_buf[constants::HDR_SUB_PKG_NUM];
        let sub_pkg_index = recv_buf[constants::HDR_SUB_INDEX];

        // 4. Checksum verification (of [HDR_TYPE .. HDR_TYPE+rawLength+4])
        let check_sum_len = raw_data_len + (constants::HDR_DATA_START - constants::HDR_TYPE);
        let calc_check: u8 = recv_buf[constants::HDR_TYPE..constants::HDR_TYPE + check_sum_len]
            .iter()
            .fold(0u8, |a, &b| a.wrapping_add(b));
        let wire_check = recv_buf[constants::HDR_CHECKSUM];
        if calc_check != wire_check {
            warn!("checksum error: calc=0x{:02X} wire=0x{:02X}", calc_check, wire_check);
            return Ok(None);
        }

        // 5. Validate data length
        let expected_pkt_len = raw_data_len + constants::HDR_DATA_START;
        if data_len != expected_pkt_len || data_len > constants::CMD_FRAME_MAX_LEN {
            warn!(
                "dataLength error: got={} expected={} max={}",
                data_len, expected_pkt_len, constants::CMD_FRAME_MAX_LEN
            );
            return Ok(None);
        }

        // 6. Validate sub-packet indices
        if sub_pkg_num > constants::CMD_FRAME_MAX_SUB_PKG_NUM as u8
            || sub_pkg_num < constants::CMD_FRAME_MIN_SUB_PKG_NUM as u8
            || sub_pkg_index as usize >= constants::CMD_FRAME_MAX_SUB_PKG_NUM
        {
            warn!("invalid sub-pkg: num={} index={}", sub_pkg_num, sub_pkg_index);
            return Ok(None);
        }

        // 7. Store sub-packet
        let sub_idx = sub_pkg_index as usize;
        self.sub_packets[sub_idx] = Some(SubPacket {
            total_index: frame_total_index,
            sub_pkg_num,
            sub_pkg_index,
            raw_data_len,
            sens_data: recv_buf[constants::HDR_DATA_START..constants::HDR_DATA_START + raw_data_len]
                .to_vec(),
        });

        // 8. Check if all sub-packets for this frame are received
        let mut all_received = true;
        for n in 0..(sub_pkg_num as usize) {
            match &self.sub_packets[n] {
                Some(sp) => {
                    if sp.total_index != frame_total_index {
                        all_received = false;
                        break;
                    }
                    // Verify sequential indices
                    if n > 0 {
                        if let Some(prev) = &self.sub_packets[n - 1] {
                            if prev.sub_pkg_index + 1 != sp.sub_pkg_index {
                                all_received = false;
                                break;
                            }
                        }
                    }
                }
                None => {
                    all_received = false;
                    break;
                }
            }
        }

        if !all_received {
            // Still waiting for more sub-packets
            return Ok(None);
        }

        // 9. Reassemble into store buffer
        let mut total_data_len = 0usize;
        for n in 0..(sub_pkg_num as usize) {
            if let Some(ref sp) = self.sub_packets[n] {
                let end = total_data_len + sp.raw_data_len;
                if end > self.store_buffer.len() {
                    warn!("store buffer overflow");
                    self.clear_sub_packets();
                    return Ok(None);
                }
                self.store_buffer[total_data_len..end].copy_from_slice(&sp.sens_data);
                total_data_len += sp.raw_data_len;
            }
        }

        // 10. Frame index ordering check
        if frame_total_index != self.last_frame_total_index.wrapping_add(1)
            && frame_total_index != 0
        {
            debug!(
                "frame out-of-order: current={} last={}",
                frame_total_index, self.last_frame_total_index
            );
        }
        self.last_frame_total_index = frame_total_index;

        self.clear_sub_packets();

        // 11. Validate total data length matches FRAME_LENGTH
        if total_data_len < constants::FRAME_LENGTH
            || total_data_len % constants::FRAME_LENGTH != 0
        {
            error!("invalid total data length: {}", total_data_len);
            self.clear_reassembly();
            return Ok(None);
        }

        // 12. Skip logic
        self.iteration_count += 1;
        if self.skip > 0 && (self.iteration_count % (self.skip as u64 + 1) != 0) {
            debug!("skip frame");
            return Ok(None);
        }

        // Success — return the reassembled data
        let result = self.store_buffer[..total_data_len].to_vec();
        self.clear_reassembly();
        Ok(Some(result))
    }

    fn clear_sub_packets(&mut self) {
        self.sub_packets = Default::default();
    }

    fn clear_reassembly(&mut self) {
        self.store_buffer.fill(0);
    }

    /// Close the socket.
    pub fn disconnect(&mut self) {
        self.socket = None;
        self.clear_sub_packets();
        self.clear_reassembly();
    }
}

impl Drop for LidarTransport {
    fn drop(&mut self) {
        let _ = self.send_command(&constants::CMD_STOP_STREAM);
        self.disconnect();
    }
}
