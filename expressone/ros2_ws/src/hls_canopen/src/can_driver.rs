/// CAN bus abstraction layer.
///
/// `CanBus` trait provides a common interface for real (socketcan) and mock CAN operations,
/// enabling full testing without hardware.
use anyhow::Result;
use socketcan::{CanFrame, CanSocket, Socket};
use std::collections::VecDeque;
use std::time::Duration;

// ── Trait ────────────────────────────────────────────────────────────────

/// Abstract CAN bus interface: send and receive CAN frames.
pub trait CanBus {
    /// Write a CAN frame to the bus.
    fn write_frame(&mut self, frame: &CanFrame) -> Result<()>;

    /// Read a CAN frame from the bus. Returns `None` if no frame available.
    fn read_frame(&mut self) -> Result<Option<CanFrame>>;
}

// ── Real: SocketCAN ──────────────────────────────────────────────────────

/// Real CAN bus using Linux SocketCAN (`socketcan` crate).
pub struct SocketCanBus {
    socket: CanSocket,
}

impl SocketCanBus {
    /// Open a blocking CAN socket on the given interface (e.g., "can0").
    /// Sets a 200 ms read timeout so transient bus errors don't hang the process.
    pub fn open(ifname: &str) -> Result<Self> {
        let socket = CanSocket::open(ifname)?;
        socket.set_read_timeout(Duration::from_millis(200))?;
        println!("✓ CAN socket opened on {} (read timeout=200ms)", ifname);
        Ok(Self { socket })
    }

    /// Reconnect after a CAN bus disconnection (e.g. USB-CAN adapter unplugged).
    ///
    /// Drops the old (dead) socket, waits for the interface to reappear and come UP,
    /// reconfigures it at the given bitrate, then opens a fresh socket.
    ///
    /// Returns an error if the interface does not recover within `timeout`.
    pub fn reconnect(&mut self, ifname: &str, bitrate: u32, timeout: Duration) -> Result<()> {
        println!("[can] Reconnecting {} (timeout={:?})...", ifname, timeout);

        // 1. Drop the dead socket — opening a new one below will replace it
        let start = std::time::Instant::now();

        // 2. Wait for interface to reappear and be UP
        loop {
            if crate::can_setup::is_can_up(ifname) {
                break;
            }
            if start.elapsed() > timeout {
                anyhow::bail!(
                    "CAN interface {} did not recover within {:?}",
                    ifname,
                    timeout
                );
            }
            std::thread::sleep(Duration::from_millis(500));
        }

        // 3. Reconfigure (setup_can skips if already UP)
        crate::can_setup::setup_can(ifname, bitrate)?;

        // 4. Open fresh socket (old one is dropped by assignment)
        self.socket = CanSocket::open(ifname)?;
        self.socket.set_read_timeout(Duration::from_millis(200))?;

        println!("[can] ✓ Reconnected to {}", ifname);
        Ok(())
    }
}

impl CanBus for SocketCanBus {
    fn write_frame(&mut self, frame: &CanFrame) -> Result<()> {
        self.socket.write_frame(frame)?;
        Ok(())
    }

    fn read_frame(&mut self) -> Result<Option<CanFrame>> {
        // Blocking read — returns one frame
        let frame = self.socket.read_frame()?;
        Ok(Some(frame))
    }
}

// ── Mock: for testing ───────────────────────────────────────────────────

/// Mock CAN bus with pre-configured response queue.
///
/// Write operations are recorded in `sent`; read operations drain from
/// `responses`. Use `push_response()` to set up expected SDO acknowledgments,
/// status word responses, etc.
pub struct MockCanBus {
    /// Frames waiting to be read (FIFO).
    pub responses: VecDeque<CanFrame>,

    /// All frames that were written (for test assertions).
    pub sent: Vec<CanFrame>,
}

impl MockCanBus {
    pub fn new() -> Self {
        Self {
            responses: VecDeque::new(),
            sent: Vec::new(),
        }
    }

    /// Enqueue a frame to be returned by the next `read_frame()` call.
    pub fn push_response(&mut self, frame: CanFrame) {
        self.responses.push_back(frame);
    }
}

impl CanBus for MockCanBus {
    fn write_frame(&mut self, frame: &CanFrame) -> Result<()> {
        self.sent.push(*frame);
        Ok(())
    }

    fn read_frame(&mut self) -> Result<Option<CanFrame>> {
        Ok(self.responses.pop_front())
    }
}
