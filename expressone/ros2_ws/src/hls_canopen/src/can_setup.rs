/// Configure and bring up a CAN interface.
/// Uses `ip link` (iproute2) — requires CAP_NET_ADMIN or root.
///
/// If the interface is already UP (e.g. configured by systemd-networkd),
/// this function skips all privileged operations and returns Ok immediately.
use anyhow::{Context, Result};
use std::fs;
use std::process::Command;

pub fn setup_can(ifname: &str, bitrate: u32) -> Result<()> {
    // Check if interface is already UP (e.g. systemd-networkd handled it)
    if is_can_up(ifname) {
        println!(
            "✓ CAN interface {} already UP (skipping config)",
            ifname
        );
        return Ok(());
    }

    // Bring it down first (ignore error if already down or doesn't exist)
    let _ = Command::new("ip")
        .args(["link", "set", "down", ifname])
        .status();

    // Set type to CAN with bitrate
    let output = Command::new("ip")
        .args([
            "link",
            "set",
            ifname,
            "type",
            "can",
            "bitrate",
            &bitrate.to_string(),
        ])
        .output()
        .with_context(|| {
            format!(
                "Failed to execute 'ip link set {} type can'",
                ifname
            )
        })?;

    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr);
        anyhow::bail!(
            "Failed to configure {} (bitrate={}): {}",
            ifname,
            bitrate,
            stderr.trim()
        );
    }

    // Bring it up
    let output = Command::new("ip")
        .args(["link", "set", "up", ifname])
        .output()
        .with_context(|| {
            format!("Failed to execute 'ip link set up {}'", ifname)
        })?;

    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr);
        anyhow::bail!(
            "Failed to bring up {}: {}",
            ifname,
            stderr.trim()
        );
    }

    println!(
        "✓ CAN interface {} configured at {} bps",
        ifname, bitrate
    );
    Ok(())
}

/// Check if a CAN interface is already UP by reading /sys/class/net/{ifname}/operstate.
pub(crate) fn is_can_up(ifname: &str) -> bool {
    let path = format!("/sys/class/net/{}/operstate", ifname);
    fs::read_to_string(&path)
        .map(|s| s.trim() == "up")
        .unwrap_or(false)
}
