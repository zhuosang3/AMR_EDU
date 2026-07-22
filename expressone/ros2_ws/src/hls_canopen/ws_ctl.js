#!/usr/bin/env node
// ws_ctl.js — enable motors then drive forward at 0.3 m/s forever
// Ctrl+C to stop

const WS_URL = 'ws://127.0.0.1:9090';
const ws = new WebSocket(WS_URL);

ws.addEventListener('open', () => {
  process.stderr.write('[ws_ctl] Connected, enabling motors...\n');

  // Wait 3s for auto-enable to settle, then confirm
  setTimeout(() => {
    ws.send(JSON.stringify({ cmd: 'enable' }));
    process.stderr.write('[ws_ctl] Motors enabled, driving at 0.3 m/s\n');

    // Send velocity every 500ms to beat the 1s watchdog
    const iv = setInterval(() => {
      ws.send(JSON.stringify({ linear: 0.3 }));
    }, 500);

    // After 10s, disable and exit
    setTimeout(() => {
      clearInterval(iv);
      ws.send(JSON.stringify({ cmd: 'disable' }));
      process.stderr.write('[ws_ctl] Disabled after 10s, exiting\n');
      setTimeout(() => ws.close(), 500);
    }, 10000);
  }, 300);
});

ws.addEventListener('message', () => {
  // Suppress odom
});

ws.addEventListener('error', (err) => {
  process.stderr.write(`Error: ${err.message}\n`);
  process.exit(1);
});

ws.addEventListener('close', () => {
  process.stderr.write('[ws_ctl] Disconnected\n');
  process.exit(0);
});
