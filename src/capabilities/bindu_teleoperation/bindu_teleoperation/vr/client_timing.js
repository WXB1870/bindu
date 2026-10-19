/* Add local timing to the existing MessagePack envelope. No extra connection,
 * pose generation, heartbeat, or control timestamp changes. */
(() => {
  'use strict';
  const encoder = new TextEncoder(), sockets = new WeakMap();
  let loopMax = 0, rafMax = 0, xrMax = 0, longMax = 0, longCount = 0;
  let lastRaf = null, lastXr = null, xrVisibility = 'unavailable';
  let due = performance.now() + 50;
  setInterval(() => {
    const now = performance.now();
    loopMax = Math.max(loopMax, now - due); due = now + 50;
  }, 50);
  const raf = window.requestAnimationFrame;
  window.requestAnimationFrame = function(callback) {
    return raf.call(this, (time, frame) => {
      const now = performance.now();
      if (lastRaf !== null) rafMax = Math.max(rafMax, now - lastRaf);
      lastRaf = now; callback(time, frame);
    });
  };
  if (window.XRSession) {
    const xrRaf = XRSession.prototype.requestAnimationFrame;
    XRSession.prototype.requestAnimationFrame = function(callback) {
      return xrRaf.call(this, (time, frame) => {
        const now = performance.now();
        if (lastXr !== null) xrMax = Math.max(xrMax, now - lastXr);
        lastXr = now; xrVisibility = this.visibilityState;
        callback(time, frame);
      });
    };
  }
  try {
    new PerformanceObserver(list => {
      for (const task of list.getEntries()) {
        longCount++; longMax = Math.max(longMax, task.duration);
      }
    }).observe({entryTypes: ['longtask']});
  } catch (_) { /* Unsupported on some headset browsers. */ }

  // Only encode our small, bounded metadata. The original map body stays intact.
  function pack(value) {
    if (typeof value === 'string') {
      const b = encoder.encode(value);
      return [...(b.length < 32 ? [0xa0 + b.length] : [0xd9, b.length]), ...b];
    }
    if (typeof value === 'number') {
      const b = new Uint8Array(9); b[0] = 0xcb;
      new DataView(b.buffer).setFloat64(1, value); return [...b];
    }
    if (value === null) return [0xc0];
    const entries = Object.entries(value);
    return [0x80 + entries.length, ...entries.flatMap(([k, v]) => [...pack(k), ...pack(v)])];
  }
  const send = WebSocket.prototype.send;
  WebSocket.prototype.send = function(data) {
    let outgoing = data;
    try {
      const b = data instanceof ArrayBuffer ? new Uint8Array(data) :
        ArrayBuffer.isView(data) ? new Uint8Array(data.buffer, data.byteOffset, data.byteLength) : null;
      if (b && b.length && b.length < 16 * 1024 * 1024) {
        let count, offset;
        if ((b[0] & 0xf0) === 0x80) { count = b[0] & 15; offset = 1; }
        else if (b[0] === 0xde && b.length >= 3) { count = b[1] * 256 + b[2]; offset = 3; }
        else if (b[0] === 0xdf && b.length >= 5) {
          count = new DataView(b.buffer, b.byteOffset).getUint32(1); offset = 5;
        }
        if (count !== undefined && count < 65535) {
          const now = performance.now(), state = sockets.get(this) || {seq: 0, last: null};
          const timing = {version: 1, seq: ++state.seq, perf_ms: now, wall_ms: Date.now(),
            send_interval_ms: state.last === null ? null : now - state.last,
            buffered_bytes: this.bufferedAmount, loop_max_ms: loopMax, raf_max_ms: rafMax,
            xr_max_ms: xrMax, long_task_max_ms: longMax, long_task_count: longCount,
            page_visibility: document.visibilityState, xr_visibility: xrVisibility};
          const extra = new Uint8Array([...pack('bindu_timing'), ...pack(timing)]);
          const header = count + 1 < 16 ? [0x80 + count + 1] : [0xde, (count + 1) >> 8, (count + 1) & 255];
          outgoing = new Uint8Array(header.length + b.length - offset + extra.length);
          outgoing.set(header); outgoing.set(b.subarray(offset), header.length);
          outgoing.set(extra, header.length + b.length - offset);
          state.last = now; sockets.set(this, state);
          loopMax = rafMax = xrMax = longMax = 0;
        }
      }
    } catch (_) { /* Diagnostics must never prevent the original send. */ }
    return send.call(this, outgoing);
  };
})();
