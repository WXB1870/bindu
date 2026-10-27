/* Add local timing to the existing MessagePack envelope. No extra connection,
 * heartbeat, or control timestamp changes. Viewer samples come from WebXR,
 * in the exact reference space used to sample the tracked controller ray. */
(() => {
  'use strict';
  const encoder = new TextEncoder(), sockets = new WeakMap();
  let loopMax = 0, rafMax = 0, xrMax = 0, longMax = 0, longCount = 0;
  let lastRaf = null, lastXr = null, xrVisibility = 'unavailable';
  let due = performance.now() + 50;
  const xrSessions = new WeakMap(), xrSpaces = new WeakMap();
  let sessionNumber = 0, spaceNumber = 0, headSequence = 0;
  const heads = new Map();
  if (typeof navigator !== 'undefined' && navigator.xr && window.XRFrame) {
    const requestSession = navigator.xr.requestSession;
    navigator.xr.requestSession = async function(mode, ...args) {
      const session = await requestSession.call(this, mode, ...args);
      if (mode === 'immersive-vr' || mode === 'immersive-ar') {
        xrSessions.set(session, {id: ++sessionNumber, mode, frameTime: null, epoch: 0});
        session.addEventListener('end', () => { heads.clear(); });
        session.addEventListener('visibilitychange', () => {
          if (session.visibilityState !== 'visible') heads.clear();
        });
      }
      return session;
    };
    const getPose = XRFrame.prototype.getPose;
    XRFrame.prototype.getPose = function(space, referenceSpace) {
      const pose = getPose.call(this, space, referenceSpace);
      try {
        const session = this.session, state = xrSessions.get(session);
        const input = state && Array.from(session.inputSources).find(input =>
          input.targetRayMode === 'tracked-pointer' && input.targetRaySpace === space &&
          (input.handedness === 'left' || input.handedness === 'right'));
        if (input && pose && session.visibilityState === 'visible') {
          let reference = xrSpaces.get(referenceSpace);
          if (!reference) {
            reference = {id: ++spaceNumber, epoch: 0}; xrSpaces.set(referenceSpace, reference);
            referenceSpace.addEventListener('reset', () => { reference.epoch++; state.epoch++; heads.clear(); });
          }
          const frameId = `${state.id}:${reference.id}:${reference.epoch}`;
          let head = heads.get(frameId);
          if (!head || head.frameTime !== state.frameTime) {
            const viewer = this.getViewerPose(referenceSpace);
            head = viewer && !viewer.emulatedPosition && state.frameTime !== null ?
              {seq: ++headSequence, frameId, trackingId: `${state.id}:${state.epoch}`, frameTime: state.frameTime,
                pose: Array.from(viewer.transform.matrix), sampled: performance.now(), mode: state.mode,
                controllers: {}} : null;
            if (head) heads.set(frameId, head); else heads.delete(frameId);
            if (heads.size > 4) heads.delete(heads.keys().next().value);
          }
          if (head) head.controllers[input.handedness] = Array.from(pose.transform.matrix);
        }
      } catch (_) { heads.clear(); } // Missing tracking must stay unavailable.
      return pose;
    };
  }
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
        const state = xrSessions.get(this);
        if (state && state.frameTime !== time) { heads.clear(); state.frameTime = time; }
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
    if (Array.isArray(value)) {
      const header = value.length < 16 ? [0x90 + value.length] : [0xdc, value.length >> 8, value.length & 255];
      return [...header, ...value.flatMap(pack)];
    }
    const entries = Object.entries(value);
    return [0x80 + entries.length, ...entries.flatMap(([k, v]) => [...pack(k), ...pack(v)])];
  }
  const send = WebSocket.prototype.send;
  WebSocket.prototype.send = function(data) {
    let outgoing = data, probe = null;
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
            page_visibility: document.visibilityState, xr_visibility: xrVisibility,
            previous_send: state.completed || null};
          const fields = [['bindu_timing', timing]];
          if (heads.size) fields.push(['bindu_head', {version: 1, samples: Array.from(heads.values(), head =>
            ({seq: head.seq, frame_id: head.frameId, tracking_id: head.trackingId, frame_time_ms: head.frameTime,
              pose: head.pose, age_ms: now - head.sampled, mode: head.mode, controllers: head.controllers}))}]);
          const extra = new Uint8Array(fields.flatMap(([key, value]) => [...pack(key), ...pack(value)]));
          const total = count + fields.length;
          if (total > 65535) return send.call(this, data);
          const header = total < 16 ? [0x80 + total] : [0xde, total >> 8, total & 255];
          outgoing = new Uint8Array(header.length + b.length - offset + extra.length);
          outgoing.set(header); outgoing.set(b.subarray(offset), header.length);
          outgoing.set(extra, header.length + b.length - offset);
          state.last = now; sockets.set(this, state);
          probe = {state, seq: state.seq, started: now};
          loopMax = rafMax = xrMax = longMax = 0;
        }
      }
    } catch (_) { /* Diagnostics must never prevent the original send. */ }
    // Report completion on the next existing message, never an extra heartbeat.
    // A returned send only queued data locally; it does not prove wire delivery.
    if (!probe) return send.call(this, outgoing);
    let started;
    try { started = performance.now(); }
    catch (_) { return send.call(this, outgoing); }
    const result = send.call(this, outgoing);
    try {
      const finished = performance.now();
      probe.state.completed = {seq: probe.seq, prepare_ms: started - probe.started,
        send_call_ms: finished - started, buffered_after_bytes: this.bufferedAmount,
        wire_bytes: outgoing.byteLength};
    } catch (_) { /* Diagnostics must not change the original send result. */ }
    return result;
  };
})();
