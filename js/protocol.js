// ELRS Phone TX — wire protocols (shared by the app and the tests)
// -----------------------------------------------------------------
// Phone -> bridge board : CRSF RC frames  OR  MAVLink v2 RC_CHANNELS_OVERRIDE
//                         + bridge config frames (CRSF framing, type 0x7F)
// Bridge board -> phone : CRSF frames (module telemetry passthrough + bridge status 0x7F)

export const CRSF = {
  SYNC: 0xC8,
  ADDR_MODULE: 0xEE,        // "CRSF transmitter" = the ELRS TX module
  ADDR_RADIO: 0xEA,         // "radio transmitter" = handset (us)
  T_GPS: 0x02, T_BATTERY: 0x08, T_LINK_STATS: 0x14, T_RC: 0x16, T_ATTITUDE: 0x1E,
  T_FLIGHT_MODE: 0x21, T_PING: 0x28, T_DEVICE_INFO: 0x29, T_RADIO_ID: 0x3A,
  T_BRIDGE: 0x7F,           // custom: bridge board config/status (never forwarded to the module)
  BR_SET: 0x01, BR_GET: 0x02, BR_STATUS: 0x81,
};

// ---------- CRC ----------
export function crc8dvb(bytes, start = 0, end = bytes.length) {
  let crc = 0;
  for (let i = start; i < end; i++) {
    crc ^= bytes[i];
    for (let b = 0; b < 8; b++) crc = (crc & 0x80) ? ((crc << 1) ^ 0xD5) & 0xFF : (crc << 1) & 0xFF;
  }
  return crc;
}

// CRC-16/MCRF4XX ("X.25" in MAVLink docs)
export function crcX25Accumulate(byte, crc) {
  let tmp = byte ^ (crc & 0xFF);
  tmp = (tmp ^ (tmp << 4)) & 0xFF;
  return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF;
}
export function crcX25(bytes, crc = 0xFFFF) {
  for (const b of bytes) crc = crcX25Accumulate(b, crc);
  return crc;
}

// ---------- unit conversion ----------
// CRSF ticks: 172 = 988us, 992 = 1500us, 1811 = 2012us
export const usToCrsf = us => Math.max(0, Math.min(2047, Math.round((us - 1500) * 1.6 + 992)));
export const crsfToUs = t => Math.round((t - 992) / 1.6 + 1500);

// ---------- CRSF frames ----------
export function crsfFrame(addr, type, payload) {
  const f = new Uint8Array(payload.length + 4);
  f[0] = addr; f[1] = payload.length + 2; f[2] = type; f.set(payload, 3);
  f[f.length - 1] = crc8dvb(f, 2, f.length - 1);
  return f;
}

/** 16 channels in microseconds -> CRSF RC_CHANNELS_PACKED frame (26 bytes) */
export function crsfRcFrame(channelsUs) {
  const p = new Uint8Array(22);
  let bitPos = 0;
  for (let i = 0; i < 16; i++) {
    const v = usToCrsf(channelsUs[i] ?? 1500);
    for (let b = 0; b < 11; b++) {
      if (v & (1 << b)) p[bitPos >> 3] |= 1 << (bitPos & 7);
      bitPos++;
    }
  }
  return crsfFrame(CRSF.ADDR_MODULE, CRSF.T_RC, p);
}

export function unpackCrsfChannels(payload) {
  const out = [];
  let bitPos = 0;
  for (let i = 0; i < 16; i++) {
    let v = 0;
    for (let b = 0; b < 11; b++) { if (payload[bitPos >> 3] & (1 << (bitPos & 7))) v |= 1 << b; bitPos++; }
    out.push(v);
  }
  return out;
}

/** Device ping (asks the ELRS module for its name/version; answer = DEVICE_INFO 0x29) */
export const crsfPingFrame = () =>
  crsfFrame(CRSF.ADDR_MODULE, CRSF.T_PING, new Uint8Array([CRSF.ADDR_MODULE, CRSF.ADDR_RADIO]));

/** Bridge configuration (consumed by the board, never forwarded) */
export function bridgeSetFrame({ outBaud, inverted, fixedPeriodUs, linkBaud, failsafeMs }) {
  const p = new Uint8Array(14); const dv = new DataView(p.buffer);
  p[0] = CRSF.BR_SET;
  dv.setUint32(1, outBaud >>> 0);
  p[5] = inverted ? 1 : 0;
  dv.setUint16(6, fixedPeriodUs & 0xFFFF);
  dv.setUint32(8, linkBaud >>> 0);
  dv.setUint16(12, failsafeMs & 0xFFFF);
  return crsfFrame(CRSF.SYNC, CRSF.T_BRIDGE, p);
}
export const bridgeGetFrame = () => crsfFrame(CRSF.SYNC, CRSF.T_BRIDGE, new Uint8Array([CRSF.BR_GET]));

// ---------- MAVLink v2 ----------
export const MAV = { RC_OVERRIDE: 70, RC_OVERRIDE_EXTRA: 124, HEARTBEAT: 0, HEARTBEAT_EXTRA: 50 };

function mavlink2(msgId, crcExtra, payload, seq, sysId, compId) {
  // trailing zero bytes are truncated in v2 (keep at least 1 byte)
  let len = payload.length;
  while (len > 1 && payload[len - 1] === 0) len--;
  const f = new Uint8Array(10 + len + 2);
  f[0] = 0xFD; f[1] = len; f[2] = 0; f[3] = 0; f[4] = seq & 0xFF; f[5] = sysId; f[6] = compId;
  f[7] = msgId & 0xFF; f[8] = (msgId >> 8) & 0xFF; f[9] = (msgId >> 16) & 0xFF;
  f.set(payload.subarray(0, len), 10);
  let crc = crcX25(f.subarray(1, 10 + len));
  crc = crcX25Accumulate(crcExtra, crc);
  f[10 + len] = crc & 0xFF; f[11 + len] = crc >> 8;
  return f;
}

/** RC_CHANNELS_OVERRIDE (#70) with 18 channel slots (us). 0 = release / ignore. */
export function mavRcOverride(channelsUs, seq, { sysId = 255, compId = 190, targetSys = 1, targetComp = 1 } = {}) {
  const p = new Uint8Array(38); const dv = new DataView(p.buffer);
  for (let i = 0; i < 8; i++) dv.setUint16(i * 2, channelsUs[i] ?? 0, true);
  p[16] = targetSys; p[17] = targetComp;
  for (let i = 8; i < 18; i++) dv.setUint16(18 + (i - 8) * 2, channelsUs[i] ?? 0, true);
  return mavlink2(MAV.RC_OVERRIDE, MAV.RC_OVERRIDE_EXTRA, p, seq, sysId, compId);
}

export function mavHeartbeat(seq, { sysId = 255, compId = 190 } = {}) {
  // custom_mode u32, type u8 (6=GCS), autopilot u8 (8=invalid), base_mode, system_status, mavlink_version
  const p = new Uint8Array([0, 0, 0, 0, 6, 8, 0, 4, 3]);
  return mavlink2(MAV.HEARTBEAT, MAV.HEARTBEAT_EXTRA, p, seq, sysId, compId);
}

// ---------- CRSF stream parser (board -> phone) ----------
export class CrsfParser {
  constructor(onFrame) { this.buf = new Uint8Array(0); this.onFrame = onFrame; }
  push(chunk) {
    const nb = new Uint8Array(this.buf.length + chunk.length);
    nb.set(this.buf); nb.set(chunk, this.buf.length); this.buf = nb;
    let i = 0;
    while (this.buf.length - i >= 4) {
      const a = this.buf[i];
      if (a !== 0xC8 && a !== 0xEA && a !== 0xEE) { i++; continue; }
      const len = this.buf[i + 1];
      if (len < 2 || len > 62) { i++; continue; }
      if (this.buf.length - i < len + 2) break;
      const end = i + len + 2;
      if (crc8dvb(this.buf, i + 2, end - 1) === this.buf[end - 1]) {
        this.onFrame(this.buf[i + 2], this.buf.subarray(i + 3, end - 1), this.buf[i]);
        i = end;
      } else i++;
    }
    this.buf = this.buf.slice(i);
  }
}

// ---------- telemetry decoding ----------
export function decodeTelemetry(type, p) {
  const dv = new DataView(p.buffer, p.byteOffset, p.byteLength);
  switch (type) {
    case CRSF.T_LINK_STATS:
      if (p.length < 10) return null;
      return { kind: 'link', rssi1: -p[0], rssi2: -p[1], lq: p[2], snr: dv.getInt8(3), ant: p[4],
               rfMode: p[5], txPowerIdx: p[6], dlRssi: -p[7], dlLq: p[8], dlSnr: dv.getInt8(9) };
    case CRSF.T_BATTERY:
      if (p.length < 8) return null;
      return { kind: 'battery', v: dv.getUint16(0) / 10, a: dv.getUint16(2) / 10,
               mah: (p[4] << 16) | (p[5] << 8) | p[6], pct: p[7] };
    case CRSF.T_FLIGHT_MODE:
      return { kind: 'fmode', text: new TextDecoder().decode(p.subarray(0, Math.max(0, p.indexOf(0)) || p.length)) };
    case CRSF.T_ATTITUDE:
      if (p.length < 6) return null;
      return { kind: 'att', pitch: dv.getInt16(0) / 10000 * 57.2958, roll: dv.getInt16(2) / 10000 * 57.2958, yaw: dv.getInt16(4) / 10000 * 57.2958 };
    case CRSF.T_DEVICE_INFO: {
      // [dest][origin][name\0][serial u32][hw u32][sw u32][paramCount][paramVer]
      const z = p.indexOf(0, 2); if (z < 0) return null;
      const name = new TextDecoder().decode(p.subarray(2, z));
      const o = z + 1; let sw = '';
      if (p.length >= o + 12) sw = `${p[o + 9]}.${p[o + 10]}.${p[o + 11]}`;
      return { kind: 'devinfo', name, sw };
    }
    case CRSF.T_BRIDGE:
      if (p[0] !== CRSF.BR_STATUS || p.length < 25) return null;
      return { kind: 'bridge', fw: p[1], synced: !!(p[2] & 1), linkOk: !!(p[2] & 2), inverted: !!(p[2] & 4),
               outBaud: dv.getUint32(3), periodUs: dv.getUint16(7), rxFps: dv.getUint16(9), txFps: dv.getUint16(11),
               telemFps: dv.getUint16(13), linkProto: ['none', 'CRSF', 'MAVLink'][p[15]] || '?',
               linkBaud: dv.getUint32(16), failsafeMs: dv.getUint16(20), fixedPeriodUs: dv.getUint16(22),
               platform: ['?', 'ESP32-S3', 'ESP32', 'RPi'][p[24]] || '?' };
    default: return null;
  }
}
