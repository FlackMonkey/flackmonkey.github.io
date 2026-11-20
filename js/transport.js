// USB transport for the bridge board.
//  1) Web Serial  (Chrome desktop, Chrome Android 148+)
//  2) WebUSB fallback with small built-in drivers:
//       - CDC-ACM  (ESP32-S3 native USB, RP2040, Raspberry Pi USB gadget, STM32 VCP)
//       - CP210x   (Silicon Labs CP2102/CP2104 on many ESP32 dev kits)
//       - CH34x    (WCH CH340/CH341/CH9102 compatible on cheap ESP32 dev kits)
// All drivers expose: open(baud), write(Uint8Array), close(), onData(cb), setBaud(baud)

const USB_FILTERS = [
  { vendorId: 0x303A },               // Espressif native USB (ESP32-S2/S3/C3/C6)
  { vendorId: 0x10C4 },               // Silicon Labs CP210x
  { vendorId: 0x1A86 },               // WCH CH34x / CH9102
  { vendorId: 0x1D6B },               // Linux Foundation (Raspberry Pi g_serial gadget)
  { vendorId: 0x0525 },               // Netchip (g_serial default VID)
  { vendorId: 0x2E8A },               // Raspberry Pi (RP2040)
  { vendorId: 0x0483 },               // STMicro VCP
  { classCode: 0x02 },                // any CDC device
];

// ---------------- Web Serial ----------------
class WebSerialPort {
  constructor(port) { this.port = port; this.kind = 'Web Serial'; this.reader = null; this.cb = () => {}; this.closing = false; }
  onData(cb) { this.cb = cb; }
  async open(baud) {
    await this.port.open({ baudRate: baud, bufferSize: 4096 });
    // DTR/RTS both de-asserted: never toggles ESP32 auto-reset / boot-mode circuits
    try { await this.port.setSignals({ dataTerminalReady: false, requestToSend: false }); } catch {}
    this.writer = this.port.writable.getWriter();
    this.loop = this._readLoop();
    const info = this.port.getInfo?.() || {};
    this.name = info.usbVendorId ? `USB ${hex4(info.usbVendorId)}:${hex4(info.usbProductId)}` : 'serial port';
  }
  async _readLoop() {
    // non-fatal errors (framing, overrun) give us a fresh readable stream -> keep looping
    while (this.port.readable && !this.closing) {
      this.reader = this.port.readable.getReader();
      try { for (;;) { const { value, done } = await this.reader.read(); if (done) break; if (value) this.cb(value); } }
      catch (e) { /* fall through, retry if port still readable */ }
      finally { try { this.reader.releaseLock(); } catch {} }
    }
    if (!this.closing) this.onDisconnect?.();
  }
  write(data) { return this.writer.write(data); }
  async setBaud(baud) { await this.close(); this.closing = false; await this.open(baud); }
  async close() {
    this.closing = true;
    try { await this.reader?.cancel(); } catch {}
    try { await this.loop; } catch {}
    try { this.writer?.releaseLock(); } catch {}
    try { await this.port.close(); } catch {}
  }
}

// ---------------- WebUSB common ----------------
class UsbSerialBase {
  constructor(dev) { this.dev = dev; this.cb = () => {}; this.closing = false; }
  onData(cb) { this.cb = cb; }
  findBulk(iface) {
    const alt = iface.alternate || iface.alternates[0];
    this.epIn = alt.endpoints.find(e => e.type === 'bulk' && e.direction === 'in');
    this.epOut = alt.endpoints.find(e => e.type === 'bulk' && e.direction === 'out');
    if (!this.epIn || !this.epOut) throw new Error('No bulk endpoints');
  }
  async _readLoop() {
    while (!this.closing) {
      try {
        const r = await this.dev.transferIn(this.epIn.endpointNumber, Math.max(64, this.epIn.packetSize));
        if (r.status === 'ok' && r.data?.byteLength) this.cb(new Uint8Array(r.data.buffer, r.data.byteOffset, r.data.byteLength));
        else if (r.status === 'stall') await this.dev.clearHalt('in', this.epIn.endpointNumber);
      } catch (e) { if (!this.closing) { this.onDisconnect?.(); } return; }
    }
  }
  write(data) { return this.dev.transferOut(this.epOut.endpointNumber, data); }
  ctrlOut(recipient, request, value, index, data) {
    return this.dev.controlTransferOut({ requestType: this.reqType || 'vendor', recipient, request, value, index }, data);
  }
  async close() { this.closing = true; try { await this.dev.close(); } catch {} }
}

class CdcAcm extends UsbSerialBase {
  constructor(dev) { super(dev); this.kind = 'WebUSB CDC-ACM'; this.reqType = 'class'; }
  async open(baud) {
    const d = this.dev; await d.open();
    if (d.configuration === null) await d.selectConfiguration(1);
    const ifs = d.configuration.interfaces;
    const data = ifs.find(i => (i.alternate || i.alternates[0]).interfaceClass === 0x0A);
    const comm = ifs.find(i => (i.alternate || i.alternates[0]).interfaceClass === 0x02);
    if (!data) throw new Error('No CDC data interface');
    this.commIf = comm ? comm.interfaceNumber : 0;
    if (comm) { try { await d.claimInterface(comm.interfaceNumber); } catch {} }
    await d.claimInterface(data.interfaceNumber);
    this.findBulk(data);
    await this.setBaud(baud);
    try { await this.ctrlOut('interface', 0x22, 0x0000, this.commIf); } catch {}   // DTR=0 RTS=0
    this.name = d.productName || 'CDC device';
    this._readLoop();
  }
  async setBaud(baud) {
    const lc = new Uint8Array(7); new DataView(lc.buffer).setUint32(0, baud, true); lc[6] = 8;   // 8N1
    try { await this.ctrlOut('interface', 0x20, 0, this.commIf, lc); } catch {}
  }
}

class Cp210x extends UsbSerialBase {
  constructor(dev) { super(dev); this.kind = 'WebUSB CP210x'; }
  async open(baud) {
    const d = this.dev; await d.open();
    if (d.configuration === null) await d.selectConfiguration(1);
    const iface = d.configuration.interfaces[0];
    await d.claimInterface(iface.interfaceNumber); this.ifn = iface.interfaceNumber;
    this.findBulk(iface);
    await this.ctrlOut('interface', 0x00, 0x0001, this.ifn);            // IFC_ENABLE
    await this.ctrlOut('interface', 0x03, 0x0800, this.ifn);            // SET_LINE_CTL 8N1
    await this.ctrlOut('interface', 0x07, 0x0300, this.ifn);            // SET_MHS: DTR=0 RTS=0 (with masks)
    await this.setBaud(baud);
    this.name = d.productName || 'CP210x';
    this._readLoop();
  }
  async setBaud(baud) {
    const b = new Uint8Array(4); new DataView(b.buffer).setUint32(0, baud, true);
    await this.ctrlOut('interface', 0x1E, 0, this.ifn, b);              // SET_BAUDRATE
  }
}

class Ch34x extends UsbSerialBase {
  constructor(dev) { super(dev); this.kind = 'WebUSB CH34x'; }
  async open(baud) {
    const d = this.dev; await d.open();
    if (d.configuration === null) await d.selectConfiguration(1);
    const iface = d.configuration.interfaces[0];
    await d.claimInterface(iface.interfaceNumber);
    this.findBulk(iface);
    try {
      const r = await d.controlTransferIn({ requestType: 'vendor', recipient: 'device', request: 0x5F, value: 0, index: 0 }, 2);
      this.version = r.data?.getUint8(0) || 0;
    } catch { this.version = 0; }
    await this.ctrlOut('device', 0xA1, 0, 0);                            // SERIAL_INIT
    await this.setBaud(baud);
    await this.ctrlOut('device', 0xA4, 0xFFFF, 0);                       // modem ctrl (~bits): DTR=0 RTS=0
    this.name = d.productName || 'CH34x';
    this._readLoop();
  }
  // divisor calculation ported from Linux drivers/usb/serial/ch341.c
  static divisor(speed) {
    const CLK = 48000000, div = (ps, f) => 1 << (12 - 3 * ps - f);
    const minRate = ps => CLK / (div(ps, 1) * 512);
    speed = Math.max(46, Math.min(3000000, speed));
    let fact = 1, ps;
    for (ps = 3; ps >= 0; ps--) if (speed > minRate(ps)) break;
    if (ps < 0) throw new Error('baud too low');
    let clkDiv = div(ps, fact);
    let d = Math.floor(CLK / (clkDiv * speed));
    if (d < 9 || d > 255) { d = Math.floor(d / 2); clkDiv *= 2; fact = 0; }
    if (d < 2) throw new Error('baud too high');
    if (Math.floor(16 * CLK / (clkDiv * d)) - 16 * speed >= 16 * speed - Math.floor(16 * CLK / (clkDiv * (d + 1)))) d++;
    if (fact === 1 && d % 2 === 0) { d /= 2; fact = 0; }
    return ((0x100 - d) << 8) | (fact << 2) | ps;
  }
  async setBaud(baud) {
    let val = Ch34x.divisor(baud);
    if (this.version > 0x27) val |= 0x80;
    await this.ctrlOut('device', 0x9A, 0x1312, val);                     // divisor/prescaler
    await this.ctrlOut('device', 0x9A, 0x2518, 0x00C3);                  // LCR: RX|TX enable, 8 bits
  }
}

const hex4 = n => (n ?? 0).toString(16).padStart(4, '0').toUpperCase();

export const transportSupport = () => ({ serial: 'serial' in navigator, usb: 'usb' in navigator });

/** Ask the user for a device and return an opened port. prefer: 'auto' | 'serial' | 'usb' */
export async function connectPort(baud, prefer = 'auto') {
  const sup = transportSupport();
  const useSerial = prefer === 'serial' || (prefer === 'auto' && sup.serial);
  if (useSerial) {
    if (!sup.serial) throw new Error('Web Serial is not available in this browser');
    const port = await navigator.serial.requestPort({});
    const p = new WebSerialPort(port); await p.open(baud); return p;
  }
  if (!sup.usb) throw new Error('Neither Web Serial nor WebUSB is available. Use Chrome on Android over HTTPS.');
  const dev = await navigator.usb.requestDevice({ filters: USB_FILTERS });
  return openUsb(dev, baud);
}

export async function openUsb(dev, baud) {
  let drv;
  if (dev.vendorId === 0x10C4) drv = new Cp210x(dev);
  else if (dev.vendorId === 0x1A86) drv = new Ch34x(dev);
  else drv = new CdcAcm(dev);
  await drv.open(baud);
  return drv;
}

/** Re-open a previously granted device without a picker (used for auto-reconnect). */
export async function reconnectKnown(baud, prefer = 'auto') {
  const sup = transportSupport();
  if ((prefer !== 'usb') && sup.serial) {
    const ports = await navigator.serial.getPorts();
    if (ports.length) { const p = new WebSerialPort(ports[0]); await p.open(baud); return p; }
  }
  if ((prefer !== 'serial') && sup.usb) {
    const devs = await navigator.usb.getDevices();
    if (devs.length) return openUsb(devs[0], baud);
  }
  return null;
}
