"use strict";
const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const vm = require("node:vm");
const protocol = require("../host_pc/telemetry_protocol.js");
const root = path.resolve(__dirname, "..");
const lines = fs.readFileSync(path.join(__dirname, "telemetry_frames.hex"), "utf8").trim().split(/\r?\n/);
const packets = lines.map(hex => [...Buffer.from(hex, "hex")]);
assert.deepEqual(packets.map(b => b.length), [44, 14, 44, 14, 22]);
function parse(chunks) {
  const frames = [], errors = [], resync = [];
  const parser = protocol.createParser({
    onFrame: f => frames.push(f), onError: e => errors.push(e), onResync: n => resync.push(n)
  });
  for (const c of chunks) parser.push(c);
  return {frames, errors, resync};
}
const wire = parse(packets);
assert.equal(wire.errors.length, 0);
assert.deepEqual(wire.frames.map(f => f.protocol), ["diag-v1", "legacy", "diag-v1", "legacy", "stats-v1"]);
assert.deepEqual(wire.frames[0].diagnostic, {
  sampleId: 0x87654321, flags: 0xaf, fpgaZero: 3162, error: -37, target: 7654321,
  anglePD: -70000, positionLoopOutput: -64, outer: -64, rate: -12, raw: -70064, window: 31
});
for (const f of wire.frames.slice(0, 2)) {
  assert.equal(f.adc, 4095); assert.equal(f.pos, -123456789);
  assert.equal(f.vel, -32760); assert.equal(f.command, -900);
  assert.equal(f.runMode, 3); assert.equal(f.controlMode, 2);
}
assert.equal(wire.frames[2].diagnostic.sampleId, 0xfffffff0);
assert.equal(wire.frames[2].diagnostic.target, -2147483647);
assert.equal(wire.frames[2].diagnostic.anglePD, 2222);
assert.equal(wire.frames[2].pos, 2147483647);
assert.equal(wire.frames[3].diagnostic, null);
assert.deepEqual(wire.frames[4].stats, {
  sampleId: 0xfffffff0, adcGlitchCount: 0x12345678, adcMaxDelta: 1023,
  limitEvents: 77, command: 389, flags: 0xc3
});
// 每个可能的分包位置和逐字节输入，都应得到相同的完整快照。
const stream = packets.flat();
for (let i = 0; i <= stream.length; i++)
  assert.deepEqual(parse([stream.slice(0, i), stream.slice(i)]).frames, wire.frames);
assert.deepEqual(parse(stream.map(b => [b])).frames, wire.frames);
// 校验错误、错误版本/长度、插入垃圾、丢字节及分离帧头后均能恢复。
const corrupt = packets[0].slice(); corrupt[31] ^= 1;
const badVersion = packets[0].slice(); badVersion[2] = 2;
const badSize = packets[0].slice(); badSize[3] = 255;
for (const broken of [corrupt, badVersion, badSize, packets[0].slice(1), packets[0].slice(0, -1)]) {
  const result = parse([[0x11, 0x55], [0, ...broken, ...packets[2]]]);
  assert.equal(result.frames.length, 1);
  assert.equal(result.frames[0].diagnostic.sampleId, 0xfffffff0);
}
const resetFrames = [];
const resetParser = protocol.createParser({onFrame: f => resetFrames.push(f)});
resetParser.push(packets[0].slice(0, 20)); resetParser.reset(); resetParser.push(packets[2]);
assert.equal(resetFrames.length, 1);
// 在轻量DOM环境中执行真实页面脚本，核对记录、缺失字段和CSV列对齐。
const html = fs.readFileSync(path.join(root, "host_pc/index.html"), "utf8");
const inline = html.match(/<script>\s*([\s\S]*?)<\/script>/)[1];
const elements = new Map();
function element(id) {
  if (!elements.has(id)) elements.set(id, {
    textContent: "", disabled: false, style: {}, children: [],
    classList: {toggle() {}}, listeners: {},
    addEventListener(event, fn) { this.listeners[event] = fn; },
    prepend(row) { this.children.unshift(row); }, remove() {},
    getBoundingClientRect() { return {width: 0, height: 0}; }
  });
  return elements.get(id);
}
const blobs = [];
const context = vm.createContext({
  ACGTelemetry: protocol, document: {getElementById: element, createElement: () => ({style: {}, click() {}})},
  navigator: {}, window: {addEventListener() {}, confirm: () => true},
  performance: {now: () => 1}, Date, Blob, URL: {createObjectURL: b => {blobs.push(b); return "blob:test";}, revokeObjectURL() {}}
});
vm.runInContext(inline, context);
const run = code => vm.runInContext(code, context);
const click = id => element(id).listeners.click();
assert.equal(run("adcLinearDelta(0, 4095)"), -4095);
assert.equal(run("adcLinearDelta(4095, 0)"), 4095);
assert.equal(run("adcLinearDelta(2048, 0)"), 2048);
click("record");
context.wirePacket = packets[0]; run("appendBytes(wirePacket)");
click("record");
click("record");
context.wirePacket = packets[1]; run("appendBytes(wirePacket)");
click("record");
assert.equal(run("csvRows.length"), 2);
assert.equal(run("recordingSegment"), 2);
assert.equal(element("fpgaError").textContent, "--");
assert.equal(element("pdOuter").textContent, "--");
assert.equal(element("recordedFrames").textContent, "2");
context.wirePacket = packets[4]; run("appendBytes(wirePacket)");
assert.equal(run("csvRows.length"), 2);
assert.equal(element("adcGlitches").textContent, "305419896 / 1023");
assert.equal(element("limitEvents").textContent, 77);
click("download");
async function verifyExport() {
  const csv = (await blobs[0].text()).replace(/^\uFEFF/, "").trim().split(/\r?\n/).map(r => r.split(","));
  assert.equal(csv.length, 3);
  assert(csv.every(row => row.length === 28));
  const column = name => csv[0].indexOf(name);
  assert.equal(csv[1][column("fpga_angle_zero")], "3162");
  assert.equal(csv[1][column("weak_position_applied_raw")], "-64");
  assert.equal(csv[2][column("fpga_angle_zero")], "");
  assert.equal(csv[1][column("recording_segment")], "1");
  assert.equal(csv[2][column("recording_segment")], "2");
  context.window.confirm = () => false; click("newRecording");
  assert.equal(run("csvRows.length"), 2);
  context.window.confirm = () => true; click("newRecording");
  assert.equal(run("csvRows.length"), 0);
  assert.equal(run("recordingSegment"), 0);
  click("record"); context.wirePacket = packets[2]; run("appendBytes(wirePacket)");
  click("newRecording"); // 记录期间即使直接触发回调也不得清空数据。
  assert.equal(run("csvRows.length"), 1);
  console.log("PASS: UART wire -> host parser -> UI/CSV; legacy, chunking, corruption, reset and recording sessions");
}
verifyExport().catch(error => {console.error(error); process.exitCode = 1;});
