#!/usr/bin/env node
// Runs the installed AdvantageScope WPILOG worker and WASM indexer unchanged.
// No npm dependencies and no changes to the installed application or its state.
const fs = require('node:fs');
const path = require('node:path');
const os = require('node:os');
const vm = require('node:vm');
const crypto = require('node:crypto');

function args() {
  const options = {app: path.join(os.homedir(), 'wpilib/2026/advantagescope/resources/app.asar'),
    'max-speed': 4.572, wheelbase: .6731, trackwidth: .6731};
  for (let i = 2; i < process.argv.length; ++i) {
    const key = process.argv[i];
    if (key === '--require-fuel') {options.requireFuel = true; continue;}
    if (!['--app', '--log', '--output', '--layout', '--max-speed', '--wheelbase', '--trackwidth'].includes(key) || !process.argv[i + 1]) {
      throw Error('Usage: node scripts/validate_advantagescope.cjs --log file.wpilog [--app app.asar] [--output evidence.json] [--layout layout.json] [--require-fuel] [--max-speed mps] [--wheelbase m] [--trackwidth m]');
    }
    options[key.slice(2)] = process.argv[++i];
  }
  if (!options.log) throw Error('--log is required');
  for (const key of ['max-speed', 'wheelbase', 'trackwidth']) {
    options[key] = Number(options[key]);
    if (!Number.isFinite(options[key]) || options[key] <= 0) throw Error(`Invalid ${key}`);
  }
  return options;
}
function writeJson(filename, value) {
  fs.mkdirSync(path.dirname(path.resolve(filename)), {recursive: true});
  fs.writeFileSync(filename, JSON.stringify(value, null, 2) + '\n');
}
function archiveReader(filename) {
  const bytes = fs.readFileSync(filename);
  const header = JSON.parse(bytes.subarray(16, 16 + bytes.readUInt32LE(12)).toString());
  const start = 8 + bytes.readUInt32LE(4);
  return name => {
    let entry = header;
    for (const part of name.split('/')) entry = entry.files[part];
    if (!entry || entry.unpacked || entry.offset === undefined) throw Error(`Unavailable ASAR entry: ${name}`);
    const offset = start + Number(entry.offset);
    return bytes.subarray(offset, offset + entry.size);
  };
}
function layout(version, fields, options) {
  const source = (type, field, options) => ({type, logKey: field.key, logType: field.type, visible: true, options});
  const pose = fields.find(field => field.type === 'Pose2d');
  const states = fields.filter(field => field.type === 'SwerveModuleState[]');
  const pose3d = fields.find(field => field.type === 'Pose3d' && field.key.includes('/Simulation/RobotTruth')) || fields.find(field => field.type === 'Pose3d');
  const speeds = fields.find(field => field.type === 'ChassisSpeeds' && field.key.endsWith('/Drive/ChassisSpeeds')) || fields.find(field => field.type === 'ChassisSpeeds');
  const colors = ['#2b66a2', '#af2437', '#3b875a'];
  const fuel = fields.filter(field => field.type === 'Pose3d[]' && field.key.includes('/Simulation/') && !field.key.includes('/Colliders/'));
  const graphSources = speeds.decodedNumbers.map((child, index) => ({
    type: 'smooth', logKey: child.key, logType: 'Number', visible: true,
    options: {color: colors[index % colors.length], size: 'normal'}
  }));
  const tabs = [
    {type: 2, title: 'MECO Field Pose', controller: {
      sources: [source('robot', pose, {bumpers: ''})], field: 'FRC:Evergreen', orientation: 0, size: 'large'
    }},
    {type: 3, title: 'MECO 3D Pose', controller: {
      // Axes needs no downloaded robot mesh; users may add their robot asset.
      sources: [source('axes', pose3d, {}), ...fuel.map(field => source('axes', field, {}))], game: 'FRC:Evergreen'
    }},
    {type: 1, title: 'MECO Chassis Speeds', controller: {
      leftSources: graphSources.filter(source => !source.logKey.endsWith('/omega')),
      rightSources: graphSources.filter(source => source.logKey.endsWith('/omega')), discreteSources: [],
      leftLockedRange: null, rightLockedRange: null,
      leftUnitConversion: {autoTarget: null, preset: null},
      rightUnitConversion: {autoTarget: null, preset: null}, leftFilter: 0, rightFilter: 0
    }},
    {type: 9, title: 'MECO Swerve', controller: {
      sources: states.map((field, index) => source('states', field, {
        color: index === 0 ? '#ff0000' : '#0000ff', arrangement: '0,1,2,3'
      })), maxSpeed: options['max-speed'], sizeX: options.wheelbase, sizeY: options.trackwidth, orientation: 0
    }},
    {type: 12, title: 'MECO Build Metadata', controller: null}
  ].map(tab => ({...tab, controllerUUID: crypto.randomUUID(), renderer: null, controlsHeight: 180}));
  return {version, hubs: [{x: 50, y: 50, width: 1280, height: 800, state: {
    sidebar: {width: 300, expanded: ['NT:', 'NT:/AdvantageScope', 'NT:/AdvantageScope/Drive']},
    tabs: {selected: 0, tabs}
  }}], satellites: []};
}

async function main() {
  const options = args();
  const read = archiveReader(options.app);
  const version = JSON.parse(read('package.json').toString()).version;
  if (version !== '26.0.0') throw Error(`Installed AdvantageScope ${version}: worker/layout contract is verified for 26.0.0 only`);
  const temporary = fs.mkdtempSync(path.join(os.tmpdir(), 'meco-advantagescope-'));
  const errors = [];
  const responses = [];
  const report = {passed: false, advantageScopeVersion: version, log: path.resolve(options.log),
    workerSha256: crypto.createHash('sha256').update(read('bundles/hub$wpilogWorker.js')).digest('hex'),
    parser: 'Installed unmodified hub$wpilogWorker.js + hub$wpilogIndexer.wasm', fields: [], metadata: [], errors};
  try {
    for (const filename of ['hub$wpilogWorker.js', 'hub$wpilogIndexer.js', 'hub$wpilogIndexer.wasm']) {
      fs.writeFileSync(path.join(temporary, filename), read('bundles/' + filename));
    }
    const context = vm.createContext({require, process, Buffer, TextEncoder, TextDecoder, URL,
      WebAssembly, setTimeout, clearTimeout, performance, crypto: crypto.webcrypto,
      __dirname: temporary, __filename: path.join(temporary, 'hub$wpilogWorker.js'), module: {exports: {}},
      console: {log() {}, warn: (...values) => errors.push(values.join(' ')), error: (...values) => errors.push(values.join(' '))},
      self: {postMessage: response => {if (response.type === 'initial' || response.type === 'failed') responses.push(response.type);}}});
    context.self.importScripts = name => {
      if (name !== './hub$wpilogIndexer.js') throw Error(`Unexpected worker dependency ${name}`);
      vm.runInContext(read('bundles/hub$wpilogIndexer.js').toString(), context);
    };
    vm.runInContext(read('bundles/hub$wpilogWorker.js').toString(), context);
    await context.self.onmessage({data: {type: 'start', data: [new Uint8Array(fs.readFileSync(options.log))]}});
    if (!responses.some(response => response === 'initial') || responses.some(response => response === 'failed')) {
      throw Error('Installed AdvantageScope worker rejected the WPILOG');
    }
    const evaluate = expression => vm.runInContext(expression, context);
    report.timestampRangeSeconds = evaluate('log.getTimestampRange()');
    const keys = evaluate('log.getFieldKeys()');
    // The UI normally requests schemas on demand before decoding structures.
    for (const key of keys.filter(key => key.includes('/.schema/'))) {
      await context.self.onmessage({data: {type: 'parseField', key}});
    }
    for (const key of keys.filter(key => key.startsWith('/Metadata/') || key.startsWith('NT:/Metadata/'))) {
      await context.self.onmessage({data: {type: 'parseField', key}});
      const field = evaluate(`log.getField(${JSON.stringify(key)}).toSerialized()`);
      report.metadata.push({key, name: key.slice(key.indexOf('/Metadata/') + '/Metadata/'.length),
        wpilibType: field.wpilibType, sampleCount: field.timestamps.length,
        last: field.values.at(-1), typeWarning: field.typeWarning});
    }
    for (const name of ['Project', 'WPILib']) {
      if (!report.metadata.some(field => field.name === name && field.wpilibType === 'string' &&
          field.sampleCount > 0 && !field.typeWarning && typeof field.last === 'string' && field.last.trim().length > 0)) {
        throw Error(`Missing or undecodable nonempty /Metadata/${name} string`);
      }
    }
    const wanted = keys.filter(key => (key.includes('/AdvantageScope/') || key.includes('/Simulation/')) &&
      evaluate(`log.getStructuredType(${JSON.stringify(key)})`) !== null);
    for (const key of wanted) {
      await context.self.onmessage({data: {type: 'parseField', key}});
      const raw = evaluate(`log.getField(${JSON.stringify(key)}).toSerialized()`);
      const childKeys = evaluate('log.getFieldKeys()').filter(child => child.startsWith(key + '/'));
      const children = childKeys.map(child => {
        const field = evaluate(`log.getField(${JSON.stringify(child)}).toSerialized()`);
        return {key: child, sampleCount: field.timestamps.length,
          last: field.values.at(-1), typeWarning: field.typeWarning};
      }).filter(child => typeof child.last === 'number');
      report.fields.push({key, type: raw.structuredType, sampleCount: raw.timestamps.length,
        typeWarning: raw.typeWarning, decodedNumbers: children});
    }
    for (const type of ['Pose2d', 'Pose3d', 'ChassisSpeeds', 'SwerveModuleState[]']) {
      const matches = report.fields.filter(field => field.type === type && (field.key.includes('/Drive/') || (type === 'Pose3d' && field.key.includes('/Simulation/RobotTruth'))));
      if (!matches.length || (type === 'SwerveModuleState[]' && matches.length < 2) || matches.some(field => !field.sampleCount || field.typeWarning ||
          !field.decodedNumbers.length || field.decodedNumbers.some(child =>
            !child.sampleCount || child.typeWarning || !Number.isFinite(child.last)))) {
        throw Error(`Missing or undecodable populated ${type} topic`);
      }
    }
    for (const states of report.fields.filter(field => field.type === 'SwerveModuleState[]' && field.key.includes('/Drive/'))) {
      if (!states.decodedNumbers.some(child => child.key.endsWith('/length') && child.last === 4)) {
        throw Error(`Expected four modules in ${states.key}`);
      }
    }
    const fuel = report.fields.filter(field => field.type === 'Pose3d[]' && field.key.includes('/Simulation/') && !field.key.includes('/Colliders/'));
    if (options.requireFuel && (!fuel.length || fuel.some(field => !field.sampleCount || field.typeWarning ||
        !field.decodedNumbers.some(child => child.key.endsWith('/length') && child.sampleCount)))) {
      throw Error('Missing or undecodable simulation Pose3d[] fuel topic');
    }
    if (errors.some(error => error.includes('Failed to decode'))) throw Error('Installed parser reported decoding errors');
    report.passed = true;
    if (options.layout) {
      writeJson(options.layout, layout(version, report.fields, options));
      report.layout = path.resolve(options.layout);
      report.layoutValidation = 'Generated from installed 26.0.0 export/restore schema; GUI import not exercised';
    }
  } catch (error) {
    report.failure = String(error);
    process.exitCode = 1;
  } finally {
    fs.rmSync(temporary, {recursive: true, force: true});
    if (options.output) writeJson(options.output, report);
    console.log(JSON.stringify(report, null, 2));
  }
}
main().catch(error => {console.error(String(error)); process.exitCode = 1;});
