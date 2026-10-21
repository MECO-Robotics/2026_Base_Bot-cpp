#!/usr/bin/env python3
"""Exercise only an owned/local HAL simulator; numerical evidence is not calibration."""
import argparse
import asyncio
import contextlib
import json
import math
from pathlib import Path
import statistics
import struct
import sys
import time
from types import SimpleNamespace
import ntcore
from websockets.asyncio.client import connect


def finite(values, length=None):
    return (length is None or len(values) == length) and all(math.isfinite(v) for v in values)


def decoded_poses(payload):
    if len(payload) % 56:
        raise RuntimeError('Malformed Pose3d struct payload')
    values = struct.unpack('<' + 'd' * (len(payload) // 8), payload)
    if not finite(values):
        raise RuntimeError('Nonfinite Pose3d struct telemetry')
    return len(payload) // 56


def struct_atom(subscriber, count):
    atom = subscriber.getAtomic()
    values = list(struct.unpack('<' + 'd' * count, atom.value)) if len(atom.value) == count * 8 else []
    return SimpleNamespace(time=atom.time, value=values)


async def exercise(args):
    evidence = {'transport': 'localhost HAL websocket + NT4',
                'scope': 'Software simulation behavior; excludes physical calibration',
                'samples': {}, 'checks': {}}
    instance = ntcore.NetworkTableInstance.create()
    instance.startClient4('base-cpp-sim-smoke')
    instance.setServer('127.0.0.1', args.nt_port)
    drive = instance.getTable('AdvantageScope/Drive')
    simulation = instance.getTable('Simulation')
    speed = drive.getRawTopic('ChassisSpeeds').subscribe('struct:ChassisSpeeds', b'')
    requested = drive.getRawTopic('RequestedChassisSpeeds').subscribe('struct:ChassisSpeeds', b'')
    pose = drive.getRawTopic('Pose').subscribe('struct:Pose2d', b'')
    truth = simulation.getRawTopic('RobotTruth').subscribe('struct:Pose3d', b'')
    sim_time = simulation.getDoubleTopic('Time').subscribe(0)
    battery = simulation.getDoubleTopic('BatteryVoltage').subscribe(float('nan'))
    supply_current = simulation.getDoubleTopic('SupplyCurrent').subscribe(float('nan'))
    held = simulation.getDoubleTopic('Held').subscribe(-1)
    intake_active = simulation.getBooleanTopic('IntakeActive').subscribe(True)
    token = simulation.getStringTopic('RunToken').subscribe('')
    collider_outline = instance.getRawTopic('/Simulation/Colliders/Box0').subscribe('struct:Pose3d[]', b'')
    module_topics = {name: instance.getRawTopic('/AdvantageScope/Drive/' + name)
                     .subscribe('struct:SwerveModuleState[]', b'')
                     for name in ('ModuleStates', 'ModuleTargets')}
    structs = {name: simulation.getRawTopic(name).subscribe(kind, b'')
               for name, kind in (('RobotTruth', 'struct:Pose3d'), ('Pieces', 'struct:Pose3d[]'))}
    structs['Pose'] = pose
    try:
        async with connect(f'ws://127.0.0.1:{args.hal_port}/wpilibws', max_size=None) as websocket:
            async def drain():
                async for _ in websocket:
                    pass
            reader = asyncio.create_task(drain())

            async def control(enabled=False, forward=0., turn=0., reset=False,
                              scene_reset=False, intake=False, launch=False, autonomous=False):
                if args.run_token and token.get() != args.run_token:
                    raise RuntimeError('Simulator identity changed or did not match owned child')
                buttons = [False, reset, False, scene_reset, intake, launch] + [False] * 4
                await websocket.send(json.dumps({'type': 'Joystick', 'device': '0', 'data': {
                    '>axes': [0., -forward, 0., 0., -turn, 0.], '>buttons': buttons, '>povs': [-1]}}))
                await websocket.send(json.dumps({'type': 'DriverStation', 'device': '', 'data': {
                    '>enabled': enabled, '>autonomous': autonomous, '>test': False,
                    '>estop': False, '>fms': False, '>ds': True, '>new_data': True}}))

            async def hold(duration=.3, **kwargs):
                deadline = time.monotonic() + duration
                while time.monotonic() < deadline:
                    await control(**kwargs)
                    await asyncio.sleep(.02)

            async def reset_scene(**kwargs):
                await hold(scene_reset=True, **kwargs)
                await hold(**kwargs)

            async def phase(name, duration=2., **kwargs):
                readings = []
                start = time.monotonic()
                last_stamp = 0
                while time.monotonic() - start < duration:
                    await control(**kwargs)
                    speeds, estimated = struct_atom(speed, 3), struct_atom(pose, 3)
                    actual = struct_atom(truth, 7)
                    if len(actual.value) == 7:
                        x, y, z, qw, qx, qy, qz = actual.value
                        actual.value = [x, y, math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))]
                    clock = sim_time.getAtomic()
                    # Cached samples never count twice. NT timestamps, not polling time,
                    # define integration intervals. Simulator time is retained independently.
                    heartbeat = clock.time
                    if heartbeat > last_stamp and finite(speeds.value, 3) and finite(estimated.value, 3):
                        last_stamp = heartbeat
                        readings.append({'ntTime': speeds.time / 1e6, 'poseTime': estimated.time / 1e6,
                                         'truthTime': actual.time / 1e6, 'simulationTime': clock.value,
                                         'batteryVoltage': battery.get(), 'supplyCurrent': supply_current.get(),
                                         'estimated': list(estimated.value),
                                         'truth': list(actual.value), 'speeds': list(speeds.value),
                                         'requestedSpeeds': struct_atom(requested, 3).value,
                                         'moduleStates': struct_atom(module_topics['ModuleStates'], 8).value,
                                         'moduleTargets': struct_atom(module_topics['ModuleTargets'], 8).value,
                                         'structTimes': {key: sub.getAtomic().time for key, sub in structs.items()}})
                    await asyncio.sleep(.02)
                if len(readings) < 5:
                    raise RuntimeError(f'Missing/stale NT speed telemetry in {name}')
                if readings[-1]['simulationTime'] - readings[0]['simulationTime'] < duration * .4:
                    raise RuntimeError(f'Missing/stale simulation clock in {name}')
                final_time = readings[-1]['simulationTime']
                tail = [row for row in readings if row['simulationTime'] >= final_time - .4]
                if len(tail) < 3:
                    raise RuntimeError(f'Insufficient fresh steady-window samples in {name}')
                result = [statistics.median(row['speeds'][i] for row in tail) for i in range(3)]
                if args.expect_jolt and name in ('forward', 'turn'):
                    for key, subscriber in module_topics.items():
                        atom = subscriber.getAtomic()
                        if len(atom.value) != 64 or atom.time < readings[0]['poseTime'] * 1e6:
                            raise RuntimeError(f'Missing/stale four-module struct array: {key}')
                        values = struct.unpack('<8d', atom.value)
                        if not finite(values):
                            raise RuntimeError(f'Nonfinite module states: {key}')
                        vectors = [(values[i] * math.cos(values[i+1]),
                                    values[i] * math.sin(values[i+1]))
                                   for i in range(0, 8, 2)]
                        if name == 'forward':
                            valid = all(abs(x - result[0]) < .3 and abs(y) < .2
                                        for x, y in vectors)
                        else:
                            # Positive yaw: left modules roll backward; front roll left.
                            valid = all(x * sx > .1 and y * sy > .1
                                        for (x, y), (sx, sy) in zip(vectors,
                                            ((-1, 1), (1, 1), (-1, -1), (1, -1))))
                        evidence['checks'][key + '_' + name + '_vectors'] = valid
                if args.expect_jolt and name in ('forward', 'reverse', 'turn', 'negative_turn', 'spin_reversal'):
                    for key in ('RobotTruth', 'Pose'):
                        if readings[-1]['structTimes'][key] <= readings[0]['structTimes'][key]:
                            raise RuntimeError(f'Missing/stale {key} struct during {name}')
                    if not all(finite(row['truth'], 3) for row in readings):
                        raise RuntimeError(f'Missing/nonfinite truth during {name}')
                tracking = {}
                for key in ('requestedSpeeds', 'moduleStates', 'moduleTargets'):
                    tracking[key] = readings[-1][key]
                if finite(tracking['requestedSpeeds'], 3):
                    tracking['chassisMeasuredMinusTarget'] = [a-b for a,b in zip(readings[-1]['speeds'], tracking['requestedSpeeds'])]
                if finite(tracking['moduleStates'], 8) and finite(tracking['moduleTargets'], 8):
                    a,b = tracking['moduleStates'],tracking['moduleTargets']
                    tracking['moduleVectorErrorMps'] = [math.hypot(a[i]*math.cos(a[i+1])-b[i]*math.cos(b[i+1]),
                                                               a[i]*math.sin(a[i+1])-b[i]*math.sin(b[i+1])) for i in range(0,8,2)]
                evidence['samples'][name] = {'speeds': result, 'trajectory': readings, 'targetMeasured': tracking}
                return result

            try:
                deadline = time.monotonic() + 15
                # Do not send any HAL input before matching the owned-process identity.
                while not instance.isConnected() or (args.run_token and token.get() != args.run_token):
                    if time.monotonic() > deadline:
                        raise RuntimeError('Local simulator readiness/identity timeout')
                    await asyncio.sleep(.05)
                evidence['runToken'] = token.get()
                await hold(reset=True)
                initial = await phase('disabled_initial', duration=1.)
                if args.expect_jolt:
                    def placed_in_field(name):
                        row = evidence['samples'][name]['trajectory'][-1]
                        return all(finite(row[key], 3) and
                                   abs(row[key][0] - 8) < .02 and
                                   abs(row[key][1] - 4) < .02 and
                                   abs(row[key][2]) < .02
                                   for key in ('truth', 'estimated'))
                    evidence['checks']['practice_start_field_frame'] = placed_in_field('disabled_initial')
                    await reset_scene()
                    await hold(enabled=True, intake=True)
                    captured = held.get() == 1
                    await hold(intake=True)  # Disable without releasing the held button.
                    disabled_inactive = not intake_active.get()
                    await reset_scene(intake=True)
                    await hold(intake=True)
                    disabled_clear = held.get() == 0
                    await hold()  # Release, then use a new rising edge after enabling.
                    await hold(enabled=True, intake=True)
                    recaptured = held.get() == 1
                    await hold(enabled=True, launch=True)
                    launched = held.get() == 0
                    await hold(enabled=True, launch=True)
                    evidence['checks'].update(intake_captures=captured, disable_held_intake_stops=disabled_clear and disabled_inactive,
                                              intake_recaptures=recaptured, launch_releases_one=launched)
                await reset_scene()
                if args.expect_jolt:
                    await phase('practice_reset', duration=1.)
                    evidence['checks']['practice_reset_field_frame'] = placed_in_field('practice_reset')
                # Keep the tracking window inside the neutral zone, clear of hubs.
                translation_duration = 1. if args.expect_jolt else 2.
                forward = await phase('forward', enabled=True, forward=.5, duration=translation_duration)
                await reset_scene()
                reverse = await phase('reverse', enabled=True, forward=-.5, duration=translation_duration)
                await reset_scene()
                turn = await phase('turn', enabled=True, turn=.5)
                negative = await phase('negative_turn', enabled=True, turn=-.5)
                reversal = await phase('spin_reversal', enabled=True, turn=.5)
                stopping = await phase('stopping', enabled=True)
                stopped = await phase('disabled_final')
                autonomous = await phase('autonomous_no_choice', duration=.5, enabled=True, autonomous=True, forward=.5)
                evidence['checks']['autonomous_without_choice_does_not_drive'] = max(abs(v) for v in autonomous) < .03
                forward_rows = evidence['samples']['forward']['trajectory']
                evidence['checks']['forward_pose_changes'] = forward_rows[-1]['estimated'][0] - forward_rows[0]['estimated'][0] > .3
                if args.expect_jolt:
                    evidence['checks']['forward_truth_moves'] = forward_rows[-1]['truth'][0] - forward_rows[0]['truth'][0] > .3
                expected = args.max_speed * 4 / 9
                evidence['checks'].update(
                    initial_disabled_stationary=max(abs(v) for v in initial) < .1,
                    forward_speed_tracking=abs(forward[0] - expected) < expected * .1,
                    reverse_speed_tracking=abs(reverse[0] + expected) < expected * .1,
                    turn_speed_tracking=abs(turn[2] - args.max_angular_rate * 4 / 9) < args.max_angular_rate * 4 / 9 * .1,
                    negative_turn_velocity=negative[2] < -.2, spin_reversal_velocity=reversal[2] > .2,
                    zero_input_stops=max(abs(v) for v in stopping) < .15,
                    final_disabled_stationary=max(abs(v) for v in stopped) < .1)
                for name in ('turn', 'negative_turn', 'spin_reversal', 'stopping'):
                    rows = evidence['samples'][name]['trajectory']
                    for source, stamp in (('estimated', 'poseTime'), ('truth', 'truthTime')):
                        unique = []
                        for row in rows:
                            if len(row[source]) == 3 and (not unique or row[stamp] > unique[-1][stamp]):
                                if not finite(row[source], 3):
                                    raise RuntimeError(f'Nonfinite {source} during {name}')
                                unique.append(row)
                        if source == 'truth' and not args.expect_jolt:
                            continue
                        if len(unique) < 2 and name != 'stopping':
                            raise RuntimeError(f'Missing/stale {source} during {name}')
                        delta = sum(math.remainder(b[source][2] - a[source][2], 2 * math.pi)
                                    for a, b in zip(unique, unique[1:]))
                        evidence['samples'][name][source + 'HeadingChange'] = delta
                    integral = sum((b['simulationTime'] - a['simulationTime']) * (a['speeds'][2] + b['speeds'][2]) / 2
                                   for a, b in zip(rows, rows[1:]))
                    evidence['checks'][name + '_heading_matches_rate'] = abs(evidence['samples'][name]['estimatedHeadingChange'] - integral) < .35
                if args.expect_jolt:
                    # Static floor geometry is published once, with retained data.
                    payload = collider_outline.get()
                    count = decoded_poses(payload)
                    values = struct.unpack('<' + 'd' * (len(payload) // 8), payload)
                    vertices = [tuple(values[i:i+3]) for i in range(0, len(values), 7)]
                    evidence['checks']['collider_wireframe_struct'] = (
                        count == 16 and len(set(vertices)) == 8 and
                        all(sum(abs(x-y) > 1e-8 for x, y in zip(a, b)) == 1
                            for a, b in zip(vertices, vertices[1:])))
                    for name, subscriber in structs.items():
                        atom = subscriber.getAtomic()
                        count = int(len(atom.value) == 24) if name == 'Pose' else decoded_poses(atom.value)
                        evidence['checks'][name + '_struct_present'] = atom.time > 0 and (count > 0 or name == 'Pieces')
                    evidence['checks']['truth_positive_spin'] = evidence['samples']['turn']['truthHeadingChange'] > .5
                    evidence['checks']['truth_negative_spin'] = evidence['samples']['negative_turn']['truthHeadingChange'] < -.5
                if args.expect_jolt:
                    power_rows = [row for sample in evidence['samples'].values() for row in sample['trajectory']]
                    evidence['checks']['power_finite_and_physical'] = all(
                        finite([row['batteryVoltage'], row['supplyCurrent']]) and
                        0 <= row['batteryVoltage'] <= 12 and row['supplyCurrent'] >= 0 for row in power_rows)
                    loaded_voltage = min(row['batteryVoltage'] for row in power_rows)
                    evidence['checks']['loaded_battery_sags'] = loaded_voltage < 11.9
                    evidence['checks']['disabled_battery_recovers'] = evidence['samples']['disabled_final']['trajectory'][-1]['batteryVoltage'] > loaded_voltage + .05
                evidence['passed'] = all(evidence['checks'].values())
            finally:
                # Identity mismatch must never result in controls sent to another process.
                if not args.run_token or token.get() == args.run_token:
                    await control()
                reader.cancel()
                with contextlib.suppress(asyncio.CancelledError):
                    await reader
    finally:
        instance.stopClient()
    return evidence


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--hal-port', type=int, default=3300)
    parser.add_argument('--nt-port', type=int, default=5810)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--expect-jolt', action='store_true')
    parser.add_argument('--run-token')
    parser.add_argument('--max-speed', type=float, default=4.572)
    parser.add_argument('--max-angular-rate', type=float, default=7.)
    args = parser.parse_args()
    try:
        result = asyncio.run(exercise(args))
    except Exception as exception:
        result = {'passed': False, 'error': str(exception)}
    rendered = json.dumps(result, indent=2)
    print(rendered)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + '\n')
    return 0 if result.get('passed') else 1


if __name__ == '__main__':
    sys.exit(main())
