#!/usr/bin/env python3
"""Audit C++ sensor-input replay logs; never reenact inputs over NT or HAL sockets."""
import argparse
import hashlib
import json
import math
from pathlib import Path


def _finite(value):
    if isinstance(value, float) and not math.isfinite(value):
        raise ValueError('Nonfinite replay value')
    if isinstance(value, dict):
        for child in value.values():
            _finite(child)
    elif isinstance(value, list):
        for child in value:
            _finite(child)


def load_recording(path):
    from wpiutil.log import DataLogReader
    path = Path(path)
    reader = DataLogReader(str(path))
    if not reader.isValid():
        raise ValueError('Invalid WPILOG file')
    entries, frames, metadata, complete = {}, [], None, False
    recording_status = None
    for record in reader:
        if record.isStart():
            start = record.getStartData()
            entries[start.entry] = (start.name, start.type)
            continue
        if record.isControl():
            continue
        name, kind = entries.get(record.getEntry(), ('', ''))
        if name.startswith('/'):
            name = name[1:]
        if name == 'Replay/RecordingFault':
            raise ValueError('Recording reported overflow or serialization failure')
        if name not in ('Replay/Frames', 'Replay/Metadata', 'Replay/Complete', 'Replay/RecordingStatus'):
            continue
        if kind != 'string':
            raise ValueError(f'{name} must be a string entry')
        if name == 'Replay/Complete':
            complete = bool(record.getString())
            continue
        value = json.loads(record.getString())
        _finite(value)
        if name == 'Replay/RecordingStatus':
            recording_status = value
            if value.get('rejected', 0) or value.get('failed', 0):
                raise ValueError('Recording worker reported rejected or failed frames')
            continue
        if name == 'Replay/Metadata':
            metadata = value
            continue
        if value.get('schemaVersion') != 1:
            raise ValueError('Unsupported replay frame schema')
        if value.get('sequence') != len(frames):
            raise ValueError('Replay frame sequence is not contiguous')
        stamp = value.get('timestampUs')
        if type(stamp) is not int or stamp < 0 or (frames and stamp <= frames[-1]['timestampUs']):
            raise ValueError('Replay timestamps must strictly increase')
        if not all(isinstance(value.get(key), dict) for key in ('inputs', 'outputs', 'ds')):
            raise ValueError('Invalid replay frame structure')
        frames.append(value)
    if not frames or not isinstance(metadata, dict) or metadata.get('schemaVersion') != 1:
        raise ValueError('Log lacks schema-versioned sensor input frames and metadata; legacy controller logs cannot be replayed')
    if recording_status is not None:
        written = recording_status.get('written')
        final = recording_status.get('final', False)
        if type(written) is not int or written < 0:
            raise ValueError('Recorder status has an invalid written-frame count')
        if (final or complete) and written != len(frames):
            raise ValueError(
                f"Persisted frame count {len(frames)} does not match final recorder status "
                f"({written} frames written)"
            )
    return {'frames': frames, 'metadata': metadata, 'complete': complete, 'recordingStatus': recording_status,
            'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}


def audit_recordings(source, replayed):
    original, actual = source['frames'], replayed['frames']
    checks = {'complete': replayed['complete'], 'frameCount': len(actual) == len(original),
              'sourceHash': replayed['metadata'].get('sourceFileSha256') == source['sha256'],
              'replayMode': replayed['metadata'].get('mode') == 'replay',
              'sourceMetadata': replayed['metadata'].get('sourceMetadata') == source['metadata']}
    checks['inputsAndClockPreserved'] = checks['frameCount'] and all(
        all(a[key] == b[key] for key in ('sequence', 'timestampUs', 'ds', 'inputs'))
        for a, b in zip(original, actual))
    checks['outputsRecorded'] = any(frame['outputs'] for frame in actual)
    differences = sum(a['outputs'] != b['outputs'] for a, b in zip(original, actual))
    difference_counts = {}
    for old, new in zip(original, actual):
        for key in old['outputs'].keys() | new['outputs'].keys():
            if old['outputs'].get(key) != new['outputs'].get(key):
                difference_counts[key] = difference_counts.get(key, 0) + 1
    # Equality is an observation, not a success criterion: changed robot code
    # should legitimately produce different outputs from identical inputs.

    return {'scope': 'recorded sensor/DS inputs injected into rerun C++ commands; Phoenix estimator and camera processing are recorded boundaries',
            'checks': checks, 'completed': all(checks.values()), 'frameCount': len(actual),
            'outputFramesDifferent': differences, 'outputDifferenceCounts': difference_counts, 'sourceSha256': source['sha256'],
            'replaySha256': replayed['sha256'], 'sourceMetadata': source['metadata'],
            'replayMetadata': replayed['metadata'], 'recordingStatus': replayed.get('recordingStatus')}


def audit(source_path, replay_path, output_path):
    report = audit_recordings(load_recording(source_path), load_recording(replay_path))
    report.update(source=str(Path(source_path).resolve()), replayLog=str(Path(replay_path).resolve()))
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, sort_keys=True))
    return 0 if report['completed'] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log', required=True, type=Path)
    parser.add_argument('--replayed-log', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    return audit(args.log, args.replayed_log, args.output)


if __name__ == '__main__':
    raise SystemExit(main())
