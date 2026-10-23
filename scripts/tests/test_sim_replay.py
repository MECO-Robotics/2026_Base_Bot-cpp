import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location(
    'sim_replay', Path(__file__).resolve().parents[1] / 'sim_replay.py')
sim_replay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sim_replay)


class StartData:
    def __init__(self, entry, name, kind):
        self.entry = entry
        self.name = name
        self.type = kind


class LogRecord:
    def __init__(self, entry=None, name=None, kind=None, value=None):
        self.entry = entry
        self.name = name
        self.kind = kind
        self.value = value

    def isStart(self):
        return self.name is not None

    def isControl(self):
        return False

    def getStartData(self):
        return StartData(self.entry, self.name, self.kind)

    def getEntry(self):
        return self.entry

    def getString(self):
        return self.value


class Reader:
    def __init__(self, records):
        self.records = records

    def isValid(self):
        return True

    def __iter__(self):
        return iter(self.records)


class ReplayLogPathTest(unittest.TestCase):
    def test_loads_wpilib_slash_prefixed_replay_entries(self):
        frame = {'schemaVersion': 1, 'sequence': 0, 'timestampUs': 100000,
                 'inputs': {}, 'outputs': {}, 'ds': {}}
        records = [
            LogRecord(1, '/Replay/Metadata', 'string'),
            LogRecord(2, '/Replay/Frames', 'string'),
            LogRecord(1, value=json.dumps({'schemaVersion': 1})),
            LogRecord(2, value=json.dumps(frame)),
        ]
        fake_log = types.ModuleType('wpiutil.log')
        fake_log.DataLogReader = lambda _: Reader(records)
        fake_wpiutil = types.ModuleType('wpiutil')
        fake_wpiutil.log = fake_log
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'recording.wpilog'
            path.write_bytes(b'test log')
            with patch.dict(sys.modules, {'wpiutil': fake_wpiutil, 'wpiutil.log': fake_log}):
                recording = sim_replay.load_recording(path)

        self.assertEqual(len(recording['frames']), 1)
        self.assertEqual(recording['metadata']['schemaVersion'], 1)

    def _load_records(self, records):
        fake_log = types.ModuleType('wpiutil.log')
        fake_log.DataLogReader = lambda _: Reader(records)
        fake_wpiutil = types.ModuleType('wpiutil')
        fake_wpiutil.log = fake_log
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'recording.wpilog'
            path.write_bytes(b'test log')
            with patch.dict(sys.modules, {'wpiutil': fake_wpiutil, 'wpiutil.log': fake_log}):
                return sim_replay.load_recording(path)

    def test_accepts_periodic_status_behind_persisted_live_frames(self):
        frame = {'schemaVersion': 1, 'sequence': 0, 'timestampUs': 100000,
                 'inputs': {}, 'outputs': {}, 'ds': {}}
        records = [
            LogRecord(1, '/Replay/Metadata', 'string'),
            LogRecord(2, '/Replay/Frames', 'string'),
            LogRecord(3, '/Replay/RecordingStatus', 'string'),
            LogRecord(1, value=json.dumps({'schemaVersion': 1})),
            LogRecord(2, value=json.dumps(frame)),
            LogRecord(2, value=json.dumps({**frame, 'sequence': 1, 'timestampUs': 120000})),
            LogRecord(3, value=json.dumps({'written': 1, 'rejected': 0, 'failed': 0})),
        ]

        recording = self._load_records(records)

        self.assertEqual(len(recording['frames']), 2)
        self.assertEqual(recording['recordingStatus']['written'], 1)

    def test_rejects_final_status_that_disagrees_with_persisted_frames(self):
        frame = {'schemaVersion': 1, 'sequence': 0, 'timestampUs': 100000,
                 'inputs': {}, 'outputs': {}, 'ds': {}}
        records = [
            LogRecord(1, '/Replay/Metadata', 'string'),
            LogRecord(2, '/Replay/Frames', 'string'),
            LogRecord(3, '/Replay/RecordingStatus', 'string'),
            LogRecord(1, value=json.dumps({'schemaVersion': 1})),
            LogRecord(2, value=json.dumps(frame)),
            LogRecord(3, value=json.dumps({'written': 0, 'rejected': 0, 'failed': 0, 'final': True})),
        ]

        with self.assertRaisesRegex(ValueError, 'final recorder status'):
            self._load_records(records)


if __name__ == '__main__':
    unittest.main()
