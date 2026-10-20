import contextlib
import copy
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location(
    'sim_benchmark', Path(__file__).resolve().parents[1] / 'sim_benchmark.py')
benchmark = importlib.util.module_from_spec(spec)
spec.loader.exec_module(benchmark)


def measurements():
    return [dict(mode=mode, scene=scene, medianSeconds=.001, p99Seconds=.005,
                 samples=100, deadlineMisses=2)
            for mode in ('jolt-planar', 'jolt-3d')
            for scene in ('empty', 'typical', 'dense')]


class BenchmarkContract(unittest.TestCase):
    def test_reporting_preserves_failed_budget_and_strict_default(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'CMakeCache.txt').write_text('CMAKE_CXX_COMPILER:FILEPATH=test\n')
            output = root / 'report.json'
            for flags, expected in [([], 1), (['--report-only'], 0)]:
                with self.subTest(flags=flags), patch.object(sys, 'argv', [
                        'benchmark', '--build-dir', directory, '--output', str(output), *flags]), \
                        patch.object(benchmark.subprocess, 'check_output', return_value=json.dumps(measurements())), \
                        patch.object(benchmark, 'capture', return_value='test'), \
                        contextlib.redirect_stdout(io.StringIO()):
                    self.assertEqual(benchmark.main(), expected)
                    report = json.loads(output.read_text())
                    self.assertFalse(report['acceptance']['passed'])
                    self.assertEqual(report['results'], measurements())

    def test_incomplete_and_invalid_measurements_are_rejected(self):
        invalid = [[], measurements()[:-1]]
        for field, value in [('p99Seconds', float('nan')), ('medianSeconds', -1),
                             ('samples', 0), ('deadlineMisses', 101), ('scene', 'unknown')]:
            rows = copy.deepcopy(measurements())
            rows[0][field] = value
            invalid.append(rows)
        for rows in invalid:
            with self.subTest(rows=rows), self.assertRaises(ValueError):
                benchmark.validate_results(rows)

    def test_report_only_does_not_hide_benchmark_process_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'CMakeCache.txt').write_text('')
            with patch.object(sys, 'argv', ['benchmark', '--build-dir', directory,
                                           '--output', str(root / 'report.json'), '--report-only']), \
                    patch.object(benchmark.subprocess, 'check_output',
                                 side_effect=subprocess.CalledProcessError(1, 'benchmark')), \
                    self.assertRaises(subprocess.CalledProcessError):
                benchmark.main()


if __name__ == '__main__':
    unittest.main()
