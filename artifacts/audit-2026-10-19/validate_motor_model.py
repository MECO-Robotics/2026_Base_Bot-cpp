#!/usr/bin/env python3
"""Independent scalar-model validation; Python timings are not robot latency.
Run from repository root: python3 artifacts/audit-2026-10-19/validate_motor_model.py
Outputs JSON beside this script. No third-party dependencies or robot IO.
"""
import json
import math
from pathlib import Path
import random
import time

R = 12 / 105
KT = 2.6 / 105
KV = 5676 * 2 * math.pi / 60 / 12
DT = .02


def exact(initial, voltage, ratio, inertia, limit, count):
    drag = .0001 / inertia
    torque = KT * count * ratio / inertia
    emf = ratio / KV
    speed, position = initial, 0.

    def advance(duration, decay, equilibrium):
        nonlocal speed, position
        integral = -math.expm1(-decay * duration) / decay
        position += equilibrium * duration + (speed - equilibrium) * integral
        speed += (equilibrium - speed) * (-math.expm1(-decay * duration))

    remaining = DT
    initial_current = (voltage - emf * speed) / R
    if abs(initial_current) > limit:
        current = math.copysign(limit, initial_current)
        equilibrium = torque * current / drag
        boundary = (voltage - R * current) / emf
        fraction = (boundary - speed) / (speed - equilibrium)
        crossing = -math.log1p(fraction) / drag if fraction > -1 else DT
        duration = min(DT, crossing) if math.isfinite(crossing) and crossing > 0 else DT
        advance(duration, drag, equilibrium)
        remaining -= duration
    if remaining > 0:
        decay = torque * emf / R + drag
        advance(remaining, decay, torque * voltage / R / decay)
    return speed, position


def reference(initial, voltage, ratio, inertia, limit, count):
    # Independent RK4 integration of the clipped-current differential equation.
    def acceleration(speed):
        current = max(-limit, min(limit, (voltage - ratio * speed / KV) / R))
        return (KT * current * count * ratio - .0001 * speed) / inertia

    speed, position = initial, 0.
    h = 1e-6
    for _ in range(20000):
        k1 = acceleration(speed)
        k2 = acceleration(speed + h * k1 / 2)
        k3 = acceleration(speed + h * k2 / 2)
        k4 = acceleration(speed + h * k3)
        position += h * (speed + 2 * (speed + h * k1 / 2)
                         + 2 * (speed + h * k2 / 2) + speed + h * k3) / 6
        speed += h * (k1 + 2 * k2 + 2 * k3 + k4) / 6
    return speed, position


def main():
    random.seed(8324)
    cases = []
    for _ in range(60):
        ratio = random.choice([1, 2, 10, 50, 200])
        inertia = random.choice([.005, .025, .5])
        limit = random.choice([10, 40, 100])
        count = random.choice([1, 2, 4])
        voltage = random.uniform(-12, 12)
        initial = random.uniform(-500, 500)
        cases.append((initial, voltage, ratio, inertia, limit, count))
    errors = []
    for case in cases:
        actual = exact(*case)
        expected = reference(*case)
        errors.append([abs(a - b) for a, b in zip(actual, expected)])
    iterations = 100000
    start = time.perf_counter_ns()
    for n in range(iterations):
        exact(*cases[n % len(cases)])
    elapsed = time.perf_counter_ns() - start
    result = {
        'scope': 'Independent Python transcription versus 1 us RK4; not execution of production C++',
        'seed': 8324, 'cases': len(cases), 'duration_seconds': DT,
        'reference_timestep_seconds': 1e-6,
        'max_velocity_error_rad_per_second': max(e[0] for e in errors),
        'max_position_error_rad': max(e[1] for e in errors),
        'python_analytical_kernel_iterations': iterations,
        'python_analytical_kernel_ns_per_iteration': elapsed / iterations,
        'performance_limit': 'Host Python microbenchmark; excludes C++, HAL, NetworkTables, scheduling and hardware; does not establish a 4 ms robot budget',
    }
    result['passed'] = (result['max_velocity_error_rad_per_second'] < 1e-6
                        and result['max_position_error_rad'] < 1e-7)
    output = json.dumps(result, indent=2) + '\n'
    Path(__file__).with_name('motor-model-validation.json').write_text(output)
    print(output, end='')
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
