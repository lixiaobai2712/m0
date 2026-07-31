#!/usr/bin/env python3
"""
TRACK mode PID auto-tuner for MSPM0 ball-balancing car.

Connects via Bluetooth serial, watches ball position telemetry,
and iteratively adjusts KP / KD / KI to minimise oscillation and
steady-state error.

Usage:
    python auto_tune_pid.py COM7           # auto-detect baud (115200)
    python auto_tune_pid.py COM7 --baud 115200
    python auto_tune_pid.py COM7 --dry-run # don't send commands, just log

How it works:
  1.  Connects, checks TRACK? is responding.
  2.  Sends TRACK START, waits for BALANCE ACTIVE.
  3.  Enables TRACK DEBUG to get periodic CSV-like telemetry.
  4.  Each tuning round (default 5 s): collects ball X, computes metrics.
  5.  Adjusts one parameter per round based on oscillation / error.
  6.  Repeats until health score stabilises or max rounds reached.

Press Ctrl+C at any time to stop and keep the last parameters.
"""

import serial
import serial.tools.list_ports
import re
import sys
import time
import argparse
from collections import deque
from datetime import datetime


# ─── Defaults ──────────────────────────────────────────────────────────
DEFAULT_BAUD = 115200
ROUND_DURATION_S = 5          # seconds of data per tuning round
MAX_ROUNDS = 20               # safety limit
SETTLE_S = 3                  # wait after param change before collecting

# Parameter ranges (×100 gain form: larger = stronger)
KP_RANGE = (1, 500)       # KP position gain ×100
KD_RANGE = (1, 2000)      # KP velocity gain ×100
KI_RANGE = (0, 500)       # I divisor (smaller = stronger)

# Health thresholds
GOOD_HEALTH = 75              # stop if health >= this for 3 consecutive rounds


class PIDTuner:
    def __init__(self, port, baud=DEFAULT_BAUD, dry_run=False, debug=False):
        self.port = port
        self.baud = baud
        self.dry_run = dry_run
        self.debug = debug
        self.ser = None
        self.kp = None
        self.kd = None
        self.ki = None
        self.max_tilt = None
        self.round_data = []

    # ─── Serial helpers ────────────────────────────────────────────────

    def connect(self):
        print(f"Connecting to {self.port} @ {self.baud}...")
        self.ser = serial.Serial(self.port, self.baud, timeout=0.5)
        self.ser.dtr = False
        self.ser.rts = False
        # Bluetooth modules need time to sync after DTR toggle
        time.sleep(2)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        print("  Connected, waiting for MCU...")
        # Send a harmless query to wake up the link
        for attempt in range(5):
            self.ser.write(b"\r\n")
            time.sleep(0.3)
            # Check if we got any response
            junk = self._read_raw(0.5)
            if any('# ERROR' in l or 'TRACK' in l or 'GYRO' in l or 'CAM' in l for l in junk):
                print(f"  MCU responding (attempt {attempt+1})")
                break
        else:
            print("  WARNING: no response from MCU — check port/power")
        self.drain()

    def _read_raw(self, timeout_s=0.5):
        """Read all available lines without filtering, for debugging."""
        lines = []
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if self.ser.in_waiting:
                try:
                    raw = self.ser.readline()
                    line = raw.decode('utf-8', errors='replace').strip()
                    if line:
                        lines.append(line)
                except Exception:
                    pass
            else:
                time.sleep(0.02)
        return lines

    def send(self, cmd):
        """Send a command and return the response line(s)."""
        full = cmd + "\r\n"
        self.ser.reset_input_buffer()
        if not self.dry_run:
            self.ser.write(full.encode())
        if self.debug:
            print(f"  >>> {cmd}")
        time.sleep(0.3)  # Bluetooth needs more latency
        return self.read_response()

    def read_response(self, timeout=2.0):
        """Read all available lines, return as list."""
        lines = self._read_raw(timeout)
        if self.debug:
            for l in lines:
                print(f"       {l}")
        return lines

    def drain(self):
        """Discard buffered serial data."""
        self.ser.reset_input_buffer()
        time.sleep(0.1)

    # ─── High-level commands ───────────────────────────────────────────

    def read_params(self):
        """Parse KP/KD/KI/MAXTILT from TRACK? response. Retries on failure."""
        for attempt in range(3):
            lines = self.send("TRACK?")
            for line in lines:
                print(f"  {line}")
                # Parse each field independently (order varies)
                m_kp = re.search(r'KPP=(\d+)', line)
                m_kd = re.search(r'KPV=(\d+)', line)
                m_ki = re.search(r'KI=(\d+)', line)
                m_mx = re.search(r'MX=(\d+)', line)
                if m_kp and m_kd and m_ki:
                    self.kp = int(m_kp.group(1))
                    self.kd = int(m_kd.group(1))
                    self.ki = int(m_ki.group(1))
                    self.max_tilt = int(m_mx.group(1)) if m_mx else 20
                    print(f"  Current: KP={self.kp} KD={self.kd} KI={self.ki} MX={self.max_tilt}")
                    return True
            if attempt < 2:
                print(f"  Retry {attempt+2}/3...")
                time.sleep(1)
        print("  WARNING: could not parse TRACK? response after 3 attempts")
        return False

    def set_kp(self, val):
        val = max(val, KP_RANGE[0])  # never send 0 (division by zero on MCU)
        self.kp = val
        self.send(f"TRACK KPP {val}")

    def set_kd(self, val):
        val = max(val, 1)
        self.kd = val
        self.send(f"TRACK KPV {val}")

    def set_ki(self, val):
        val = max(val, 0)
        self.ki = val
        self.send(f"TRACK KI {val}")

    def start_track(self):
        """Start TRACK mode and wait for active control."""
        self.drain()
        # Send TRACK START without draining the initial response
        self.ser.write(b"TRACK START\r\n")
        print("  >>> TRACK START")
        time.sleep(0.3)

        # Continuously read until BALANCE ACTIVE or timeout
        deadline = time.time() + 35  # camera timeout is 10s, be generous
        last_dot = time.time()
        while time.time() < deadline:
            if self.ser.in_waiting:
                try:
                    raw = self.ser.readline()
                    line = raw.decode('utf-8', errors='replace').strip()
                    if line:
                        print(f"  {line}")
                        if "BALANCE ACTIVE" in line:
                            return True
                        if "TIMEOUT" in line:
                            print("  ! TRACK reported timeout")
                            return False
                        last_dot = time.time()
                except Exception:
                    pass
            else:
                time.sleep(0.05)
                # Print a dot every 2s to show we're alive
                if time.time() - last_dot > 2:
                    print("  .")
                    last_dot = time.time()
        return False

    def stop_track(self):
        self.send("TRACK STOP")
        time.sleep(0.3)

    def enable_debug(self):
        """Ensure TRACK DEBUG is ON. Toggle until confirmed."""
        for attempt in range(4):
            lines = self.send("TRACK DEBUG")
            for line in lines:
                print(f"  {line}")
                if "DEBUG ON" in line:
                    return True
            time.sleep(0.2)
        # If we can't confirm, assume it's on (toggle worked)
        print("  (debug state unknown, assuming ON)")
        return True

    # ─── Data collection ────────────────────────────────────────────────

    def collect_round(self, duration_s=ROUND_DURATION_S):
        """Collect ball X position data for `duration_s` seconds.
        Parses '# DBG X=...' lines from serial."""
        data = []
        other_lines = []
        deadline = time.time() + duration_s
        print(f"  Collecting {duration_s}s of data...", end='', flush=True)

        # Use non-blocking read with short timeout
        self.ser.timeout = 0.1
        while time.time() < deadline:
            try:
                raw = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode('utf-8', errors='replace').strip()
            except Exception:
                continue

            if not line:
                continue

            # Parse: # DBG X=158 BALL=1 TILT=2000 KPP=50 KPV=300 KI=128 VEL=-2 INT=-4000 ...
            m = re.search(
                r'X=(-?\d+).*BALL=(\d+).*TILT=(-?\d+).*VEL=(-?\d+).*INT=(-?\d+)',
                line)
            if m:
                ball = int(m.group(2))
                if ball:
                    data.append({
                        'x': int(m.group(1)),
                        'tilt': int(m.group(3)),
                        'vel': int(m.group(4)),
                        'integral': int(m.group(5)),
                    })
            elif line.startswith('#') or line.startswith('$'):
                other_lines.append(line)

        # Restore timeout
        self.ser.timeout = 0.5
        print(f" {len(data)} samples", end='')
        if data:
            xs = [d['x'] for d in data]
            ts = [d['tilt'] for d in data]
            vs = [d['vel'] for d in data]
            print(f"  X:[{min(xs)}..{max(xs)}]"
                  f"  TILT:[{min(ts)}..{max(ts)}]"
                  f"  VEL:[{min(vs)}..{max(vs)}]")
            # Print first and last DBG sample for debugging
            if len(data) >= 2:
                d0 = data[0]
                d1 = data[-1]
                print(f"    first: X={d0['x']} TILT={d0['tilt']} VEL={d0['vel']} INT={d0['integral']}")
                print(f"    last:  X={d1['x']} TILT={d1['tilt']} VEL={d1['vel']} INT={d1['integral']}")
        elif other_lines:
            print(f"  (other: {len(other_lines)}, e.g. {other_lines[0][:80]})")
        else:
            print("  (no data — is TRACK DEBUG ON?)")
        self.round_data = data
        return data

    # ─── Metrics (adapted from KeilPIDTuner) ────────────────────────────

    def analyze(self, data):
        """Compute health metrics from one round of data."""
        n = len(data)
        if n < 10:
            return {"error": "too few samples", "health": 0}

        xs = [d['x'] for d in data]
        abs_xs = [abs(x) for x in xs]

        # 1. Mean absolute position (0 = perfectly centered)
        mean_abs = sum(abs_xs) / n

        # 2. RMS position
        rms = (sum(x * x for x in xs) / n) ** 0.5

        # 3. Oscillation rate: how often ball crosses center
        crossings = 0
        for i in range(1, n):
            if (xs[i] >= 0) != (xs[i - 1] >= 0):
                crossings += 1
        osc_rate = crossings / max(n, 1)

        # 4. Saturation: fraction of time at max tilt
        tilts = [d['tilt'] for d in data]
        max_t = self.max_tilt or 20
        sat_count = sum(1 for t in tilts if abs(t) >= max_t * 0.9)
        sat_pct = sat_count / max(n, 1) * 100

        # 5. Settling: std dev of last 20% of samples
        tail = max(n // 5, 5)
        tail_xs = xs[-tail:]
        tail_std = (sum((x - sum(tail_xs) / tail) ** 2 for x in tail_xs) / tail) ** 0.5

        # 6. Integral windup
        integrals = [d['integral'] for d in data]
        integ_max = max(abs(i) for i in integrals)
        integ_winding = integ_max > 3000

        # Scores (0 = perfect, 100 = terrible)
        osc_score = min(osc_rate * 400, 100)
        dev_score = min(rms * 0.5, 100)
        sat_score = sat_pct
        tail_score = min(tail_std * 2, 100)

        health = max(0, 100 - (osc_score + dev_score + sat_score + tail_score) / 4)

        return {
            "samples": n,
            "mean_abs": round(mean_abs, 1),
            "rms": round(rms, 1),
            "zero_crossings": crossings,
            "osc_rate": round(osc_rate, 4),
            "sat_pct": round(sat_pct, 1),
            "tail_std": round(tail_std, 1),
            "integral_max": integ_max,
            "integral_winding": integ_winding,
            "osc_score": round(osc_score, 1),
            "dev_score": round(dev_score, 1),
            "sat_score": round(sat_score, 1),
            "tail_score": round(tail_score, 1),
            "health": round(health, 1),
        }

    # ─── Tuning logic ───────────────────────────────────────────────────
    #
    # IMPORTANT: our system uses DIVISOR parameters.
    #   target = ball_x * 100 / KP_divisor
    # A SMALLER divisor = STRONGER response (more degrees per pixel).
    # A LARGER  divisor = WEAKER response.
    #
    # Tuning rules for ball-on-plate:
    #   - Ball oscillates (crosses center frequently): KD too weak →
    #     decrease KD divisor (stronger damping)
    #   - Ball can't reach center, stuck at edge despite max tilt:
    #     mechanical/sign problem, or KP too weak → decrease KP divisor
    #   - Ball overshoots and gets stuck on other side: too much momentum,
    #     KD too weak → decrease KD divisor
    #   - Integral windup: KI accumulating too fast → increase KI divisor

    def suggest(self, metrics):
        """Return list of (param, new_value, reason) tuples.
        New semantics: KP/KPV are GAINS (larger = stronger).
        KI is still a divisor (smaller = stronger)."""
        m = metrics
        changes = []

        # Priority 1: SEVERE oscillation → reduce KP, increase KPV
        if m['osc_score'] > 50:
            new_kp = max(int(self.kp * 0.7), KP_RANGE[0])
            changes.append(('KP', new_kp,
                f"severe oscillation ({m['osc_score']:.0f}/100)→weaker KP"))
            new_kd = min(int(self.kd * 1.5), KD_RANGE[1])
            changes.append(('KD', new_kd,
                f"stronger velocity damping"))

        # Priority 2: moderate oscillation → increase KPV
        elif m['osc_score'] > 25:
            new_kd = min(int(self.kd * 1.3), KD_RANGE[1])
            changes.append(('KD', new_kd,
                f"moderate oscillation→stronger damping"))

        # Priority 3: saturated + ball not centered → increase KP
        if m['sat_score'] > 60 and m['mean_abs'] > 20 and not changes:
            new_kp = min(int(self.kp * 1.3), KP_RANGE[1])
            changes.append(('KP', new_kp,
                f"saturated {m['sat_score']:.0f}%,ball@{m['mean_abs']:.0f}px→stronger KP"))

        # Priority 4: integral windup → larger KI divisor
        if m['integral_winding'] and not changes:
            new_ki = min(int(self.ki * 1.5), KI_RANGE[1])
            changes.append(('KI', new_ki, "integral windup→slower I"))

        # Priority 5: steady error, no oscillation → increase KP
        if m['mean_abs'] > 30 and m['osc_score'] < 15 and not changes:
            new_kp = min(int(self.kp * 1.2), KP_RANGE[1])
            changes.append(('KP', new_kp,
                f"steady error {m['mean_abs']:.0f}px→stronger KP"))

        # Safety clamps
        clamped = []
        for p, v, r in changes:
            if p == 'KP':   v = max(v, KP_RANGE[0])
            elif p == 'KD': v = max(v, 1)
            elif p == 'KI': v = max(v, 0)
            clamped.append((p, v, r))
        changes = clamped

        # No-ops filter
        changes = [(p, v, r) for p, v, r in changes
                   if (p == 'KP' and v != self.kp) or
                      (p == 'KD' and v != self.kd) or
                      (p == 'KI' and v != self.ki)]

        return changes

    def apply_change(self, param, value):
        if param == 'KP':
            self.set_kp(value)
        elif param == 'KD':
            self.set_kd(value)
        elif param == 'KI':
            self.set_ki(value)

    # ─── Main loop ──────────────────────────────────────────────────────

    def run(self, rounds=MAX_ROUNDS):
        print("=" * 60)
        print("PID Auto-Tuner for TRACK Ball Balance")
        print("=" * 60)

        self.connect()

        # Read current params
        if not self.read_params():
            print("ERROR: cannot communicate with MCU")
            return

        # Start TRACK mode
        print("\nStarting TRACK mode...")
        if not self.start_track():
            print("ERROR: TRACK did not reach BALANCE ACTIVE")
            return

        # Enable debug output
        self.enable_debug()
        time.sleep(0.5)
        self.drain()

        best_health = 0
        best_params = (self.kp, self.kd, self.ki)
        stable_count = 0

        print(f"\n{'='*60}")
        print(f"Round   KP   KD   KI  | Health  Osc  Dev  Sat  Tail")
        print(f"{'='*60}")

        for r in range(1, rounds + 1):
            # Wait for system to settle after last param change
            if r > 1:
                print(f"  Settling {SETTLE_S}s...")
                time.sleep(SETTLE_S)

            # Collect data
            data = self.collect_round()
            if len(data) < 10:
                print(f"  Round {r}: insufficient data, skipping")
                continue

            # Analyze
            metrics = self.analyze(data)
            health = metrics['health']

            print(f"  {r:3d}   {self.kp:4d} {self.kd:4d} {self.ki:4d}"
                  f"  |  {health:5.1f}  {metrics['osc_score']:4.1f}"
                  f"  {metrics['dev_score']:4.1f}  {metrics['sat_score']:4.1f}"
                  f"  {metrics['tail_score']:4.1f}")

            # Track best
            if health > best_health:
                best_health = health
                best_params = (self.kp, self.kd, self.ki)

            # Stop condition: health good for 3 consecutive rounds
            if health >= GOOD_HEALTH:
                stable_count += 1
                if stable_count >= 3:
                    print(f"\n  Health >= {GOOD_HEALTH} for 3 rounds, converged!")
                    break
            else:
                stable_count = 0

            # Get suggestions
            changes = self.suggest(metrics)
            if not changes:
                print(f"  No changes suggested, converged.")
                break

            # Apply first change
            param, value, reason = changes[0]
            print(f"  → {param} {self.kp if param=='KP' else self.kd if param=='KD' else self.ki}"
                  f" → {value}  ({reason})")
            self.apply_change(param, value)

        # Done
        print(f"\n{'='*60}")
        print(f"Tuning complete.")
        print(f"Best:  KP={best_params[0]}  KD={best_params[1]}  KI={best_params[2]}"
              f"  (health={best_health:.1f})")
        print(f"To save: TRACK KPP{best_params[0]}; TRACK KPV {best_params[1]};"
              f" TRACK KI {best_params[2]}")
        print(f"{'='*60}")

        # Stop track mode
        self.stop_track()
        self.ser.close()


# ─── CLI ────────────────────────────────────────────────────────────────

def list_ports():
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No COM ports found.")
        return
    print("Available COM ports:")
    for p in ports:
        print(f"  {p.device} - {p.description}")


def main():
    parser = argparse.ArgumentParser(
        description="PID auto-tuner for MSPM0 ball-balancing car (TRACK mode)")
    parser.add_argument("port", nargs="?", help="COM port (e.g. COM7)")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD,
                        help=f"Baud rate (default: {DEFAULT_BAUD})")
    parser.add_argument("--rounds", type=int, default=MAX_ROUNDS,
                        help=f"Max tuning rounds (default: {MAX_ROUNDS})")
    parser.add_argument("--dry-run", action="store_true",
                        help="Don't send commands, just read and log")
    parser.add_argument("--debug", action="store_true",
                        help="Print all raw serial data")
    parser.add_argument("--list", action="store_true",
                        help="List COM ports and exit")
    args = parser.parse_args()

    if args.list:
        list_ports()
        return

    if not args.port:
        list_ports()
        print("\nUsage: python auto_tune_pid.py <COM_PORT>")
        return

    tuner = PIDTuner(args.port, args.baud, dry_run=args.dry_run, debug=args.debug)
    try:
        tuner.run(rounds=args.rounds)
    except KeyboardInterrupt:
        print("\n\nInterrupted. Stopping TRACK...")
        tuner.stop_track()
        tuner.ser.close()
        print(f"Last params: KP={tuner.kp} KD={tuner.kd} KI={tuner.ki}")
    except Exception as e:
        print(f"\nERROR: {e}")
        if tuner.ser and tuner.ser.is_open:
            tuner.ser.close()


if __name__ == "__main__":
    main()
