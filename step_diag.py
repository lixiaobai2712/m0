#!/usr/bin/env python3
"""
Stepper motor diagnostic tool.
Connects to car via Bluetooth serial, sends commands, logs all replies.
Usage: python step_diag.py COM23
"""

import sys
import time
import serial

def main():
    if len(sys.argv) < 2:
        # Try to find the port
        ports = list(serial.tools.list_ports.comports())
        print("Available ports:")
        for i, p in enumerate(ports):
            print(f"  [{i+1}] {p.device} - {p.description}")
        port = input("Enter COM port (e.g. COM23): ").strip()
    else:
        port = sys.argv[1]

    ser = serial.Serial(port, 115200, timeout=0.5)

    def send(cmd, wait=0.5):
        """Send command and print all responses received during wait period."""
        ser.reset_input_buffer()
        ser.write(f"{cmd}\r\n".encode())
        print(f"\n>>> {cmd}")
        deadline = time.time() + wait
        while time.time() < deadline:
            line = ser.readline()
            if line:
                text = line.decode("utf-8", errors="replace").strip()
                if text:
                    print(f"    {text}")
            else:
                time.sleep(0.02)

    print(f"Connected to {port}. Starting diagnostics...\n")
    time.sleep(0.5)

    # --- Test 1: Basic connectivity ---
    print("=" * 60)
    print("TEST 1: Basic connectivity (STEP? / CAM? / STATUS)")
    print("=" * 60)
    send("STEP?", 1.0)
    send("STATUS", 0.5)

    # --- Test 2: Wake and origin ---
    print("\n" + "=" * 60)
    print("TEST 2: Wake motor, set origin")
    print("=" * 60)
    send("STEP WAKE", 1.0)
    send("STEP ZERO", 1.0)

    # --- Test 3: Relative rotation ---
    print("\n" + "=" * 60)
    print("TEST 3: STEP 90 (large rotation, easy to see)")
    print("=" * 60)
    send("STEP 90", 2.0)
    print("    ^^^ Did motor shaft rotate ~90 degrees? Check physically.")
    input("\nPress Enter after checking...")

    # --- Test 4: Reverse direction ---
    print("\n" + "=" * 60)
    print("TEST 4: STEP -90 (reverse)")
    print("=" * 60)
    send("STEP -90", 2.0)

    # --- Test 5: Absolute position ---
    print("\n" + "=" * 60)
    print("TEST 5: STEP A 0 (return to origin)")
    print("=" * 60)
    send("STEP A 0", 2.0)

    # --- Test 6: Repeated small steps ---
    print("\n" + "=" * 60)
    print("TEST 6: STEP 10 five times (small increments)")
    print("=" * 60)
    for i in range(5):
        send("STEP 10", 0.8)

    # --- Summary ---
    print("\n" + "=" * 60)
    print("DIAGNOSTIC COMPLETE")
    print("=" * 60)
    print("Check: Did you see 'CMD 0x11 OK' or 'CMD 0x13 OK' replies?")
    print("If YES but motor doesn't move → mechanical / motor power issue")
    print("If NO replies at all → TX not reaching motor (wiring / pin config)")
    print("If 'ERR:' replies → protocol mismatch")

    ser.close()


if __name__ == "__main__":
    main()
