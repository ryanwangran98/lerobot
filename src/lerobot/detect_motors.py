#!/usr/bin/env python
"""
Motor Detection Script

This script scans all available USB serial ports to detect connected motors.
Helpful for finding which ports have which motors.

Usage:
    PYTHONPATH=src python examples/detect_motors.py
"""

import glob
import sys
from lerobot.motors.feetech import FeetechMotorsBus


def scan_port(port):
    """Scan a single USB port for motors."""
    try:
        bus = FeetechMotorsBus(port=port, motors={})
        found_motors = bus.broadcast_ping(raise_on_error=False)
        
        if found_motors:
            print(f"  ✓ Found {len(found_motors)} motor(s):")
            for motor_id in sorted(found_motors.keys()):
                model = found_motors[motor_id]
                print(f"      ID {motor_id}: Model {model}")
            return found_motors
        else:
            print(f"  ✗ No motors found")
            return None
    except Exception as e:
        print(f"  ✗ Error: {e}")
        return None


def main():
    print("="*60)
    print("Motor Detection Tool")
    print("="*60)
    print("\nScanning all USB serial ports...")
    print()
    
    # Common USB serial port patterns
    port_patterns = [
        "/dev/ttyACM*",
        "/dev/ttyUSB*",
        "/dev/tty.usbmodem*",  # MacOS
        "/dev/tty.usbserial*",  # MacOS
    ]
    
    all_ports = set()
    for pattern in port_patterns:
        all_ports.update(glob.glob(pattern))
    
    if not all_ports:
        print("No USB serial ports found!")
        print("Make sure your robot is connected via USB.")
        return
    
    # Sort ports for consistent output
    sorted_ports = sorted(all_ports)
    
    all_found_motors = {}
    
    for port in sorted_ports:
        print(f"[{port}]")
        found_motors = scan_port(port)
        if found_motors:
            all_found_motors[port] = found_motors
        print()
    
    # Summary
    print("="*60)
    print("Summary")
    print("="*60)
    
    if not all_found_motors:
        print("\n✗ No motors detected on any port!")
        print("\nTroubleshooting:")
        print("  1. Check USB cables are properly connected")
        print("  2. Check robot power is on")
        print("  3. Try different USB ports")
        return
    
    total_motors = sum(len(motors) for motors in all_found_motors.values())
    print(f"\nTotal ports with motors: {len(all_found_motors)}")
    print(f"Total motors found: {total_motors}")
    
    print("\nDetailed breakdown:")
    for port, motors in all_found_motors.items():
        motor_ids = sorted(motors.keys())
        motor_ids_str = ", ".join(f"ID {mid}" for mid in motor_ids)
        print(f"  {port}: {len(motors)} motors [{motor_ids_str}]")
    
    print("\n" + "="*60)
    print("Recommendations:")
    print("="*60)
    
    # Analyze configuration
    if len(all_found_motors) == 1:
        port = list(all_found_motors.keys())[0]
        motors = all_found_motors[port]
        if len(motors) == 9:
            print("\n✓ Single port with 9 motors detected")
            print(f"  This matches XLerobotSingleArm configuration")
            print(f"  Use port1={port} in config")
        elif len(motors) == 6:
            print("\n⚠ Single port with 6 motors detected")
            print(f"  This looks like an arm-only configuration")
            print(f"  If you also have a base, check:")
            print(f"    - Base power connection")
            print(f"    - Base USB connection")
            print(f"    - Base motor wiring")
        elif len(motors) == 3:
            print("\n⚠ Single port with 3 motors detected")
            print(f"  This looks like a base-only configuration")
            print(f"  If you also have an arm, check arm connection")
        else:
            print(f"\n? Single port with {len(motors)} motors detected")
            print(f"  Motor count doesn't match standard configurations")
    
    elif len(all_found_motors) == 2:
        ports = sorted(all_found_motors.keys())
        port1_motors = all_found_motors[ports[0]]
        port2_motors = all_found_motors[ports[1]]
        
        print("\n✓ Two ports with motors detected")
        print(f"  {ports[0]}: {len(port1_motors)} motors")
        print(f"  {ports[1]}: {len(port2_motors)} motors")
        
        # Check if this matches dual-arm configuration
        total = len(port1_motors) + len(port2_motors)
        if total == 17:  # Left(8) + Right+Base(9)
            print("\n  This matches dual-arm + base configuration")
            print(f"  Use standard XLerobotConfig with:")
            print(f"    port1={ports[0]}")
            print(f"    port2={ports[1]}")
        elif total == 12:  # Left(6) + Right(6)
            print("\n  This matches dual-arm configuration")
            print(f"  Consider using different configuration")
        else:
            print(f"\n  Total {total} motors - check your hardware setup")
    
    else:
        print(f"\n? {len(all_found_motors)} ports detected - check your hardware")


if __name__ == "__main__":
    main()
