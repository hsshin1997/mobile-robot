
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Spin Dynamixel Motor for One Second
For motor ID 200 on /dev/ttyACM0
"""

import time
from .port_handler import *
from .packet_handler import *
from .group_sync_read import *
from .group_sync_write import *
from .group_bulk_read import *
from .group_bulk_write import *
import os
import sys
import argparse, glob, sys, time
import serial.tools.list_ports
from typing import Dict, Tuple


# Motor settings (from your scan results)
MOTOR_ID = 200
PORT_NAME = '/dev/ttyACM0'
BAUDRATE = 57600
PROTOCOL_VERSION = 2.0

# Control table addresses for Dynamixel X series
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_VELOCITY = 104
ADDR_PRESENT_VELOCITY = 128

# Operating modes
VELOCITY_CONTROL_MODE = 1

# Settings
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
SPIN_VELOCITY = 200  # Velocity units (positive = CCW, negative = CW)
SPIN_DURATION = 10.0  # seconds

def main():
    """
    Main function to spin motor for one second
    """
    print("=" * 50)
    print("Dynamixel Motor Control - Spin for 1 Second")
    print("=" * 50)
    print(f"Motor ID: {MOTOR_ID}")
    print(f"Port: {PORT_NAME}")
    print(f"Velocity: {SPIN_VELOCITY} (CCW)")
    print(f"Duration: {SPIN_DURATION} second")
    print("=" * 50)
    
    # Initialize PortHandler and PacketHandler
    portHandler = PortHandler(PORT_NAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)
    
    # Open port
    print("\n1. Opening port...")
    if not portHandler.openPort():
        print("   ❌ Failed to open port")
        print("   Try running with: sudo python3 this_script.py")
        return
    print("   ✅ Port opened")
    
    # Set baudrate
    print("\n2. Setting baudrate...")
    if not portHandler.setBaudRate(BAUDRATE):
        print("   ❌ Failed to set baudrate")
        portHandler.closePort()
        return
    print("   ✅ Baudrate set")
    
    # Ping motor to verify connection
    print("\n3. Checking motor connection...")
    model_number, comm_result, error = packetHandler.ping(portHandler, MOTOR_ID)
    if comm_result != COMM_SUCCESS:
        print(f"   ❌ Failed to connect to motor ID {MOTOR_ID}")
        print(f"   Error: {packetHandler.getTxRxResult(comm_result)}")
        portHandler.closePort()
        return
    print(f"   ✅ Motor connected (Model: {model_number})")
    
    try:
        # Disable torque to change operating mode
        print("\n4. Setting velocity control mode...")
        comm_result, error = packetHandler.write1ByteTxRx(
            portHandler, MOTOR_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
        )
        if comm_result != COMM_SUCCESS:
            print(f"   ⚠️  Could not disable torque: {packetHandler.getTxRxResult(comm_result)}")
        
        # Set velocity control mode
        comm_result, error = packetHandler.write1ByteTxRx(
            portHandler, MOTOR_ID, ADDR_OPERATING_MODE, VELOCITY_CONTROL_MODE
        )
        if comm_result != COMM_SUCCESS:
            print(f"   ⚠️  Could not set mode: {packetHandler.getTxRxResult(comm_result)}")
        else:
            print("   ✅ Velocity control mode set")
        
        # Enable torque
        print("\n5. Enabling torque...")
        comm_result, error = packetHandler.write1ByteTxRx(
            portHandler, MOTOR_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
        )
        if comm_result != COMM_SUCCESS:
            print(f"   ❌ Failed to enable torque: {packetHandler.getTxRxResult(comm_result)}")
            portHandler.closePort()
            return
        print("   ✅ Torque enabled")
        
        # Start spinning
        print(f"\n6. Starting motor spin at velocity {SPIN_VELOCITY}...")
        comm_result, error = packetHandler.write4ByteTxRx(
            portHandler, MOTOR_ID, ADDR_GOAL_VELOCITY, SPIN_VELOCITY
        )
        if comm_result != COMM_SUCCESS:
            print(f"   ❌ Failed to set velocity: {packetHandler.getTxRxResult(comm_result)}")
        else:
            print("   ✅ Motor spinning!")
            print("\n   🔄 Motor is running...", end="", flush=True)
            
            # Run for specified duration with progress indicator
            start_time = time.time()
            while (time.time() - start_time) < SPIN_DURATION:
                elapsed = time.time() - start_time
                print(f"\r   🔄 Motor is running... {elapsed:.1f}s", end="", flush=True)
                time.sleep(0.1)
            
            print(f"\r   🔄 Motor is running... {SPIN_DURATION:.1f}s - Complete!")
        
        # Stop motor
        print("\n7. Stopping motor...")
        comm_result, error = packetHandler.write4ByteTxRx(
            portHandler, MOTOR_ID, ADDR_GOAL_VELOCITY, 0
        )
        if comm_result != COMM_SUCCESS:
            print(f"   ⚠️  Could not stop motor: {packetHandler.getTxRxResult(comm_result)}")
        else:
            print("   ✅ Motor stopped")
        
        # Wait a moment for motor to fully stop
        time.sleep(0.5)
        
        # Disable torque
        print("\n8. Disabling torque...")
        comm_result, error = packetHandler.write1ByteTxRx(
            portHandler, MOTOR_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
        )
        if comm_result != COMM_SUCCESS:
            print(f"   ⚠️  Could not disable torque: {packetHandler.getTxRxResult(comm_result)}")
        else:
            print("   ✅ Torque disabled")
        
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted! Stopping motor...")
        # Emergency stop
        packetHandler.write4ByteTxRx(portHandler, MOTOR_ID, ADDR_GOAL_VELOCITY, 0)
        packetHandler.write1ByteTxRx(portHandler, MOTOR_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        print("   Motor stopped")
    
    finally:
        # Always close port
        portHandler.closePort()
        print("\n✅ Port closed")
        print("\nDone!")

if __name__ == "__main__":
    # Quick check for common issues
    import os
    if not os.path.exists(PORT_NAME):
        print(f"❌ Error: Port {PORT_NAME} not found!")
        print("Please check your connection.")
        exit(1)
    
    # Run the main function
    main()