# This Python script scans for Dynamixel motors connected to an OpenCR board acting as a USB-to-Dynamixel bridge.
# Prerequisites:
# 1. Install the Dynamixel SDK for Python: pip install dynamixel_sdk or clone from https://github.com/ROBOTIS-GIT/DynamixelSDK
# 2. Set up OpenCR as a bridge:
#    - Install Arduino IDE and add OpenCR board manager.
#    - Upload the example: File > Examples > OpenCR > 10. Etc. > usb_to_dxl to the OpenCR board.
# 3. Connect OpenCR to Linux PC via USB (appears as /dev/ttyACM0 or similar).
# 4. Connect Dynamixel motors to the OpenCR's Dynamixel port and provide power to motors.
# 5. Adjust DEVICENAME and BAUDRATE as needed (default baudrate for many Dynamixels is 57600, but check your motor's setting).
# 6. Run this script with Python 3.

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

# Protocol version
PROTOCOL_VERSION = 2.0

# Default baudrate for OpenCR and Dynamixel X series
BAUDRATE = 57600

# Control table addresses (common for X series)
ADDR_MODEL_NUMBER = 0
ADDR_ID = 7
ADDR_BAUDRATE = 8
ADDR_FIRMWARE_VERSION = 6

def find_opencr_port():
    """
    Find OpenCR port on Linux
    OpenCR typically shows up as /dev/ttyACM* on Linux
    """
    print("Searching for OpenCR board...")
    print("-" * 50)
    
    # List all available serial ports
    ports = serial.tools.list_ports.comports()
    opencr_ports = []
    
    for port in ports:
        print(f"Found port: {port.device}")
        print(f"  Description: {port.description}")
        print(f"  Manufacturer: {port.manufacturer}")
        
        # OpenCR typically shows up as ttyACM on Linux
        if "ttyACM" in port.device or "OpenCR" in str(port.description):
            opencr_ports.append(port.device)
            print(f"  *** Likely OpenCR port ***")
        # Some USB-to-serial adapters might show as ttyUSB
        elif "ttyUSB" in port.device:
            opencr_ports.append(port.device)
            print(f"  *** Possible serial port ***")
        print()
    
    return opencr_ports

def test_port_connection(port_name, baudrate):
    """
    Test if we can open and communicate through the port
    """
    portHandler = PortHandler(port_name)
    packetHandler = PacketHandler(PROTOCOL_VERSION)
    
    # Try to open port
    if not portHandler.openPort():
        return False, "Cannot open port"
    
    # Try to set baudrate
    if not portHandler.setBaudRate(baudrate):
        portHandler.closePort()
        return False, "Cannot set baudrate"
    
    # Port opened successfully
    portHandler.closePort()
    return True, "Port accessible"

def scan_for_motors(port_name, baudrate, scan_range=253):
    """
    Scan for all connected motors on the given port
    """
    portHandler = PortHandler(port_name)
    packetHandler = PacketHandler(PROTOCOL_VERSION)
    
    # Open port
    if not portHandler.openPort():
        print(f"Failed to open port {port_name}")
        return []
    
    # Set port baudrate
    if not portHandler.setBaudRate(baudrate):
        print(f"Failed to set baudrate {baudrate}")
        portHandler.closePort()
        return []
    
    print(f"\nScanning for motors on {port_name} at {baudrate} bps...")
    print("This may take a moment...")
    print("-" * 50)
    
    found_motors = []
    
    # Scan motor IDs from 0 to scan_range-1
    for motor_id in range(scan_range):
        # Show progress
        if motor_id % 10 == 0:
            print(f"\rScanning ID: {motor_id}/{scan_range-1}", end="", flush=True)
        
        # Try to ping the motor
        dxl_model_number, dxl_comm_result, dxl_error = packetHandler.ping(portHandler, motor_id)
        
        if dxl_comm_result == COMM_SUCCESS and dxl_error == 0:
            # Motor found! Get more details
            motor_info = {
                'id': motor_id,
                'model': dxl_model_number
            }
            
            # Try to read firmware version
            fw_version, comm_result, error = packetHandler.read1ByteTxRx(
                portHandler, motor_id, ADDR_FIRMWARE_VERSION
            )
            if comm_result == COMM_SUCCESS:
                motor_info['firmware'] = fw_version
            
            found_motors.append(motor_info)
            
            # Clear the progress line and show found motor
            print(f"\r" + " " * 50 + "\r", end="")
            print(f"✓ Found Motor - ID: {motor_id}, Model: {dxl_model_number}")
    
    # Clear the final progress line
    print(f"\r" + " " * 50 + "\r", end="")
    
    # Close port
    portHandler.closePort()
    
    return found_motors

def get_motor_model_name(model_number):
    """
    Convert model number to motor name
    """
    models = {
        1020: "XM430-W350",
        1030: "XM540-W150", 
        1040: "XM540-W270",
        1050: "XH430-V210",
        1060: "XL430-W250",
        1070: "XH430-V350",
        1080: "XH430-W210",
        1090: "XH430-W350",
        1110: "XM430-W210",
        1120: "XH540-W150",
        1130: "XH540-W270",
        1140: "XH540-V150",
        1150: "XH540-V270",
        350: "XL320",
        360: "XL330-M077",
        370: "XL330-M288"
    }
    return models.get(model_number, f"Unknown (Model {model_number})")

def main():
    """
    Main function
    """
    print("\n=== OpenCR Dynamixel Motor Scanner for Linux ===")
    print("=" * 50)
    
    # Check if running with proper permissions
    if os.geteuid() != 0 and not os.access('/dev/ttyACM0', os.R_OK | os.W_OK):
        print("\n⚠️  WARNING: You might need sudo permissions or add your user to dialout group")
        print("   If the scan fails, try:")
        print("   1. sudo python3 this_script.py")
        print("   2. sudo usermod -a -G dialout $USER (then logout and login)")
        print()
    
    # Find possible OpenCR ports
    opencr_ports = find_opencr_port()
    
    if not opencr_ports:
        print("\n❌ No OpenCR or serial ports found!")
        print("Please check:")
        print("  1. OpenCR is connected via USB")
        print("  2. OpenCR is powered on")
        print("  3. USB cable is working")
        return
    
    print(f"\n📍 Found {len(opencr_ports)} possible port(s)")
    
    # Test each port and scan for motors
    all_motors = []
    working_port = None
    
    for port in opencr_ports:
        print(f"\n🔍 Testing port: {port}")
        
        # Test if port is accessible
        success, message = test_port_connection(port, BAUDRATE)
        print(f"   Status: {message}")
        
        if success:
            # Scan for motors on this port
            motors = scan_for_motors(port, BAUDRATE)
            
            if motors:
                working_port = port
                all_motors.extend(motors)
                print(f"\n✅ Found {len(motors)} motor(s) on {port}")
                break
            else:
                print(f"   No motors found on this port")
    
    # Display results
    print("\n" + "=" * 50)
    print("SCAN RESULTS")
    print("=" * 50)
    
    if all_motors:
        print(f"\n✅ Total motors found: {len(all_motors)}")
        print("\nMotor Details:")
        print("-" * 50)
        for motor in all_motors:
            model_name = get_motor_model_name(motor['model'])
            print(f"  ID: {motor['id']}")
            print(f"  Model: {model_name}")
            if 'firmware' in motor:
                print(f"  Firmware: v{motor.get('firmware', 'Unknown')}")
            print("-" * 30)
        
        print(f"\n📌 Connected via: {working_port}")
        print(f"📌 Baudrate: {BAUDRATE}")
        
        # Save results to file
        with open("motor_scan_results.txt", "w") as f:
            f.write(f"OpenCR Motor Scan Results\n")
            f.write(f"Port: {working_port}\n")
            f.write(f"Baudrate: {BAUDRATE}\n")
            f.write(f"Motors found: {len(all_motors)}\n\n")
            for motor in all_motors:
                f.write(f"ID: {motor['id']}, Model: {get_motor_model_name(motor['model'])}\n")
        print("\n💾 Results saved to: motor_scan_results.txt")
        
    else:
        print("\n❌ No motors detected!")
        print("\nTroubleshooting:")
        print("  1. Check motor power supply (12V for X series)")
        print("  2. Verify motor cables are properly connected")
        print("  3. Check if OpenCR switches are in correct position")
        print("  4. Try different baudrate (1000000 for high-speed mode)")
        print("  5. Ensure motors are compatible with Protocol 2.0")

if __name__ == "__main__":
    # Install required packages if not available
    try:
        import serial.tools.list_ports
    except ImportError:
        print("Installing required package: pyserial")
        os.system("pip3 install pyserial")
        import serial.tools.list_ports
    
    # try:
    #     from dynamixel_sdk import *
    # except ImportError:
    #     print("Installing required package: dynamixel-sdk")
    #     os.system("pip3 install dynamixel-sdk")
    #     from dynamixel_sdk import *
    
    main()