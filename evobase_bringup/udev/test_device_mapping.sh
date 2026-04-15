#!/bin/bash

# Two Differential Drive Robot - Device Mapping Test Script
# This script tests if the udev rules are working correctly

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=== Two Differential Drive Robot - Device Mapping Test ===${NC}"
echo

# Function to check if a device exists and show its properties
check_device() {
    local device_path=$1
    local expected_serial=$2
    
    echo -e "${YELLOW}Checking $device_path (expected serial: $expected_serial)...${NC}"
    
    if [ -e "$device_path" ]; then
        echo -e "${GREEN}✓ Device exists: $device_path${NC}"
        
        # Get real device path
        real_path=$(readlink -f "$device_path")
        echo "  Real path: $real_path"
        
        # Get device info using udevadm
        if command -v udevadm &> /dev/null; then
            echo "  Device information:"
            udevadm info --name="$device_path" | grep -E "(ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT|DEVNAME)" | sed 's/^/    /'
        fi
        
        # Check permissions
        ls -la "$device_path" | sed 's/^/  Permissions: /'
        
        echo
        return 0
    else
        echo -e "${RED}✗ Device not found: $device_path${NC}"
        echo
        return 1
    fi
}

# Check expected devices
echo "Checking expected device mappings..."
echo

success_count=0
total_count=3

if check_device "/dev/drive_left" "left"; then
    ((success_count++))
fi

if check_device "/dev/drive_right" "right"; then
    ((success_count++))
fi

if check_device "/dev/drive_lidar" "lidar"; then
    ((success_count++))
fi

# Summary
echo -e "${BLUE}=== Summary ===${NC}"
echo "Successfully mapped devices: $success_count/$total_count"

if [ $success_count -eq $total_count ]; then
    echo -e "${GREEN}✓ All devices are properly mapped!${NC}"
    exit 0
elif [ $success_count -gt 0 ]; then
    echo -e "${YELLOW}⚠ Some devices are missing. Check if all USB devices are connected.${NC}"
    exit 1
else
    echo -e "${RED}✗ No devices found. Please check:${NC}"
    echo "  1. Are the USB devices connected?"
    echo "  2. Are the udev rules installed? (run ./install_udev_rules.sh)"
    echo "  3. Try reconnecting the USB devices"
    exit 2
fi
