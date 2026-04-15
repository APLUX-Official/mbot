#!/bin/bash

# Two Differential Drive Robot - udev Rules Installation Script
# This script installs udev rules for consistent serial device naming

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RULES_FILE="99-differential-drive.rules"
RULES_PATH="/etc/udev/rules.d/${RULES_FILE}"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== Two Differential Drive Robot - udev Rules Installation ===${NC}"

# Check if running as root
if [[ $EUID -eq 0 ]]; then
    echo -e "${RED}Error: This script should not be run as root directly.${NC}"
    echo "Please run: ./install_udev_rules.sh"
    exit 1
fi

# Check if rules file exists
if [ ! -f "${SCRIPT_DIR}/${RULES_FILE}" ]; then
    echo -e "${RED}Error: Rules file ${RULES_FILE} not found in ${SCRIPT_DIR}${NC}"
    exit 1
fi

echo "Installing udev rules for differential drive robot..."

# Copy rules file
echo "Copying rules file to ${RULES_PATH}..."
sudo cp "${SCRIPT_DIR}/${RULES_FILE}" "${RULES_PATH}"

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Rules file copied successfully${NC}"
else
    echo -e "${RED}✗ Failed to copy rules file${NC}"
    exit 1
fi

# Set proper permissions
sudo chmod 644 "${RULES_PATH}"
echo -e "${GREEN}✓ Permissions set${NC}"

# Add user to dialout group if not already a member
if ! groups $USER | grep -q "\bdialout\b"; then
    echo "Adding user $USER to dialout group..."
    sudo usermod -a -G dialout $USER
    echo -e "${YELLOW}⚠ You need to log out and log back in for group changes to take effect${NC}"
fi

# Reload udev rules
echo "Reloading udev rules..."
sudo udevadm control --reload-rules
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ udev rules reloaded${NC}"
else
    echo -e "${RED}✗ Failed to reload udev rules${NC}"
    exit 1
fi

# Trigger udev
echo "Triggering udev..."
sudo udevadm trigger
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ udev triggered${NC}"
else
    echo -e "${RED}✗ Failed to trigger udev${NC}"
    exit 1
fi

echo -e "${GREEN}=== Installation completed successfully! ===${NC}"
echo
echo "Expected device mappings:"
echo "  - Serial 'left'  → /dev/drive_left"
echo "  - Serial 'right' → /dev/drive_right" 
echo "  - Serial 'lidar' → /dev/drive_lidar"
echo
echo "You can verify the installation by:"
echo "1. Reconnecting your USB devices"
echo "2. Running: ls -la /dev/drive_*"
echo "3. Or checking: udevadm info --name=/dev/drive_left"
echo
if groups $USER | grep -q "\bdialout\b"; then
    echo -e "${GREEN}✓ User is already in dialout group${NC}"
else
    echo -e "${YELLOW}⚠ Please log out and log back in to apply group changes${NC}"
fi
