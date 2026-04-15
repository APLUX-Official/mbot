#!/bin/bash

# Two Differential Drive Robot - udev Rules Uninstallation Script
# This script removes udev rules for differential drive robot

RULES_FILE="99-differential-drive.rules"
RULES_PATH="/etc/udev/rules.d/${RULES_FILE}"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== Two Differential Drive Robot - udev Rules Uninstallation ===${NC}"

# Check if running as root
if [[ $EUID -eq 0 ]]; then
    echo -e "${RED}Error: This script should not be run as root directly.${NC}"
    echo "Please run: ./uninstall_udev_rules.sh"
    exit 1
fi

# Check if rules file exists
if [ ! -f "${RULES_PATH}" ]; then
    echo -e "${YELLOW}Warning: Rules file ${RULES_PATH} does not exist${NC}"
    echo "Nothing to uninstall."
    exit 0
fi

echo "Removing udev rules for differential drive robot..."

# Remove rules file
echo "Removing rules file from ${RULES_PATH}..."
sudo rm "${RULES_PATH}"

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Rules file removed successfully${NC}"
else
    echo -e "${RED}✗ Failed to remove rules file${NC}"
    exit 1
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

echo -e "${GREEN}=== Uninstallation completed successfully! ===${NC}"
echo
echo "The device symlinks /dev/drive_* should be removed after reconnecting devices."
echo "You can verify by running: ls -la /dev/drive_*"
