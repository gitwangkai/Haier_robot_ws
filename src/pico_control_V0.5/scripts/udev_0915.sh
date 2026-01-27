#!/bin/bash

# ==============================================================================
#                 Udev Rule Installation Script for Robot Devices
#
# This script creates a udev rule to assign persistent names to the robot's
# serial devices (Lidar and arm_right) based on their physical USB port path.
# This prevents the device names from changing (e.g., from ttyUSB0 to ttyUSB1)
# after a reboot or reconnect.
#
# It will create the following symbolic links:
#   - /dev/lidar      -> points to the device at physical path 3-2.1.1
#   - /dev/arm_right  -> points to the device at physical path 3-2.4
#
# ==============================================================================

# --- Configuration ---
# The full path to the udev rules file to be created.
RULE_FILE="/etc/udev/rules.d/99-my-robot.rules"

# The content of the udev rules. Using a "here document" for clarity.
# KERNELS=="3-2.4"     is ttyUSB0 -> to be named arm_right
# KERNELS=="3-2.1.1"   is ttyUSB1 -> to be named lidar
read -r -d '' RULE_CONTENT << 'EOF'
# Rules for Robot Serial Devices
# Assign persistent names based on physical USB port path

# Right Arm Controller (was ttyUSB0)
#SUBSYSTEMS=="usb", KERNELS=="3-2.4", MODE="0666", SYMLINK+="arm_right"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="arm_right", MODE="0666", SYMLINK+="arm_right"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="a01ed1fe1901f011b4a2c6295c2a50c9", MODE="0666", SYMLINK+="lidar"

# Lidar (was ttyUSB1)
#SUBSYSTEMS=="usb", KERNELS=="3-2.1.1", MODE="0666", SYMLINK+="lidar"

# Camera (was /dev/video0)
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="0bda", ATTRS{idProduct}=="5844", ATTRS{serial}=="200901010001", SYMLINK+="arm_right_camera"

EOF

# --- Script Logic ---

# Check if the script is being run with root privileges (sudo)
if [ "$EUID" -ne 0 ]; then
  echo "❌ Error: This script must be run with root privileges."
  echo "Please run it using 'sudo ./install_udev_rules.sh'"
  exit 1
fi

echo "🚀 Starting udev rule installation..."

# Write the rules to the file.
# We use 'tee' to write the file as root. The output of tee is redirected to /dev/null
# to keep the script output clean.
echo "   1. Creating udev rule file at ${RULE_FILE}..."
echo "${RULE_CONTENT}" | tee "${RULE_FILE}" > /dev/null

if [ $? -eq 0 ]; then
    echo "   ✅ File created successfully."
else
    echo "   ❌ Error: Failed to create rule file. Aborting."
    exit 1
fi


# Reload udev rules to apply the changes.
echo "   2. Reloading udev rules..."
udevadm control --reload-rules
echo "   ✅ Rules reloaded."

# Trigger the new rules to be applied to currently connected devices.
echo "   3. Triggering new rules on connected devices..."
udevadm trigger
echo "   ✅ Trigger command sent."

echo ""
echo "🎉 Installation complete!"
echo ""
echo "--------------------------------------------------------"
echo "VERIFICATION:"
echo "To verify that the symbolic links have been created, run:"
echo "ls -l /dev/lidar /dev/arm_right"
echo ""
echo "You should see output similar to this:"
echo "lrwxrwxrwx 1 root root 7 Aug 10 17:00 /dev/arm_right -> ttyUSB0"
echo "lrwxrwxrwx 1 root root 7 Aug 10 17:00 /dev/lidar -> ttyUSB1"
echo "(Note: The target ttyUSB* number might swap, but the symlink name will be correct)"
echo "--------------------------------------------------------"
echo ""

exit 0
