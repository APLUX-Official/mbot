#!/bin/bash
#
# ===================================================================
# ==         Minimalist ROS Version Selector Script              ==
# ==         (Intended to be sourced by .bashrc)                 ==
# ===================================================================

# Display the selection menu.
echo ""
echo "--- ROS Version Selector ---"
echo "  1) ROS 1 Noetic"
echo "  2) ROS 2 Foxy"
read -p "请选择版本 (直接回车默认选 2): " ros_choice
echo "" # Add a newline for cleaner output.

# Process the choice.
case "$ros_choice" in
    "1")
        echo "Sourcing ROS 1 Noetic..."
        source /opt/ros/noetic/setup.bash
        echo "✓ ROS 1 已激活。"
        ;;
    *) # Any other input, including empty (just pressing Enter), defaults to ROS 2.
        echo "Sourcing ROS 2 Foxy..."
        source /opt/ros/foxy/setup.bash
        echo "✓ ROS 2 已激活。"
        ;;
esac

# Always try to source colcon auto-completion for convenience with ROS 2.
# It's safe to run this even if ROS 1 is selected.
if [ -f /usr/share/colcon-argcomplete/hook/colcon-argcomplete.bash ]; then
    source /usr/share/colcon-argcomplete/hook/colcon-argcomplete.bash
fi
if [ -f /usr/share/colcon_cd/function/colcon_cd.sh ]; then
    source /usr/share/colcon_cd/function/colcon_cd.sh
    export _colcon_cd_root=~/
fi
