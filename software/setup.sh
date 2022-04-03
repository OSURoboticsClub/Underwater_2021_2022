#!/usr/bin/env bash
catkin_ws="$HOME/catkin_ws"
catkin_src="$catkin_ws/src"

dev_path="$HOME/Dev/Underwater_2021_2022"
dev_ros_pkgs="$dev_path/software/ros_pkgs"

symlinked_folders=$(find $catkin_src -maxdepth 1 -type l)
if [ -n $symlinked_folders ]; then
    echo "Nothing to remove"
else 
    echo "Removing existing symlinks"
    rm $symlinked_folders
fi

# Make the new symbolic link connections
echo "Making new symlinks."
ros_pkgs=$(find $dev_ros_pkgs/* -maxdepth 0 -type d)
for folder in $ros_pkgs; do
    ln -s "${folder}" "$catkin_src/"
    echo "Adding symlink for $folder."
done

cd "$catkin_ws"
catkin_make

source devel/setup.bash

exit 0
