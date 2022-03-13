<h1>2021/2022 Oregon State University Underwater ROV Repository</h1>

## Acknowledgements

The overall structure of the repo is inspired by OSURC's Rover Team's repository.

## Setup
Assuming you already have ROS Melodic installed and created a catkin workspace (named "catkin_ws"), create a folder named "Dev" in the root of your home directory. Clone this repo into that directory, then run "setup.sh". This links all of packages from this repo into catkin_ws.

For launching the ground station, run "roslaunch rov_main ground_station.launch". For launching the ROV, run "roslaunch rov_main rov.launch".

## To Do
Create separate setup scripts for ground station and ROV.
