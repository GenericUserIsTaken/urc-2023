#!/bin/bash

# ------------------------------------
#              NOTICE
# This script should NOT be used on
# its own. Please use the VSCode task 
# to run it
# ------------------------------------

set -e

# Set out working directory to the 'auto_build' folder
cd auto_build

# Function that's run when a user kills the process (e.g. ctrl + c, closing VSC)
on_exit() {
    echo "Ending task... shutting down container..." 
    sudo docker container kill trickfirerobot
}

# Register our function to run on sigint
trap 'on_exit' SIGINT

# Launch our docker container, making sure to create a new container and rebuild
# the echo pipe is just to exit the bash terminal when build is completed
# prob not the best way to do it but eh
echo "Launching container..."
echo | sudo ./container_launch.sh -b -c

# Build and launch our new code
echo "Building and launching code..."
sudo docker exec -it trickfirerobot /bin/bash -c "./build.sh && ./launch.sh"

on_exit