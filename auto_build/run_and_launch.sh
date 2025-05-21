# ------------------------------------
#              NOTICE
# This script should NOT be used on
# its own. Please use the VSCode task 
# to run it
# ------------------------------------

set -e

# Read the data being streamed from the client
tar xvzC ./auto_build

# Set out working directory to the 'auto_build' folder
cd auto_build

# Launch our docker container, making sure to create a new container and rebuild
sudo ./container_launch.sh -b -c


# Function that's run when a user kills the process (e.g. ctrl + c, closing VSC)
on_sigint() {
    echo "Caught SIGINT, shutting down container..." 
    sudo docker container kill trickfirerobot
}

# Register our function to run on sigint
trap 'on_sigint' SIGINT


# Build and launch our new code
./build.sh && ./launch.sh