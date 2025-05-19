# ------------------------------------
#              NOTICE
# This script should NOT be used on
# its own. Please use the VSCode task 
# to run it
# ------------------------------------

. variables.sh
. create_master_connection.sh

# Send all the files in the folder to the remote machine's 'auto_build' folder
scp -r -o ControlPath=$SSH_CONTROL_PATH . $REMOTE_USER@$REMOTE_IP:~/auto_build/

# Create ssh connection to send commands
ssh -o ControlPath=$SSH_CONTROL_PATH $REMOTE_USER@$REMOTE_IP


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