# ------------------------------------
#              NOTICE
# This script should NOT be used on
# its own. Please use the VSCode task 
# to run it
# ------------------------------------

set -e

. ./auto_build/constants.sh
./auto_build/create_master_connection.sh

# Send all the files in the folder to the remote machine's 'auto_build' folder
# Use tar to not have to transfer files individually, but one archive file
# After tar, run the rest of the script remotely
tar cz --exclude=./build --exclude=./log --exclude=./install ./* | 
    ssh -o ControlPath=$SSH_CONTROL_PATH $REMOTE_USER@$REMOTE_IP \
        ./auto_build/auto_build/run_and_launch.sh
    