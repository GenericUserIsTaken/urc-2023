#!/bin/bash

# ------------------------------------
#              NOTICE
# This script should NOT be used on
# its own. Please use the VSCode task 
# to run it
# ------------------------------------

set -e

. ./auto_build/constants.sh
./auto_build/create_master_connection.sh

echo "Sending files to orin"

# Create dest folder
ssh -o ControlPath=$SSH_CONTROL_PATH $REMOTE_USER@$REMOTE_IP << EOF
    sudo rm -rf $DEST_FOLDER
    mkdir -p $DEST_FOLDER
EOF

# Send all the files in the folder to the remote machine's 'auto_build' folder
# Use tar to not have to transfer files individually, but one archive file
tar cz --exclude=*/.mypy_cache --exclude=./.git \
    --exclude=./build --exclude=./install --exclude=./log . | 
        ssh -o ControlPath=$SSH_CONTROL_PATH $REMOTE_USER@$REMOTE_IP \
            tar xvzC ./$DEST_FOLDER

# After tar, run the rest of the script remotely
# -t here tells ssh to open a proper terminal, which is needed for docker and sudo
ssh -t -o ControlPath=$SSH_CONTROL_PATH $REMOTE_USER@$REMOTE_IP \
    "./$DEST_FOLDER/auto_build/run_and_launch.sh"
