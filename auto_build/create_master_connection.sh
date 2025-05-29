#!/bin/bash

# ------------------------------------
#              NOTICE
# This script should NOT be used on
# its own. Please use the VSCode task 
# to run it
# ------------------------------------

# Initialize global variables
. ./auto_build/constants.sh

if [ ! -e $SSH_CONTROL_PATH ]; then
    echo "Creating global SSH connection..."

    # Don't exit on command fail
    set +e

    ssh \
        -fN ${COMMENT# This tells SSH to move to background after connection is established} \
        -o ControlMaster=yes ${COMMENT# This is the master connection, to be reused later} \
        -o ControlPersist=10m ${COMMENT# Stay open for 10 minutes} \
        -o ControlPath=$SSH_CONTROL_PATH ${COMMENT# Store the connection at this path} \
        -o ConnectTimeout=5 \
        $REMOTE_USER@$REMOTE_IP

    # Check if the command failed
    if [ $? -ne 0 ]; then
        echo -e "${RED}Failed to connect to SSH. Are you connected to the correct Wifi and did you input the right password?${NC}"
        exit 1
    fi

    # Reset exit on command failed
    set -e
fi