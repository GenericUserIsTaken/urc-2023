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

# Connect to orin
ssh -o ControlPath=$SSH_CONTROL_PATH $REMOTE_USER@$REMOTE_IP