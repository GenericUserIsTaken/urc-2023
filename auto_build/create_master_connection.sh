# ------------------------------------
#              NOTICE
# This script should NOT be used on
# its own. Please use the VSCode task 
# to run it
# ------------------------------------

set -e

# Initialize global variables
. ./auto_build/constants.sh

if [ ! -e $SSH_CONTROL_PATH ]; then
    ssh \
        -fN ${COMMENT# This tells SSH to move to background after connection is established} \
        -o ControlMaster=yes ${COMMENT# This is the master connection, to be reused later} \
        -o ControlPersist=10m ${COMMENT# Stay open for 10 minutes} \
        -o ControlPath=$SSH_CONTROL_PATH ${COMMENT# Store the connection at this path} \
        $REMOTE_USER@$REMOTE_IP
fi