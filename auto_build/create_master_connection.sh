# ------------------------------------
#              NOTICE
# This script should NOT be used on
# its own. Please use the VSCode task 
# to run it
# ------------------------------------

# Initialize global variables
. constants.sh

if [ ! -f $SSH_CONTROL_PATH ]; then
    ssh \
        # This is the master connection, to be reused later
        -o ControlMaster=yes \
        
        # Stay open for 10 minutes
        -o ControlPersist=10m \ 

        # Store the connection at this path
        -o ControlPath=$SSH_CONTROL_PATH \

        # Tell SSH the machine we want to connect to
        $REMOTE_USER@$REMOTE_IP
fi