#!/bin/bash

# ------------------------------------
#              NOTICE
# This script should NOT be used on
# its own. Please use the VSCode task 
# to run it
# ------------------------------------

set -e

REMOTE_IP=192.168.0.145
REMOTE_USER=trickfire
SSH_CONTROL_PATH=~/.ssh/auto_build.sock
DEST_FOLDER=auto_build
COMMENT=

bold=$(tput bold)
BLUE='\033[0;34m'
RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m' # No Color