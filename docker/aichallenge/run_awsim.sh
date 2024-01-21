#!/bin/bash

sudo ip link set multicast on lo
SCRIPT_DIR=$(cd $(dirname $0); pwd)
source $SCRIPT_DIR/aichallenge_ws/install/setup.bash
$SCRIPT_DIR/AWSIM/AWSIM.x86_64
