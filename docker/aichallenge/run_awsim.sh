#!/bin/bash

sudo ip link set multicast on lo

source ./aichallenge_ws/install/setup.bash
./AWSIM/AWSIM.x86_64
