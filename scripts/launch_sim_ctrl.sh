#!/bin/bash

sudo ip link set lo multicast on
sudo ip route add 224.0.0.0/4 dev lo

sudo LD_LIBRARY_PATH=. ldconfig

sudo LD_LIBRARY_PATH=. /home/ouge/Software/dg_fsm_whl/build/test_repo/sim_ctrl
#sudo LD_LIBRARY_PATH=. ./../build/test_repo/sim_test --sim_model_name=../robot/robot_model/belt/scene.xml --sub_imu=false --sub_usb2can=false --use_rc=true --run_type=3
