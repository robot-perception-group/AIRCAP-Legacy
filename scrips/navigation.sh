#!/bin/bash

roslaunch nmpc_planner planner_realTarget.launch robotID:=$MACHINE NUM_ROBOTS:=$CCOUNT

echo "nmpc_planner failed with errorcode $?" >>${LOGDIR}/current/faillog
