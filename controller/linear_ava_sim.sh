#!/bin/bash
rosclean purge
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 2; rosrun sim_radio sim_radio_node; exec bash"' \
--tab -e 'bash -c "sleep 2; rosrun controller linear_ava_controller_node; exec bash"' \
--tab -e 'bash -c "sleep 1; rostopic echo /main_position; exec bash"' \
--tab -e 'bash -c "sleep 1; rostopic echo /main_euler_angles; exec bash"' \
--tab -e 'bash -c "sleep 1; rostopic echo /main_velocity; exec bash"' \
--tab -e 'bash -c "sleep 1; rostopic echo /main_body_rates; exec bash"' \

