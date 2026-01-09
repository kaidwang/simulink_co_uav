#!/bin/bash
rosclean purge
gnome-terminal --window -e 'bash -c "roslaunch mavros apm2.launch fcu_url:="/dev/ttyACM0"; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch swarm_child child1.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch swarm_child child2.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch swarm_child child3.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch swarm_child child4.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun swarm_child child1_node; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun swarm_child child2_node; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun swarm_child child3_node; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun swarm_child child4_node; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun get_master_make_m get_master_make_m_node; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun controller four_controller_node; exec bash"' \
#--tab -e 'bash -c "sleep 5; rosrun mpc_controller mpc_controller_node; exec bash"' \
#--tab -e 'bash -c "sleep 5; rosrun drive_servo robot_arm_node; exec bash"' \
#--tab -e 'bash -c "sleep 5; rosrun plant plant_node; exec bash"' \
#--tab -e 'bash -c "sleep 5; rosrun mpc_controller cv_process_node"' \
#--tab -e 'bash -c "sleep 5; rosrun mpc_controller detect_target_node"' \

