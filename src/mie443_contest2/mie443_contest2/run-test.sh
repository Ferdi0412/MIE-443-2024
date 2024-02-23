#!/bin/bash

################
### FUNCTION ###
check_length() {
    if [ ${#1} -gt 0 ]; then
        return 0
    else
        return 1
    fi
}

command_in_new_terminal() {
    tab_title="$1"
    target_command="$2"
    if ! check_length "$1" || ! check_length "$2"; then
        echo "Could not execute '$2' in tab '$1'"
        exit 1
    fi
    gnome-terminal --tab --title="$1" --command="bash -c 'echo $1; echo; $2; exec bash'" &> /dev/null
}

############
### MAIN ###
command_in_new_terminal "Gazebo" "roslaunch mie443_contest2 turtlebot_world.launch world:=1"

echo "Press [ENTER] to start trial..."
read usr_input
# sleep 2 # Sleep 2 seconds for gazebo to (hopefully) start up properly

command_in_new_terminal "TEST" "rosrun mie443_contest2 priv_test"
