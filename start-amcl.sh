#!/bin/bash

#########################
### DEFAULT VARIABLES ###
user=$(whoami)
maps_dir="/home/$user/catkin_ws/src/mie443_contest2/mie443_contest2/maps/"
map_file="map_1.yaml"

#################
### FUNCTIONS ###
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

##################
### SET PARAMS ###
while [ $# -gt 0 ]; do
    case "$1" in
        --map_file | -m)
            if check_length "$2"; then
                map_file="$2"
                shift 2
            else
                shift 1
            fi
            ;;
        map_file=*)
            map_file="${1#*=}"
            shift 1
            ;;
        -h)
            echo "Usage: $0 [OPTIONS]"
            echo "1. To set the map file/directory (default: $maps_dir$map_file):"
            echo "|-> [--map_file <value>]"
            echo "|-> [-m <value>]"
            echo "|-> [map_file=<value>]"
            echo "2. For help:"
            echo "|-> [--help]"
            echo "|-> [-h]"
            exit 0
            ;;
        *)
            ;;
    esac
done


############
### MAIN ###
command="roslaunch turtlebot_gazebo amcl_demo.launch map_file:=\"$maps_dir$map_file\""
echo ""
echo "$command"
echo ""
eval "$command"
