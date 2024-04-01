#!/bin/bash

#########################
### DEFAULT VARIABLES ###
webcam_id="0"

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
        --webcam | --webcam_id | -w)
            if check_length "$2"; then
                webcam_id="$2"
                shift 2
            else
                shift 1
            fi
            ;;
        webcam=*)
            webcam_id="${1#*=}"
            shift 1
            set_world_handle "$world"
            ;;
        -h)
            echo "Usage: $0 [OPTIONS]"
            echo "1. To set the GAZEBO world (default: $world):"
            echo "|-> [--world <value>]"
            echo "|-> [-w <value>]"
            echo "|-> [world=<value>]"
            echo "2. For help:"
            echo "|-> [--help]"
            echo "|-> [-h]"
            echo "3. To list webcams:"
            echo "|-> ls /dev | grep video"
            exit 0
            ;;
        *)
            ;;
    esac
done


############
### MAIN ###
command="rosrun mie443_contest3 publisher $webcam_id"
echo ""
echo "$command"
echo ""
eval "$command"