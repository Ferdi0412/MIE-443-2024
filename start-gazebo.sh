#!/bin/bash

#########################
### DEFAULT VARIABLES ###
world=1
contest=2

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
        --world | -w)
            if check_length "$2"; then
                world="$2"
                shift 2
            else
                shift 1
            fi
            ;;
        --contest | -c)
            if check_length "$2"; then
                contest="$2"
                shift 2
            else
                shift 1
            fi
            ;;
        world=*)
            world="${1#*=}"
            shift 1
            ;;
        contest=*)
            contest="${1#*=}"
            shift 1
            ;;
        -h)
            echo "Usage: $0 [OPTIONS]"
            echo "To set the GAZEBO world:"
            echo "|-> [--world <value>]"
            echo "|-> [-w <value>]"
            echo "|-> [world=<value>]"
            echo "To select the contest:"
            echo "|-> [--contest <value>]"
            echo "|-> [-c <value>]"
            echo "|-> [contest=<value>]"
            echo "For help:"
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
echo "roslaunch 'mie443_contest$contest' turtlebot_world.launch 'world:=$world'"
roslaunch "mie443_contest$contest" turtlebot_world.launch "world:=$world"
