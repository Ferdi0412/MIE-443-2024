#!/bin/bash

#########################
### DEFAULT VARIABLES ###
world="N/A"
world_handle=""
contest="3"
contest_handle="turtlebot_gazebo"

#################
### FUNCTIONS ###
check_length() {
    if [ ${#1} -gt 0 ]; then
        return 0
    else
        return 1
    fi
}

set_world_handle() {
    local world_val="$1"
    case "$world_val" in
        "_")
            world_handle=""
            ;;
        *)
            world_handle="world:=$world_val"
            ;;
    esac
}

set_contest_handle() {
    local contest_val="$1"
    case "$contest_val" in
        "3")
            contest_handle="turtlebot_gazebo"
            ;;
        *)
            "mie443_contest$contest_val"
            ;;
    esac
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
                set_world_handle "$world"
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
            set_world_handle "$world"
            ;;
        contest=*)
            contest="${1#*=}"
            shift 1
            ;;
        -h)
            echo "Usage: $0 [OPTIONS]"
            echo "1. To set the GAZEBO world (default: $world):"
            echo "|-> [--world <value>]"
            echo "|-> [-w <value>]"
            echo "|-> [world=<value>]"
            echo "2. To select the contest (default: $contest):"
            echo "|-> [--contest <value>]"
            echo "|-> [-c <value>]"
            echo "|-> [contest=<value>]"
            echo "3. For help:"
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
command="roslaunch $contest_handle turtlebot_world.launch $world_handle"
echo ""
echo "$command"
echo ""
eval "$command"
