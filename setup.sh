## This is the setup file for linux users...

#################
### VARIABLES ###
# Get base-directory
base_dir=$(dirname "$(readlink -f "$0")")

contest1_dir="src/mie443_contest1/mie443_contest1"
contest2_dir="src/mie443_contest2/mie443_contest2"
contest3_dir="src/mie443_contest3/mie443_contest3"
follower_dir="src/turtlebot_follower/turtlebot_follower"

# Print repository's base_dir to ensure it works correctly
echo ""
echo "base_dir := ${base_dir}"
echo ""



#################
### FUNCTIONS ###
# Returns TRUE if a command-line executable is available
is_command_available() {
    command_to_check="$1"
    if command -v "$command_to_check" &> /dev/null
    then
        return 0 # 0 means no error/TRUE
    else
        return 1 # 1 means error/FALSE
    fi
}

exit_if_command_unavailable() {
    command_to_check="$1"
    if ! is_command_available "$command_to_check"; then
        echo ""
        echo "!!! WARNING !!!"
        echo "${command_to_check} is not available..."
        echo "exiting now..."
        exit 1
    fi
}



####################
### CATKIN SETUP ###
cd "${base_dir}"

if ! [ -e "src/CMakeLists.txt" ]; then
    exit_if_command_unavailable "catkin_init_workspace"

    echo "[Catkin Setup] Running catkin_init_workspace..."
    cd src
    catkin_init_workspace > /dev/null
    cd ..
    echo ""
else
    echo "[Catkin Setup] Workspace already initialized..."
    echo ""
fi

if ! [ -d "build" ] || ! [ -d "devel" ]; then
    exit_if_command_unavailable "catkin_make"

    rm -rf build &> /dev/null
    rm -rf devel &> /dev/null
    echo "[Catkin Setup] Do you wish to run catkin_make [y/n]?"
    read confirmation
    if [ "$confirmation" = "y" ]; then
        echo "[Catkin Setup] Running catkin_make..."
        echo ""
        catkin_make
        echo "[Catkin Setup] catkin_make completed..."
        echo ""
    else
        echo "[Catkin Setup] Not running catkin_make, run this manually later to build project..."
        echo ""
    fi
else
    echo "[Catkin Setup] Project already built..."
    echo ""
fi

## Re-source terminal
cd "${base_dir}"
source devel/setup.sh > /dev/null # re-source

#################
### SHORTCUTS ###
# Setup shortcut directories (if they don't already exist)
cd "${base_dir}"

# Check if symbolic linked directory "priv-c1" exists in this folder...
if ! [ -L "priv-c1" ]; then
    # If not, then create the new shortcut
    echo "[Shortcuts] Creating shortcut: priv-c1..."
    ln -s "${contest1_dir}" priv-c1 > /dev/null
    echo ""
fi

if ! [ -L "priv-c2" ]; then
    echo "[Shortcuts] Creating shortcut: priv-c2..."
    ln -s "${contest2_dir}" priv-c2 > /dev/null
    echo ""
fi

if ! [ -L "priv-c3" ]; then
    echo "[Shortcuts] Creating shortcut: priv-c3..."
    ln -s "${contest3_dir}" priv-c3 > /dev/null
    echo ""
fi

if ! [ -L "priv-follower" ]; then
    echo "[Shortcuts] Creating shortcut: priv-follower..."
    ln -s "${follower_dir}" priv-follower > /dev/null
    echo ""
fi


#################
### GIT STUFF ###
# Setup ignore for "GLOBAL" test files (ie. they exist with a default, but local changes should remain uncommited)
cd "${base_dir}"

git update-index --assume-unchanged "${contest2_dir}/src/priv-test.cpp"
git update-index --assume-unchanged "${contest3_dir}/src/priv-response-test.cpp"



###########
### END ###
echo ""
echo "[setup.sh] DONE..."
echo ""


