## This is the setup file for linux users...

#################
### VARIABLES ###
# Get base-directory
base_dir=$(dirname "$(readlink -f "$0")")

contest1_dir="src/mie443_contest1/mie443_contest1"
contest2_dir="src/mie443_contest2/mie443_contest2"

# Print repository's base_dir to ensure it works correctly
echo ""
echo "base_dir := ${base_dir}"



#################
### SHORTCUTS ###
# Setup shortcut directories (if they don't already exist)
cd "${base_dir}"

if ! [ -L "priv-c1" ]; then
    echo "Creating shortcut: priv-c1..."
    ln -s "${contest1_dir}" priv-c1
fi

if ! [ -L "priv-c2" ]; then
    echo "Creating shortcut: priv-c2..."
    ln -s "${contest2_dir}" priv-c2
fi

#################
### GIT STUFF ###
# Setup ignore for "GLOBAL" test files (ie. they exist with a default, but local changes should remain uncommited)
cd "${base_dir}"

git update-index --assume-unchanged "${contest2_dir}/src/priv-test.cpp"



echo "DONE..."
echo ""

