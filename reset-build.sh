base_dir=$(dirname "$(readlink -f "$0")")
cd "$base_dir"           # Go to reset-build.sh file's directory


sudo rm -rf build     # Remove all build files created by catkin_make
sudo rm -rf devel     # Remove all devel files created by catkin_make


catkin_make           # Re-build projecct


source devel/setup.sh # re-source
