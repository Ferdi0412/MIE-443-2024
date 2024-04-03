########################
### INSTALLATION FIX ###
########################
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F42ED6FBAB17C654
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key EB3E94ADBE1229CF
sudo apt update
sudo apt-get update

##################
### GAZEBO FIX ###
##################
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update
sudo apt upgrade
