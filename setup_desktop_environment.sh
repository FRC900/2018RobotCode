#!/bin/bash
echo "WARNING this script will not work with custom configs for vim as well as some other programs, please read over this script before running if this is not a fresh install, press CTRL+C to quit"
read -n1 -r -p "Press any key to continue..." key

#apt string copied from setup_environment.sh
sudo apt update
sudo apt upgrade -y
sudo apt install -y libeigen3-dev build-essential gfortran git cmake libleveldb-dev libsnappy-dev libhdf5-dev libhdf5-serial-dev liblmdb-dev libgflags-dev libgoogle-glog-dev libatlas-base-dev python-dev python-pip libtinyxml2-dev v4l-conf v4l-utils libgtk2.0-dev pkg-config exfat-fuse exfat-utils libprotobuf-dev protobuf-compiler unzip python-numpy python-scipy python-opencv python-matplotlib wget unzip
sudo apt-get install --no-install-recommends -y libboost-all-dev
sudo apt-get install -y libflann-dev libpcl-dev

#add repos for neovim, vim-backports, and ros
sudo add-apt-repository -y ppa:jonathonf/vim
sudo add-apt-repository -y ppa:neovim-ppa/stable

echo "Do you wish to install ROS? Select No if already installed"
select yn in "Yes" "No"; do
    case $yn in
        Yes ) sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 && sudo apt update && sudo apt install -y --install-recommends ros-kinetic-desktop-full && source /opt/ros/kinetic/setup.bash && sudo rosdep init && rosdep update; break;;
        No ) break;;
    esac
done
sudo apt update

#removes regular vim and installs Vim with py2 support enabled, as well as other tools
sudo apt remove -y *vim*
sudo apt install -y vim-gtk-py2 neovim python3-dev python3-pip ros-kinetic-desktop-full python-rosinstall python-rosinstall-generator python-wstool aptitude docker clang colorgcc ranger
sudo apt install -y ros-kinetic-hardware-interface ros-kinetic-realtime-tools ros-kinetic-controller-interface ros-kinetic-controller-manager ros-kinetic-joint-limits-interface ros-kinetic-transmission-interface ros-kinetic-control-toolbox ros-kinetic-rosparam-shortcuts
#installs NeoVim Python bindings (for rosvim)
sudo pip2 install --upgrade neovim
sudo pip3 install --upgrade neovim

#install vim-plug (this will likely break any custom vim configs you have)
#for Vim
curl -fLo ~/.vim/autoload/plug.vim --create-dirs \
    https://raw.githubusercontent.com/junegunn/vim-plug/master/plug.vim
#for NeoVim
curl -fLo ~/.local/share/nvim/site/autoload/plug.vim --create-dirs \
    https://raw.githubusercontent.com/junegunn/vim-plug/master/plug.vim
#add to vim config
cat <<EOF >> ~/.vimrc
call plug#begin()
Plug 'taketwo/vim-ros'
Plug 'tpope/vim-sensible'
call plug#end()
EOF
#add to neovim config
cat <<EOF >> ~/.config/nvim/init.vim
call plug#begin()
Plug 'taketwo/vim-ros'
Plug 'tpope/vim-sensible'
call plug#end()
EOF
#end install of vim-plug

#add aliases to .bashrc
echo '
if [[ -n $SSH_CONNECTION ]]; then
 export EDITOR='vim'
else
 export EDITOR='nvim'
fi' >> ~/.bashrc
echo alias vi=\'\$EDITOR\' >> ~/.bash_aliases
echo alias vim=\'\$EDITOR\' >> ~/.bash_aliases
echo 'Do you wish to add "source /opt/ros/kinetic/setup.bash" to your .bashrc? If unsure or ROS has already been installed, select YES'
select yn in "Yes" "No"; do
    case $yn in
        Yes ) echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc; break;;
        No ) break;;
    esac
done
