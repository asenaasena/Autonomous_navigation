
# While working with Visual Studio Code, you can install extension C/CPP AND CMake Tools extensions

#We will work with Cmake 3.16.7
#You need to install cmake using the commands below

#1- Firstly, download the files of the required version of the corresponding system in cmake official by typing the line below into your terminal.

wget https://cmake.org/files/v3.16/cmake-3.16.7-Linux-x86_64.tar.gz  


#2- Unzip the files using tar command.

tar zxvf cmake-3.16.7-Linux-x86_64.tar.gz

#3- View unzipped files

tree -L 2 cmake-3.16.7-Linux-x86_64

#4- If you get an error saying there's no tree command, install it by using the following line

sudo apt install tree

#5-Create soft link

sudo mv cmake-3.16.7-Linux-x86_64     /opt/cmake-3.16.7  
sudo ln -sf  /opt/cmake-3.16.7/bin/*    /usr/bin/

#6- Execute the following command to check the installed version

cmake --version


#-You need to install pcl

sudo apt install libpcl-dev


sudo apt-get install ros-melodic-jsk-recognition-msgs & sudo apt-get install ros-melodic-jsk-rviz-plugins