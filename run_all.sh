
# build everything
#gnome-terminal -- roslaunch mueavi_tf mueavi_tf.launch

cd n03
./build.sh

cd ../mueavi_tf
./build.sh

cd ../eu_cluster
./build.sh

#gnome-terminal -- ./path/eu_cluster

gnome-terminal -- roslaunch eu_cluster eu_cluster.launch

  
