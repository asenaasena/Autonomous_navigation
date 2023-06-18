import os
import sys


#do not change 
itr = 1             # Iteration initialisation
bag = '.bag'        # bag file extension
n = 'n'             # none 
lz4 = '--lz4'       # compress type lz4 
bz2 = '-q'          # compress type bz2 

#change 
node_name = ['n03','n05','n06','n11','n13','n15','n18','n19']                 # list of node_number 
bag_dir = "/home/asena/lidar/"                       # input bag dir 
tf_changed_dir = "/home/asena/lidar/tf_changed/"     # tf_changed bag save dir 
comp_type= lz4                                        # choose comress type 



while itr<len(node_name)+1:
    

    for file in os.listdir(bag_dir):
        if node_name[itr-1] in file:
            print(file, 'detected')
            file_1= file.replace(bag,"")
            id_name = node_name[itr-1].replace(n,'')
            print("change id to base{0}/os1_lidar".format(id_name))
            cmd_1 = "python3 change_frame_id.py -o {3}{0}_tf.bag -i {2}{0}.bag -f base{1}/os1_lidar\
            -t TOPIC /node{1}/os1_cloud_node/points && rosbag compress\
             {4} {3}{0}_tf.bag && rm {3}{0}_tf.orig.bag".format(file_1,id_name,bag_dir,tf_changed_dir,comp_type)
            os.system(cmd_1)
            
    print(node_name[itr-1], 'done')
    itr = itr +1 

print('id change done')



