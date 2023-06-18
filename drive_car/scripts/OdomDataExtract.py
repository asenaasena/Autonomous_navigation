import rosbag
import csv

# Load the ROS bag file
bag = rosbag.Bag('/pathtodata/lidarg1.bag')

# Define the topic to extract data from
topic_name = '/gps/odom'
begin_time = 1680605578


# Define the output file name
csv_file = 'OdomData.csv' # the file to store data
header = ['Time', 'Pos_X', 'Pos_Y', 'Pos_Z', 'Orient_X', 'Orient_Y', 'Orient_Z', 'Orient_W'] # the CSV file header


def callback(msg):
    with open(csv_file, 'a') as f:
        writer = csv.writer(f)
        writer.writerow([msg.header.stamp.secs, msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])

# Open the output file for writing
with open(csv_file, 'w') as f:
    writer = csv.writer(f)
    writer.writerow(header)

    # Iterate through each message in the ROS bag file
    for topic, msg, t in bag.read_messages():
        # Check if the message is on the topic of interest
        if topic == topic_name:
            if round(msg.header.stamp.secs) == begin_time:
                callback(msg)
                begin_time=begin_time+1


# Close the output file and ROS bag file
f.close()
bag.close()
