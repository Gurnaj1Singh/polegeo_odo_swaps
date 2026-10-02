import rosbag
from tqdm import tqdm

def remove_specified_topics(input_bag_path, output_bag_path, topics_to_remove):
    """
    Removes the specified topics from the input ROS bag file and saves the rest of the data to the output ROS bag file.

    Args:
    input_bag_path (str): The path to the input ROS bag file.
    output_bag_path (str): The path to the output ROS bag file.
    topics_to_remove (list): A list of topics to be removed from the ROS bag file.
    """
    with rosbag.Bag(input_bag_path, 'r') as input_bag, rosbag.Bag(output_bag_path, 'w') as output_bag:
        for topic, msg, t in tqdm(input_bag.read_messages(), total=input_bag.get_message_count()):
            if topic not in topics_to_remove:
                output_bag.write(topic, msg, t)

# Specify the topics to remove
topics_to_remove = [
    '/ouster/points',
    '/interfacea/link0/camera_info',
    '/interfacea/link0/image/h264',
    '/interfacea/link1/camera_info',
    '/interfacea/link1/image/h264',
    '/interfacea/link2/camera_info',
    '/interfacea/link2/image/h264',
    '/interfacea/link3/camera_info',
    '/interfacea/link3/image/h264',
    '/vehicle/brake_cmd',
    '/vehicle/brake_report',
    '/vehicle/dbw_enabled',
    '/vehicle/gear_report',
    '/vehicle/steering_cmd',
    '/vehicle/steering_report',
    '/vehicle/throttle_cmd',
    '/vehicle/throttle_report',
    '/vehicle_drive_path',
    '/vehicle_enu_heading'
]

# Example usage
input_bag_path = '2024-02-28-12-59-51.bag'
output_bag_path = './2024-02-28-12-59-51_no_unwanted_topics.bag'
remove_specified_topics(input_bag_path, output_bag_path, topics_to_remove)