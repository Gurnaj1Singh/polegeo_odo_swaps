import matplotlib.pyplot as plt
import numpy as np
import bagpy
from bagpy import bagreader
import cv2
import torch

# Load the bag file using bagpy
bag = bagreader('2024-02-28-12-59-51.bag')


# Initialize lists to store data for each channel
signal_image_data = []
nearir_image_data = []
reflec_image_data = []
range_image_data = []

# Read signal, near IR, and reflection image data from the bag file
for topic, msg, t in bag.reader.read_messages():
    if topic == '/ouster/signal_image':
        signal_image_data.append(np.frombuffer(msg.data, dtype=np.uint16).reshape((msg.height, msg.width)))

    elif topic == '/ouster/nearir_image':
        nearir_image_data.append(np.frombuffer(msg.data, dtype=np.uint16).reshape((msg.height, msg.width)))
    elif topic == '/ouster/reflec_image':
        reflec_image_data.append(np.frombuffer(msg.data, dtype=np.uint16).reshape((msg.height, msg.width)))
    elif topic == '/ouster/range_image':
        range_image_data.append(np.frombuffer(msg.data, dtype=np.uint16).reshape((msg.height, msg.width)))
# print(len(signal_image_data))
# print(len(nearir_image_data))
# print(len(reflec_image_data))



# Loop through each frame
for i in range(len(signal_image_data)):
    # Stack the frames from each channel to create an RGB image
    rgb_image = np.stack((signal_image_data[i], nearir_image_data[i], reflec_image_data[i]), axis=-1)
    # rgb_image = np.stack((nearir_image_data[i], signal_image_data[i], reflec_image_data[i]), axis=-1)
    # print(rgb_image.shape)

    # Normalize or scale the image data to uint8
    rgb_image = ((rgb_image - rgb_image.min()) / (rgb_image.max() - rgb_image.min()) * 255).astype(np.uint8)

    # Stack the frames from each range image to create an RGB image
    range_image = np.stack((range_image_data[i], range_image_data[i], range_image_data[i]), axis=-1)
    ## Normalize or scale the image data to uint8
    range_image = ((range_image - range_image.min()) / (range_image.max() - range_image.min()) * 255).astype(np.uint8)


    #print(rgb_image.shape)
    #rgb_results = model(rgb_image)
    #rgb_annotated_img = rgb_results.render()[0]

    #Display the BGR image using OpenCV
    cv2.imshow("Signal_Image from ROS topic", signal_image_data[i])
    cv2.waitKey(10)  # Wait for a key press to advance to the next frame

    #Display the BGR image using OpenCV
    cv2.imshow("Near IR_Image from ROS topic", nearir_image_data[i])
    cv2.waitKey(10)  # Wait for a key press to advance to the next frame

    #Display the BGR image using OpenCV
    cv2.imshow("Reflective_Image from ROS topic", reflec_image_data[i])
    cv2.waitKey(10)  # Wait for a key press to advance to the next frame

    #Display the BGR image using OpenCV
    cv2.imshow("Range_Image from ROS topic", range_image)
    cv2.waitKey(10)  # Wait for a key press to advance to the next frame
    

    # Display the BGR image using OpenCV
    cv2.imshow("RGB_Image from ROS topic", rgb_image)
    #cv2.imshow("RGB_Image from ROS topic", rgb_annotated_img)
    cv2.waitKey(0)  # Wait for a key press to advance to the next frame

cv2.destroyAllWindows()  # Close all OpenCV windows once all frames are shown