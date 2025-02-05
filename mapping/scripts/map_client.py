#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
import sys
import math
import rospkg
import yaml
import numpy as np
from PIL import Image
from mapping.srv import MapData


def test_load(package_name="mapping", map_folder="maps", map_name="map"):
    """
    Load a PGM map and YAML configuration.
    param package_name: Name of the catkin package.
    param map_folder: Desired Folder to save.
    param map_name:  Name for map file.
    return: occupancy grid and metadata. 
    """
    
    # Get the package path dynamically
    rospack = rospkg.RosPack()
    package_path = rospack.get_path(package_name)
    
    # Build paths to map files
    yaml_path = f"{package_path}/{map_folder}/{map_name}.yaml"
    pgm_path = f"{package_path}/{map_folder}/{map_name}.pgm"

    # Load YAML metadata
    with open(yaml_path, "r") as file:
        metadata = yaml.safe_load(file)

    # Load PGM image and Convert to grayscale
    img = Image.open(pgm_path).convert("L") 
    # Flip so image will be correctly positioned
    img = img.transpose(Image.FLIP_TOP_BOTTOM)  

    # Get NumPy array
    occupancy_grid = np.array(img, dtype=np.int8)

    # Get all uniquie values in array
    variba = np.unique(occupancy_grid)
    print(variba)

    # Create occupancy grid object
    map_msg = OccupancyGrid()
    # Set metadata

    # Set all metadata
    width,height = img.size
    map_msg.header.stamp = rospy.Time.now()
    map_msg.header.frame_id = "map"
    map_msg.info.resolution = metadata['resolution']
    map_msg.info.width = width
    map_msg.info.height = height
    map_msg.info.origin.position.x = metadata['origin'][0]
    map_msg.info.origin.position.y = metadata['origin'][1]
    map_msg.info.origin.position.z = metadata['origin'][2]

  
    print("Unique Values - " + str(np.unique(occupancy_grid)))

    # Convert grayscale values to occupancy grid  values (-1, 0, 100)
    occupancy_grid[occupancy_grid == -51] = -1 # Unknown
    occupancy_grid[occupancy_grid == 0] = 100 # Occupied
    occupancy_grid[occupancy_grid == -2] = 0 # Free

    rospy.loginfo(f"Loaded map from {pgm_path} with size {occupancy_grid.shape}")

    # Go from NumPy array back to occupancy grid
    map_msg.data = occupancy_grid.flatten().tolist()
    print(str(len(list(img.getdata()))))
    print("PATH - " + yaml_path )
    return map_msg, metadata, yaml_path

def save_map( yaml_path, occupancy_grid, metadata, save_folder="maps", save_name="final_map777"):
    """
    Saves the modified occupancy grid  and YAML configuration.
    
    occupancy_grid: Occupancy grid.
    metadata: Metadata from YAML.
    save_folder: Folder to save map.
    save_name: File name.
    """
    # Get package
    rospack = rospkg.RosPack()
    package_path = rospack.get_path("mapping")

    final_yaml_path = f"{package_path}/{save_folder}/{save_name}.yaml"
    print("PATH - " + save_folder)

    # Desired path for pgm map
    pgm_path = f"{save_folder}/{save_name}.pgm"
    
    # Get NumPy array from occupancy grid
    occupancy_data = np.array(occupancy_grid.data, dtype=np.int8)
    occupancy_reshaped = occupancy_data.reshape((384,384))

    # Print unique values
    variba = np.unique(occupancy_reshaped)
    print(variba)

    # Convert occupancy grid back to grayscale

    # Default free
    grayscale_map = np.full_like(occupancy_reshaped, 255, dtype=np.uint8)  
    # Occupied
    grayscale_map[occupancy_reshaped == 0] = 0   
    # Uknown area
    grayscale_map[occupancy_reshaped == -1] = 205   # uknown

    # Noise area
    grayscale_map[occupancy_reshaped == 6] = 255   # black
    grayscale_map[occupancy_reshaped == 7] = 255 # white
    grayscale_map[occupancy_reshaped == 8] = 255  # white

    # 0 - black (Obstacle)
    # 255 - white (Free)
    # 205 - gray (Uknown)
 
    # Save as PGM
    img = Image.fromarray(grayscale_map)
    img.save(f"{package_path}/{save_folder}/{save_name}.pgm", format="PPM")  # ROS maps use binary PGM (P5), PIL auto-detects

    # Save YAML configuration and PGM map
    metadata["image"] = f"{save_name}.pgm"
    print(metadata)
    with open(f"{package_path}/{save_folder}/{save_name}.yaml", "w") as yaml_file:
        yaml.dump(metadata, yaml_file, default_flow_style=False)

    print("PATH - " + yaml_path)

    print(f"Map saved as {pgm_path} and {yaml_path}")

def send_request(map, metadata):
    rospy.wait_for_service('map_service')
    try:
        map_exchange = rospy.ServiceProxy('map_service', MapData)
        rospy.loginfo("Sending map to server...")
        response = map_exchange(map)
        print("SEND")
        rospy.loginfo("Received response map from server.")

        return response.map

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return None

    

def publish_map(occupancy_grid_msg):
    """Continuously publish the OccupancyGrid map to /new_map topic."""
    pub = rospy.Publisher('/new_map', OccupancyGrid, queue_size=10)
     # Publish at 5 Hz (5 times per second)
    rate = rospy.Rate(5) 

    # Publish in a while cycle
    while not rospy.is_shutdown():
        occupancy_grid_msg.header.stamp = rospy.Time.now()
        pub.publish(occupancy_grid_msg)
        rospy.loginfo("Published occupancy grid to /new_map")
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('map_client')
        print("Started service-client")
        occupancy, metadata, yaml_path = test_load()
        print("Got map from folder")
        print("Sending map to the server")
        received_map = send_request(occupancy, metadata)
        if received_map:
            rospy.loginfo(f"Received map of size: {received_map.info.width}x{received_map.info.height}")    

        publish_map(received_map)
    except rospy.ROSInterruptException:
        pass

