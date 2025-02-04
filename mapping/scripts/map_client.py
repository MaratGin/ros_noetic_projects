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

grid = []

def printGridToFile():
    global grid
    rospy.loginfo("Print info to file grid_py.txt")
    fw = open('grid_py_test.txt', 'w')
    for g in grid:
        for g1 in g:
            if (g1 == True):
                fw.write("1")
            else:
                fw.write("0")
        fw.write("\n")


def requestMap():
    global grid
    rospy.init_node('load_map', anonymous=False)
    rospy.wait_for_service('static_map')
    try:
        static_map = rospy.ServiceProxy("static_map", GetMap)
        map = static_map().map
        rows = map.info.height
        cols = map.info.width
        rospy.loginfo("rows=%i, cols=%i", rows, cols)
        currCell = 0
        i = 0
        while i < rows:
            j = 0
            grid.append([])
            while j < cols:
                if (map.data[currCell] == 0):
                    # Free space
                    grid[i].append(False)
                else:
                    # At least 1% occupied, so we consider it as occupied
                    grid[i].append(True)
                j = j + 1
                currCell = currCell + 1
            i = i + 1

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
    rospy.loginfo("Write to grid finished")



def test_load(package_name="mapping", map_folder="maps", map_name="map"):
    """
    Loads a map (PGM and YAML) from the specified package and folder.
    
    :param package_name: Name of the catkin package containing the map.
    :param map_folder: Subfolder inside the package where maps are stored.
    :param map_name: Base name of the map files (without extension).
    :return: occupancy grid (numpy array), metadata (dict)
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

    # Load PGM image
    img = Image.open(pgm_path).convert("L")  # Convert to grayscale
    img = img.transpose(Image.FLIP_TOP_BOTTOM)  # Flip to match ROS coordinates
    occupancy_grid = np.array(img, dtype=np.int8)
    variba = np.unique(occupancy_grid)
    print(variba)
    map_msg = OccupancyGrid()
    # Set metadata


    width,height = img.size

    map_msg.header.stamp = rospy.Time.now()
    map_msg.header.frame_id = "map"
    map_msg.info.resolution = metadata['resolution']
    map_msg.info.width = width
    map_msg.info.height = height
    map_msg.info.origin.position.x = metadata['origin'][0]
    map_msg.info.origin.position.y = metadata['origin'][1]
    map_msg.info.origin.position.z = metadata['origin'][2]


    # Convert grayscale values to occupancy grid (-1, 0, 100)
    print("Unique Values - " + str(np.unique(occupancy_grid)))

    occupancy_grid[occupancy_grid == -51] = -1 # Unknown
    occupancy_grid[occupancy_grid == 0] = 100 # Occupied
    occupancy_grid[occupancy_grid == -2] = 0 # Free

    rospy.loginfo(f"Loaded map from {pgm_path} with size {occupancy_grid.shape}")

    map_msg.data = occupancy_grid.flatten().tolist()
    print(str(len(list(img.getdata()))))
    print("PATH - " + yaml_path )
    return map_msg, metadata, yaml_path

def save_map( yaml_path, occupancy_grid, metadata, save_folder="maps", save_name="final_map777"):
    """
    Saves the modified occupancy grid as a PGM and YAML file.
    
    :param occupancy_grid: The NumPy array representing the occupancy grid.
    :param metadata: The dictionary containing the map's metadata.
    :param save_folder: Folder where the map should be saved.
    :param save_name: Base name for the saved files (without extension).
    """
    rospack = rospkg.RosPack()
    package_path = rospack.get_path("mapping")
    final_yaml_path = f"{package_path}/{save_folder}/{save_name}.yaml"
    print("PATH - " + save_folder)

    pgm_path = f"{save_folder}/{save_name}.pgm"
    
    occupancy_data = np.array(occupancy_grid.data, dtype=np.int8)
    occupancy_reshaped = occupancy_data.reshape((384,384))
    variba = np.unique(occupancy_reshaped)

    print(variba)
    # Convert occupancy grid back to grayscale (255=free, 0=obstacle, 205=unknown)
    # 255
    grayscale_map = np.full_like(occupancy_reshaped, 255, dtype=np.uint8)  # Default unknown
    grayscale_map[occupancy_reshaped == 0] = 0   # occupied
    # 205
    grayscale_map[occupancy_reshaped == -1] = 205   # uknown
    grayscale_map[occupancy_reshaped == 6] = 255   # black
    grayscale_map[occupancy_reshaped == 7] = 255 # white
    grayscale_map[occupancy_reshaped == 8] = 255  # white



    # grayscale_map[occupancy_reshaped == -2] = 205   # Uknown

    # grayscale_map[occupancy_grid == -2] = -1   # Obstacles

    # 0 - black (Obstacle)
    # 255 - white (Free)
    # 205 - gray (Uknown)
 
    # Save as PGM
    img = Image.fromarray(grayscale_map)
    img.save(f"{package_path}/{save_folder}/{save_name}.pgm", format="PPM")  # ROS maps use binary PGM (P5), PIL auto-detects

    # Save YAML metadata
    # metadata["image"] = f"{save_name}.pgm"
    metadata["image"] = f"{save_name}.pgm"
    print(metadata)
    # with open(yaml_path, "r") as file:
    #     data = yaml.safe_load(file)
    # data['image'] = "example2.pgm"
    with open(f"{package_path}/{save_folder}/{save_name}.yaml", "w") as yaml_file:
        yaml.dump(metadata, yaml_file, default_flow_style=False)

    print("PATH - " + yaml_path)
    # print(data)
    # f.close()
    # with open("/home/marat/catkin_ws/src/mapping/maps/modified_map.pgm", "w") as yaml_file:
        # yaml.dump(metadata, yaml_file, default_flow_style=False)

    print(f"Map saved as {pgm_path} and {yaml_path}")

def send_request(map, metadata):
    print("SEND 1")
    rospy.wait_for_service('map_service')
    print("SEND 2")
    try:
        print("SEND 3")
        map_exchange = rospy.ServiceProxy('map_service', MapData)
        print("SEND 4")
        rospy.loginfo("Sending map to server...")
        print("SEND 5" + str(type(map)))
        response = map_exchange(map)
        print("SEND 6")
        rospy.loginfo("Received response map from server.")

        return response.map

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return None

    

def publish_map(occupancy_grid_msg):
    """Continuously publish the OccupancyGrid map to /new_map topic."""
    pub = rospy.Publisher('/new_map', OccupancyGrid, queue_size=10)
    rate = rospy.Rate(5)  # Publish at 1 Hz (once per second)

    # Load the map once
    map_yaml_path = "/path/to/map.yaml"  # Change to your map.yaml path

    while not rospy.is_shutdown():
        # Update timestamp and publish map
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
        # save_map(yaml_path, received_map, metadata)
        # requestMap()
        # printGridToFile()
    except rospy.ROSInterruptException:
        pass

