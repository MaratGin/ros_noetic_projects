#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap, GetMapResponse
import sys
import math
import rospkg
import yaml
import numpy as np
from PIL import Image
from mapping.srv import MapData


def remove_noise_clusters(matrix, threshold, clear_black=False, clear_white=False, clear_gray=False):
    """
    Remove noise clusters from a 2D NumPy array containing values -1, 0, and 1.
    A cluster is removed if it has a size less than or equal to the threshold.
    """
    delete_pixel = 6
    detect_pixel = 0


    if clear_white:
        delete_pixel = 7
        detect_pixel = -2
    if clear_black:
        delete_pixel = 6
        detect_pixel = 0

    if clear_gray:
        delete_pixel = 8
        detect_pixel = -1

    if matrix.size == 0:
        return matrix

    rows, cols = matrix.shape
    visited = np.zeros((rows, cols), dtype=bool)  # Track visited cells
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # 4-connected neighbors

    def get_cluster(r, c):
        """Finds all connected cells of the same value starting from (r, c)."""
        value = matrix[r, c]
        stack = [(r, c)]
        cluster = []
        visited[r, c] = True

        while stack:
            cr, cc = stack.pop()
            cluster.append((cr, cc))
            for dr, dc in directions:
                nr, nc = cr + dr, cc + dc
                if 0 <= nr < rows and 0 <= nc < cols:
                    if not visited[nr, nc] and matrix[nr, nc] == value:
                        visited[nr, nc] = True
                        stack.append((nr, nc))
        return cluster, value

    for r in range(rows):
        for c in range(cols):
            if not visited[r, c] and matrix[r, c] != detect_pixel:  # Ignore background (0)
                cluster, value = get_cluster(r, c)
                if len(cluster) <= threshold:
                    for cr, cc in cluster:
                        print("CLEAR " + str(cr) + str(cc))
                        matrix[cr, cc] = delete_pixel  # Remove noise clusters by setting to 0
    return matrix

def handle_map_request(req):
    rospy.loginfo("Received an OccupancyGrid request.")
    base_req = req
    array = req.map
    print(len(array.data))
    # print(array.data[1].shape)
    occupancy_data = np.array(array.data, dtype=np.int8)
    occupancy_reshaped = occupancy_data.reshape((384,384))
    answer_black = remove_noise_clusters(occupancy_reshaped, 4, clear_black=True)
    answer_white = remove_noise_clusters(answer_black, 4, clear_white=True)
    answer_gray = remove_noise_clusters(answer_white, 4, clear_gray=True)

    print("Save changes image")
    save_map_cnanges(answer_gray)


    answer_gray[answer_gray == 6] = 0
    answer_gray[answer_gray == 7] = 0
    answer_gray[answer_gray == 8] = 0
    # answer_gray[answer_gray == -1]
    # answer_gray[answer_gray == -2] = 0
    # answer_gray[answer_gray == 0] = 100


    flattened = answer_gray.flatten().tolist()
    base_req.map.data = flattened
    # print(occupancy_reshaped)
    print("Data type - " + str(type(array)) )
    print("Shape - " + str(occupancy_reshaped.shape))
    map_msg = OccupancyGrid()
    # print(req)
    # Set metadata
    map_msg.header.stamp = rospy.Time.now()
    map_msg.header.frame_id = "map"
    map_msg.info.resolution = req.map.info.resolution
    map_msg.info.width = req.map.info.width
    map_msg.info.height = req.map.info.height
    map_msg.info.origin = req.map.info.origin
    map_msg.data = flattened

    
    # Simply return the same map received
    # response = MapExchangeResponse()
    # response.map = req.map
    return map_msg

def map_server():
    rospy.init_node('map_server')
    service = rospy.Service('map_service', MapData, handle_map_request)
    rospy.loginfo("Map Server Ready.")
    rospy.spin()



def save_map_cnanges(occupancy_grid, save_folder="maps", save_name="changes"):
    """
    Saves the modified occupancy grid as a PGM and YAML file.
    
    :param occupancy_grid: The NumPy array representing the occupancy grid.
    :param metadata: The dictionary containing the map's metadata.
    :param save_folder: Folder where the map should be saved.
    :param save_name: Base name for the saved files (without extension).
    """
    rospack = rospkg.RosPack()
    package_path = rospack.get_path("mapping")

    pgm_path = f"{save_folder}/{save_name}.pgm"
    print("Unique Values - " + str(np.unique(occupancy_grid)))
    grayscale_map = np.full_like(occupancy_grid, 255, dtype=np.uint8)  # Default unknown

    grayscale_map[occupancy_grid == 0] = 0   # occupied
    grayscale_map[occupancy_grid == -1] = 0   # uknown
    grayscale_map[occupancy_grid == 100] = 0   # uknown

    grayscale_map[occupancy_grid == 6] = 255   # black
    grayscale_map[occupancy_grid == 7] = 255 # white
    grayscale_map[occupancy_grid == 8] = 255  # white

    # Save as PGM
    img = Image.fromarray(grayscale_map)
    img = img.transpose(Image.FLIP_TOP_BOTTOM)  # Flip to match ROS coordinates

    img.save(f"{package_path}/{save_folder}/{save_name}.pgm", format="PPM")  # ROS maps use binary PGM (P5), PIL auto-detects

    print(f"Changes map saved as {pgm_path} and {save_name}.pgm")


if __name__ == "__main__":
    print("START")
    map_server()
# Example usage: