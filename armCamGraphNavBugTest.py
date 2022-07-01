import bosdyn.client
import bosdyn.client.util
from bosdyn.client.image import ImageClient
from PIL import Image
import io
import time

from bosdyn.client.graph_nav import GraphNavClient
from bosdyn.api.graph_nav import graph_nav_pb2
from bosdyn.api.graph_nav import map_pb2
from bosdyn.api.graph_nav import nav_pb2
from bosdyn.client.frame_helpers import get_odom_tform_body
from bosdyn.client.exceptions import ResponseError
from bosdyn.api import image_pb2

def print_image_cap_times(image_client, start_time):
    image_response_hand = image_client.get_image_from_sources(["hand_color_image"])[0]

    image_response_frontleft = image_client.get_image_from_sources(["frontleft_fisheye_image"])[0]

    print("hand color image acquisition_time: ", image_response_hand.shot.acquisition_time.seconds - start_time)
    print("frontleft fisheye image acquisition_time: ", image_response_frontleft.shot.acquisition_time.seconds - start_time)

def image_graph_test(*, image_client, graph_nav_client, upload_filepath, start_time):
    """
    IMAGE + GRAPH TEST:
    1. Gets 5 images from hand camera and frontleft camera (1 second interval)
    2. Uploads graph
    3. waits 5 seconds
    4. Gets another 5 images from hand camera and frontleft camera (1 second interval)
    """

    # Step 1
    for i in range(5):
        time.sleep(1)
        print_image_cap_times(image_client, start_time)
    # ARYA OBSERVED: Hand camera and frontleft camera had matching acquisition
    # times, both incrementing over 5 seconds
    # occasionally, hand camera reports weird acquisition time on first image retrieval

    # Step 2
    current_graph = None
    current_waypoint_snapshots = dict()
    current_edge_snapshots = dict()
    with open(upload_filepath + "/graph", "rb") as graph_file:
        # Load the graph from disk.
        data = graph_file.read()
        current_graph = map_pb2.Graph()
        current_graph.ParseFromString(data)
        print("Loaded graph has {} waypoints and {} edges".format(
            len(current_graph.waypoints), len(current_graph.edges)))

    response = graph_nav_client.upload_graph(graph=current_graph,
                                                   generate_new_anchoring=False)

    print("GRAPH UPLOAD COMPLETE")

    # Step 3
    time.sleep(5)

    # Step 4
    for i in range(5):
        time.sleep(1)
        print_image_cap_times(image_client, start_time)
    # ARYA OBSERVED: Hand camera acquisition_time stuck while frontleft camera
    # acquisition_time incremented over 5 seconds

    # Demonstrates that accessing hand camera after uploading graph
    # returns an old image, taken roughly at the time of the graph upload.
    # Hand camera fails to obtain any new images after the graph upload.
    # This bug was observed for the other image sources on the hand as well
    # but not for the mobility camera image sources on the base robot.

def main():

    start_time = round(time.time())
    #Setup
    sdk = bosdyn.client.create_standard_sdk('ASTRO')
    #TODO: CHANGE ACCORDINGLY
    ROBOT_IP = '192.168.50.3'
    robot = sdk.create_robot(ROBOT_IP)
    #Authenticate
    bosdyn.client.util.authenticate(robot)

    #Graph Nav
    graph_nav_client = robot.ensure_client(GraphNavClient.default_service_name)
    #Tried multiple graphs, ranging from 1 to ~80 waypoints in size, same failure each time
    #TODO: CHANGE ACCORDINGLY
    upload_filepath = '/home/gluser/groundlight/bdspot/smallGraph'
    response = graph_nav_client.clear_graph()

    #image client
    image_client = robot.ensure_client(ImageClient.default_service_name)
    print("GETTING IMAGES BEFORE UPLOADING GRAPH:")
    image_graph_test(image_client = image_client,
                     graph_nav_client = graph_nav_client,
                     upload_filepath = upload_filepath,
                     start_time = start_time)

if __name__ == '__main__':
    main()
