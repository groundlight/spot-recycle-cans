#Basic imports
import numpy as np
import matplotlib.pyplot as plt
import io
import os
from copy import deepcopy
import cv2
import time
from PIL import Image
import logging

#Bosdyn client imports
import bosdyn.client
import bosdyn.client.util
import bosdyn.client.robot_command as bdcrc
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn.client.robot_command import RobotCommandClient
from bosdyn.client.image import ImageClient
from bosdyn.client.graph_nav import GraphNavClient
from bosdyn.client.manipulation_api_client import ManipulationApiClient
from bosdyn.client.world_object import WorldObjectClient
from bosdyn.client.frame_helpers import (get_a_tform_b,
                                        get_vision_tform_body,
                                        get_odom_tform_body,
                                        BODY_FRAME_NAME,
                                        VISION_FRAME_NAME)

#Protocol buffer imports
from bosdyn.api import (arm_command_pb2,
                       geometry_pb2,
                       robot_command_pb2,
                       synchronized_command_pb2,
                       trajectory_pb2,
                       world_object_pb2,
                       manipulation_api_pb2,
                       image_pb2,
                       estop_pb2,
                       robot_state_pb2,
                       image_pb2)
from bosdyn.api.graph_nav import graph_nav_pb2, map_pb2, nav_pb2

#Other bosdyn imports
from bosdyn.util import seconds_to_duration
from bosdyn.geometry import EulerZXY
from bosdyn.client.exceptions import ResponseError

#Internal repo imports
import constants


def init_robot(IP):
    #Setup
    sdk = bosdyn.client.create_standard_sdk('ASTRO')
    robot = sdk.create_robot(IP)
    id_client = robot.ensure_client('robot-id')

    #Authenticate
    bd_username = os.environ.get("BOSDYN_CLIENT_USERNAME")
    bd_password = os.environ.get("BOSDYN_CLIENT_PASSWORD")
    robot.authenticate(bd_username, bd_password)

    #Clients
    robot_state_client = robot.ensure_client('robot-state')
    command_client = robot.ensure_client(RobotCommandClient.default_service_name)
    image_client = robot.ensure_client(ImageClient.default_service_name)

    graph_nav_client = robot.ensure_client(GraphNavClient.default_service_name)
    world_object_client = robot.ensure_client(WorldObjectClient.default_service_name)
    manipulation_api_client = robot.ensure_client(ManipulationApiClient.default_service_name)

    return sdk, robot, id_client, robot_state_client, command_client, image_client, \
           graph_nav_client, world_object_client, manipulation_api_client

def gl_init():
    GROUNDLIGHT_API_TOKEN = os.environ.get("GROUNDLIGHT_API_TOKEN")
    gl = Groundlight(api_token=GROUNDLIGHT_API_TOKEN)
    return gl

def get_batt_info(robot_state_client):
    robot_state = robot_state_client.get_robot_state()
    battery_state = robot_state.battery_states[0]
    charge_percent = battery_state.charge_percentage.value
    runtime_remaining = battery_state.estimated_runtime.seconds
    print("Battery at:", charge_percent, "%. Robot has", runtime_remaining, "seconds left")
    return charge_percent, runtime_remaining

def get_lease(robot):
    lease_client = robot.ensure_client('lease')
    lease = lease_client.acquire()
    lease_keep_alive = bosdyn.client.lease.LeaseKeepAlive(lease_client)
    return lease_client, lease, lease_keep_alive

def power_on(robot):
    robot.power_on(timeout_sec=20)
    logging.info(robot.is_powered_on())
    robot.time_sync.wait_for_sync()
    return True

def cap_hand_image(image_client, source):
    dtype = None
    channels = None

    if source == 'hand_depth_in_hand_color_frame':
        dtype = np.uint8
        channels = 2
    elif source == 'hand_color_image':
        dtype = np.uint8
        channels = 3
    else:
        logging.error('NOT A SUPPORTED IMAGE SOURCE')
        return None

    image_response = image_client.get_image_from_sources([source])[0]
    img = np.frombuffer(image_response.shot.image.data, dtype=dtype)

    if source == 'hand_depth_in_hand_color_frame':
        img = img.reshape((image_response.shot.image.rows, image_response.shot.image.cols, channels))
    else:
        img = cv2.imdecode(img, -1)

    return img, image_response

def move_robot_relative(command_client, robot,
                        x_meters,
                        y_meters,
                        theta_rad,
                        timeout_sec = 10, blocking = True):
    """
    Moves Spot relative to its current position.
    """
    frame_tree_snapshot = robot.get_frame_tree_snapshot()
    cmd = RobotCommandBuilder.synchro_trajectory_command_in_body_frame(x_meters, y_meters, theta_rad, frame_tree_snapshot = frame_tree_snapshot)
    cmd_id = cmd_id = command_client.robot_command(cmd, end_time_secs = time.time()+timeout_sec)
    if blocking:
        bdcrc.block_for_trajectory_cmd(command_client, cmd_id, timeout_sec = timeout_sec)

def make_grasp(grasp_request,
               manipulation_api_client,
               command_client,
               graph_nav_client = None, graphExists = True, verbose = False):
    """
    Makes a grasp with arm given a grasp request proto described here:
    https://dev.bostondynamics.com/protos/bosdyn/api/proto_reference#manipulation-api-proto

    Uploading a graph to graph_nav_client makes arm camera fail. Grasp WILL NOT
    WORK if graph is uploaded. First step of method is to clear graph if it exists.

    Graph must be reuploaded and position reinitialized later. Method must be
    called explicitly with graphExists = False if there is no graph in use.
    """

    if graphExists:
        graph_nav_client.clear_graph()

    cmd_response = manipulation_api_client.manipulation_api_command(
    manipulation_api_request = grasp_request)

    start_time_sec = time.time()

    # Get feedback from the robot
    response = None

    success = False
    while True:
        feedback_request = manipulation_api_pb2.ManipulationApiFeedbackRequest(
            manipulation_cmd_id=cmd_response.manipulation_cmd_id)

        # Send the request
        response = manipulation_api_client.manipulation_api_feedback_command(
            manipulation_api_feedback_request=feedback_request)

        if verbose:
            logging.info('Current state: ',
                  str(manipulation_api_pb2.ManipulationFeedbackState.Name(response.current_state)))

        if response.current_state == manipulation_api_pb2.MANIP_STATE_GRASP_SUCCEEDED:
            success = True
            break
        if response.current_state == manipulation_api_pb2.MANIP_STATE_GRASP_FAILED \
            or response.current_state == manipulation_api_pb2.MANIP_STATE_GRASP_PLANNING_NO_SOLUTION:
            break
        cur_time_sec = time.time()

        if cur_time_sec - start_time_sec > 15:
            logging.warning('GRASP TOOK TOO LONG, STOPPING NOW')
            response.current_state == manipulation_api_pb2.MANIP_STATE_GRASP_FAILED
            break

        time.sleep(0.25)

    override_response = stowable_override(manipulation_api_client)    
        
    cmd = RobotCommandBuilder.arm_stow_command()
    cmd_id = command_client.robot_command(cmd, end_time_secs = time.time()+5)

    return response, success

def stowable_override(manipulation_api_client):
    #overrides so object can be carried and stowed
    carry_override_rq = manipulation_api_pb2.ApiGraspedCarryStateOverride(override_request = 3)
    override_rq = manipulation_api_pb2.ApiGraspOverrideRequest(
        carry_state_override = carry_override_rq)
    override_response = manipulation_api_client.grasp_override_command(
        grasp_override_request = override_rq)
    
    return override_response

def move_to_and_look_at(command_client, move_to, look_at, blocking = True):
    """Takes in two, 3-element array like types.
        First is the coordinates for the gripper head location.
        Second is the coordinates for the gripper head to look at.
        Blocking argument will pause code until the move command is finished
    """

    arm_pos_list = [move_to[0],move_to[1],move_to[2]]
    dur_sec = 3

    duration = seconds_to_duration(dur_sec)


    arm_pos = geometry_pb2.Vec3(x=arm_pos_list[0],
                                y=arm_pos_list[1],
                                z=arm_pos_list[2])
    hand_pose = geometry_pb2.SE3Pose(position=arm_pos)
    hand_pose_traj_point = trajectory_pb2.SE3TrajectoryPoint(pose=hand_pose,
                                                             time_since_reference=duration)
    hand_traj = trajectory_pb2.SE3Trajectory(points=[hand_pose_traj_point])


    gaze_pos_list = [look_at[0],look_at[1],look_at[2]]
    gaze_pos = geometry_pb2.Vec3(x=gaze_pos_list[0],
                                 y=gaze_pos_list[1],
                                 z=gaze_pos_list[2])
    gaze_pos_traj_point = trajectory_pb2.Vec3TrajectoryPoint(point=gaze_pos)
    gaze_traj = trajectory_pb2.Vec3Trajectory(points=[gaze_pos_traj_point])

    gaze_cmd = arm_command_pb2.GazeCommand.Request(target_trajectory_in_frame1=gaze_traj,
                                                   frame1_name='body',
                                                   tool_trajectory_in_frame2=hand_traj,
                                                   frame2_name='body')

    arm_command = arm_command_pb2.ArmCommand.Request(arm_gaze_command=gaze_cmd)
    synchronized_command = synchronized_command_pb2.SynchronizedCommand.Request(arm_command=arm_command)
    command = robot_command_pb2.RobotCommand(synchronized_command=synchronized_command)
    gaze_command_id = command_client.robot_command(command)
    if blocking:
        try:
            bdcrc.block_until_arm_arrives(command_client, gaze_command_id, timeout_sec = time.time() + 3.0)
        except:
            logging.warning('ARM already at desired location and orientation')

def randomGazeDown():
    # NOT EXACT, used for roughly calculating points of maximum reach
    ARM_ORIGIN = np.array([0.35,0,0.1])
    ARM_LENGTH = 0.7

    arm_z_pos = np.random.uniform(0.2, ARM_LENGTH-0.2)
    arm_x_pos = np.random.uniform(0,(ARM_LENGTH**2 - arm_z_pos**2)**0.5)

    maxYPos = (ARM_LENGTH**2 - arm_x_pos**2 - arm_z_pos**2)**0.5
    arm_y_pos = np.random.uniform(-maxYPos, maxYPos)

    move_to = np.array([arm_x_pos, arm_y_pos, arm_z_pos]) + ARM_ORIGIN

    look_at = [np.random.uniform(1, 1.5),
              np.random.uniform(-0.4,0.4),
              np.random.uniform(-0.2,-1)]
    return move_to, look_at

def gripper_open(command_client, timeout_sec = 5):
    cmd = RobotCommandBuilder.claw_gripper_open_fraction_command(1.0)
    cmd_id = command_client.robot_command(cmd, end_time_secs = time.time()+timeout_sec)
    
def gripper_close(command_client, timeout_sec = 5):
    cmd = RobotCommandBuilder.claw_gripper_open_fraction_command(0.0)
    cmd_id = command_client.robot_command(cmd, end_time_secs = time.time()+timeout_sec)
    
def blocking_stow(command_client, timeout_sec = 10):
    cmd = RobotCommandBuilder.arm_stow_command()
    cmd_id = command_client.robot_command(cmd, end_time_secs = time.time() + timeout_sec)
    success = bdcrc.block_until_arm_arrives(command_client, cmd_id, 
                                            timeout_sec = time.time() + timeout_sec)
    return success

# From "graph_nav_command_line.py"
# https://github.com/boston-dynamics/spot-sdk/blob/master/python/examples/graph_nav_command_line/graph_nav_command_line.py
def upload_graph_and_snapshots(graph_nav_client, upload_filepath):

    current_graph = None
    current_waypoint_snapshots = dict()
    current_edge_snapshots = dict()

    """Upload the graph and snapshots to the robot."""
    logging.info("Loading the graph from disk into local storage...")
    with open(upload_filepath + "/graph", "rb") as graph_file:
        # Load the graph from disk.
        data = graph_file.read()
        current_graph = map_pb2.Graph()
        current_graph.ParseFromString(data)
        logging.info("Loaded graph has {} waypoints and {} edges".format(
            len(current_graph.waypoints), len(current_graph.edges)))

    for waypoint in current_graph.waypoints:
        # Load the waypoint snapshots from disk.
        with open(upload_filepath + "/waypoint_snapshots/{}".format(waypoint.snapshot_id),
                  "rb") as snapshot_file:
            waypoint_snapshot = map_pb2.WaypointSnapshot()
            waypoint_snapshot.ParseFromString(snapshot_file.read())
            current_waypoint_snapshots[waypoint_snapshot.id] = waypoint_snapshot

    for edge in current_graph.edges:
        if len(edge.snapshot_id) == 0:
            continue
        # Load the edge snapshots from disk.
        with open(upload_filepath + "/edge_snapshots/{}".format(edge.snapshot_id),
                  "rb") as snapshot_file:
            edge_snapshot = map_pb2.EdgeSnapshot()
            edge_snapshot.ParseFromString(snapshot_file.read())
            current_edge_snapshots[edge_snapshot.id] = edge_snapshot
    # Upload the graph to the robot.
    logging.info("Uploading the graph and snapshots to the robot...")
    true_if_empty = not len(current_graph.anchoring.anchors)
    response = graph_nav_client.upload_graph(graph=current_graph,
                                             generate_new_anchoring=true_if_empty)

    # Upload the snapshots to the robot.
    for snapshot_id in response.unknown_waypoint_snapshot_ids:
        waypoint_snapshot = current_waypoint_snapshots[snapshot_id]
        graph_nav_client.upload_waypoint_snapshot(waypoint_snapshot)
        logging.info("Uploaded {}".format(waypoint_snapshot.id))
    for snapshot_id in response.unknown_edge_snapshot_ids:
        edge_snapshot = current_edge_snapshots[snapshot_id]
        graph_nav_client.upload_edge_snapshot(edge_snapshot)
        logging.info("Uploaded {}".format(edge_snapshot.id))

    # The upload is complete! Check that the robot is localized to the graph,
    # and if it is not, prompt the user to localize the robot before attempting
    # any navigation commands.
    localization_state = graph_nav_client.get_localization_state()
    if not localization_state.localization.waypoint_id:
        # The robot is not localized to the newly uploaded graph.
        logging.warning("Upload complete! The robot is currently not localized to the map; please localize")
    return response

def create_waypoint_list(graph_nav_client):

    """
    Creates and returns ordered list of waypoints.
    """

    #Gets the current graph and reads off waypoints
    current_graph = graph_nav_client.download_graph()
    waypoint_list_raw = [(waypoint.annotations.name,waypoint.id) for waypoint in current_graph.waypoints]

    #Remove occurences of default waypoints
    for elem in waypoint_list_raw:
        if(elem[0] == 'default'):
            waypoint_list_raw.remove(elem)

    waypoint_list = [0] * len(waypoint_list_raw)
    for waypoint_num_string, waypoint_id in waypoint_list_raw:
        waypoint_num = int(waypoint_num_string[9:])
        waypoint_list[waypoint_num] = waypoint_id

    return waypoint_list

# From "graph_nav_command_line.py"
# https://github.com/boston-dynamics/spot-sdk/blob/master/python/examples/graph_nav_command_line/graph_nav_command_line.py
def set_initial_localization_fiducial(state_client, graph_nav_client):
    """Trigger localization when near a fiducial."""
    robot_state = state_client.get_robot_state()
    current_odom_tform_body = get_odom_tform_body(
        robot_state.kinematic_state.transforms_snapshot).to_proto()

    # Create an empty instance for initial localization since we are asking it to localize
    # based on the nearest fiducial.
    localization = nav_pb2.Localization()
    graph_nav_client.set_localization(initial_guess_localization=localization,
                                            ko_tform_body=current_odom_tform_body)

def graph_localize_fiducial(command_client, graph_nav_client, robot_state_client, robot, timeout_sec = 5):

    """
    Rotates + moves in 30˚ and 0.5m increments to find a fiducial and initialize graph.
    Returns True if fiducial found and graph initialized. Returns False otherwise.
    """

    bdcrc.blocking_stand(command_client, timeout_sec = timeout_sec)

    for i in range(3):
        for j in range(4):
            logging.info("attempt #: ", i*4+j)
            try:
                set_initial_localization_fiducial(robot_state_client, graph_nav_client)
                logging.info("SUCCESS – initialized location to using nearest fiducial")
                return True
            except ResponseError as e:
                logging.warning("could not perform fidicial init, will shift and re-attempt")
                pass
            move_robot_relative(command_client, robot, 0, 0, np.pi/6) #rotates 30˚ CCW
        move_robot_relative(command_client, robot, 0, 0.5, 0) #moves 0.5 meters left
    logging.error("FAILURE – could not find nearby fiducial")
    return False

def graph_del_save_state(graph_nav_client, robot):
    """
    Graph MUST be deleted BEFORE arm camera(s) (color or depth) are used. This includes
    implicit uses of arm camera(s) such as manipulation/door api calls.

    If graph is not deleted the arm camera will return an old image and not the current one.
    This is an ongoing issue being investigated by Boston Dynamics as of 6/27/22. For more
    information see "imageTest.py" in this repo.

    Graph can be reuploaded with upload_graph_and_snapshots and reinitialized by saving
    wp_loc_proto and calling graph_localize_guess with "prev_graph_state = prev_graph_state".

    """

    prev_loc_response = graph_nav_client.get_localization_state()
    prev_graph_loc_proto = prev_loc_response.localization

    prev_waypoint_id = prev_graph_loc_proto.waypoint_id

    prev_waypoint_to_body_proto = prev_graph_loc_proto.waypoint_tform_body
    prev_waypoint_to_body_se3 = bosdyn.client.math_helpers.SE3Pose.from_proto(
        prev_waypoint_to_body_proto)

    frame_tree_snapshot = robot.get_frame_tree_snapshot()
    prev_body_to_vision_se3 = get_a_tform_b(frame_tree_snapshot, frame_a = 'vision', frame_b = 'body')

    vision_to_waypoint_se3 = (prev_waypoint_to_body_se3 * prev_body_to_vision_se3).inverse()
    
    prev_graph_state = {'prev_waypoint_id':prev_waypoint_id, 
                   'vision_to_waypoint_se3': vision_to_waypoint_se3}
    
    graph_nav_client.clear_graph()

    return prev_graph_state

def graph_localize_saved_state(graph_nav_client, robot, prev_graph_state):
    
    prev_waypoint_id = prev_graph_state['prev_waypoint_id']
    vision_to_waypoint_se3 = prev_graph_state['vision_to_waypoint_se3']
    
    fid_init_code_NO_FID = graph_nav_pb2.SetLocalizationRequest.FiducialInit.FIDUCIAL_INIT_NO_FIDUCIAL
    
    frame_tree_snapshot = robot.get_frame_tree_snapshot()
    #get_a_tform_b returns transformation such that a_coords = tform * b_coords
    body_to_vision_se3 = get_a_tform_b(frame_tree_snapshot, frame_a = 'vision', frame_b = 'body')
    body_wrt_waypoint_proto = (body_to_vision_se3 * vision_to_waypoint_se3).to_proto()
    
    init_guess_loc = nav_pb2.Localization(waypoint_id = prev_waypoint_id,
                                      waypoint_tform_body = body_wrt_waypoint_proto)
    
    response = graph_nav_client.set_localization(initial_guess_localization=init_guess_loc,
                                  fiducial_init = fid_init_code_NO_FID)
    return response

# From "graph_nav_command_line.py"
# https://github.com/boston-dynamics/spot-sdk/blob/master/python/examples/graph_nav_command_line/graph_nav_command_line.py
def check_success(graph_nav_client, command_id=-1):
    """Use a navigation command id to get feedback from the robot and sit when command succeeds."""
    if command_id == -1:
        # No command, so we have no status to check.
        return False
    status = graph_nav_client.navigation_feedback(command_id)
    if status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_REACHED_GOAL:
        # Successfully completed the navigation commands!
        return True
    elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_LOST:
        logging.warning("Robot got lost when navigating the route, the robot will now sit down.")
        return True
    elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_STUCK:
        logging.warning("Robot got stuck when navigating the route, the robot will now sit down.")
        return True
    elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_ROBOT_IMPAIRED:
        logging.error("Robot is impaired.")
        return True
    else:
        # Navigation command is not complete yet.
        return False

def wait_until_finished_nav(graph_nav_client, nav_to_cmd_id, timeout_sec = 10):
    """
    Takes in a navigation command id.
    Blocks until command is finished executing as determined by check_success.
    """
    is_finished = False
    prev_timeout_timer_sec = time.time()
    while not is_finished:
        cur_time_sec = time.time()
        if cur_time_sec - prev_timeout_timer_sec > timeout_sec:
            break
        time.sleep(0.2)
        is_finished = check_success(graph_nav_client, nav_to_cmd_id)
