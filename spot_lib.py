#Basic imports
import numpy as np
import matplotlib.pyplot as plt
import io
import os
import sys
from copy import deepcopy
import cv2
import time
from PIL import Image
import logging
from scipy.spatial.transform import Rotation as sciRot
import argparse

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
import bosdyn.client.math_helpers as bdcmh

#Protocol buffer imports
from bosdyn.api import (arm_command_pb2,
                       basic_command_pb2,
                       geometry_pb2,
                       gripper_command_pb2,
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

from google.protobuf import wrappers_pb2

#Other bosdyn imports
from bosdyn.util import seconds_to_duration
from bosdyn.geometry import EulerZXY
from bosdyn.client.exceptions import ResponseError

#Constants
timeout_sec = 10

graph_upload_filepath = "/home/gluser/groundlight/bdspot/spot/office_graph_8_1"


def init_robot(IP):

    """
    Initalizes basic Spot classes
    """

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

def get_batt_info(robot_state_client):

    """
    Gets battery charge as both a percent and runtime remaining.
    """

    robot_state = robot_state_client.get_robot_state()
    battery_state = robot_state.battery_states[0]
    charge_percent = battery_state.charge_percentage.value
    runtime_remaining = battery_state.estimated_runtime.seconds
    return charge_percent, runtime_remaining

def get_lease(robot):

    """
    Gets the robot lease.
    """

    lease_client = robot.ensure_client('lease')
    lease = lease_client.acquire()
    lease_keep_alive = bosdyn.client.lease.LeaseKeepAlive(lease_client)
    return lease_client, lease, lease_keep_alive

def power_on(robot):

    """
    Powers on Spot.
    """

    robot.power_on(timeout_sec=20)
    logging.info(robot.is_powered_on())
    robot.time_sync.wait_for_sync()
    return True

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


###__ARM_MOVEMENT__###

def compute_random_gaze_down():

    """
    Computes a random move_to and look_at to be used with move_to_and_look_at\
    that will aim the gripper at a pseudo random spot downward.
    """

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

def blocking_stow(command_client, timeout_sec = 10):

    cmd = RobotCommandBuilder.arm_stow_command()
    cmd_id = command_client.robot_command(cmd, end_time_secs = time.time() + timeout_sec)
    success = bdcrc.block_until_arm_arrives(command_client, cmd_id,
                                            timeout_sec = time.time() + timeout_sec)
    return success

### GRASP ###

def make_grasp(grasp_request,
               manipulation_api_client,
               command_client,
               verbose = False):
    """
    Makes a grasp with arm given a grasp request proto described here:
    https://dev.bostondynamics.com/protos/bosdyn/api/proto_reference#manipulation-api-proto

    Uploading a graph to graph_nav_client makes arm camera fail. Grasp WILL NOT
    WORK if graph is uploaded. First step of method is to clear graph if it exists.

    Graph must be reuploaded and position reinitialized later. Method must be
    called explicitly with graphExists = False if there is no graph in use.
    """
    cmd_response = manipulation_api_client.manipulation_api_command(
    manipulation_api_request = grasp_request)
    start_time_sec = time.time()
    response = None
    success = False
    while True:
        feedback_request = manipulation_api_pb2.ManipulationApiFeedbackRequest(
            manipulation_cmd_id=cmd_response.manipulation_cmd_id)
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
    """
    Overrides so object can be carried and stowed.
    ONLY use when objects are small enough to be carried.

    """

    carry_override_rq = manipulation_api_pb2.ApiGraspedCarryStateOverride(override_request = 3)
    override_rq = manipulation_api_pb2.ApiGraspOverrideRequest(
        carry_state_override = carry_override_rq)
    override_response = manipulation_api_client.grasp_override_command(
        grasp_override_request = override_rq)

    return override_response

def cap_hand_image(image_client, source):
    """
    Gets either a color or depth image along with the image response from the
    gripper camera depending upon the source provided. Options are:
    source = 'hand_depth_in_hand_color_frame'
    source = 'hand_color_image'

    Depth image is returned as a 1d array of 16 bit ints split into two channels
    such that img[:, :, 0] is the MSB and img[:, :, 1] is the LSB.

    Color image is a simple three channel np array.
    """


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



### GRIPPER ###
def gripper_open(command_client, timeout_sec = 5):
    cmd = RobotCommandBuilder.claw_gripper_open_fraction_command(1.0)
    cmd_id = command_client.robot_command(cmd, end_time_secs = time.time()+timeout_sec)

def gripper_slow_open(command_client):
    """
    Slowly opens the gripper. Useful for minimizing bouncing when placing objects.
    """
    gripper_angles = np.linspace(0, 1, 1000)

    for gripper_angle in gripper_angles:
        cmd = RobotCommandBuilder.claw_gripper_open_fraction_command(gripper_angle)
        cmd_id = command_client.robot_command(cmd, end_time_secs = time.time()+timeout_sec)
        time.sleep(1/gripper_angles.shape[0])

def gripper_close(command_client, timeout_sec = 5):
    cmd = RobotCommandBuilder.claw_gripper_open_fraction_command(0.0)
    cmd_id = command_client.robot_command(cmd, end_time_secs = time.time()+timeout_sec)

def gripper_close_with_torque(command_client, torque = 5.5):

    """
    Closes the gripper with a specific torque.
    Useful for picking up delicate objects or dragging heavy objects.
    Torque can range from 0-16.25 N*m.
    Default close torque is 5.5 N*m
    """
    gripper_traj_point = trajectory_pb2.ScalarTrajectoryPoint(point = 0)
    gripper_traj = trajectory_pb2.ScalarTrajectory(points = [gripper_traj_point])
    claw_gripper_command = gripper_command_pb2. \
        ClawGripperCommand.Request(trajectory = gripper_traj,
                                   maximum_torque = wrappers_pb2.DoubleValue(value = torque))
    gripper_command = gripper_command_pb2. \
        GripperCommand.Request(claw_gripper_command = claw_gripper_command)
    synchronized_command = synchronized_command_pb2.SynchronizedCommand.Request(
        gripper_command = gripper_command)
    gripper_close_command = robot_command_pb2.RobotCommand(synchronized_command = synchronized_command)
    gripper_cmd_id = command_client.robot_command(gripper_close_command,
                                              end_time_secs = time.time()+timeout_sec)

def get_gripper_torque(robot_state_client):
    """
    Returns the current torque being applied by the gripper.
    """
    robot_state = robot_state_client.get_robot_state()
    return robot_state.kinematic_state.joint_states[-1].load

def init_robot(IP):

    """
    Initalizes basic Spot classes
    """

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

def get_batt_info(robot_state_client):

    """
    Gets battery charge as both a percent and runtime remaining.
    """

    robot_state = robot_state_client.get_robot_state()
    battery_state = robot_state.battery_states[0]
    charge_percent = battery_state.charge_percentage.value
    runtime_remaining = battery_state.estimated_runtime.seconds
    return charge_percent, runtime_remaining

def get_lease(robot):

    """
    Gets the robot lease.
    """

    lease_client = robot.ensure_client('lease')
    lease = lease_client.acquire()
    lease_keep_alive = bosdyn.client.lease.LeaseKeepAlive(lease_client)
    return lease_client, lease, lease_keep_alive

def power_on(robot):

    """
    Powers on Spot.
    """

    robot.power_on(timeout_sec=20)
    logging.info(robot.is_powered_on())
    robot.time_sync.wait_for_sync()
    return True

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
        logging.info("Upload complete! The robot is currently not localized to the map; please localize")
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

def real_frame_add(frame_tree_snapshot,
                   parent_tform_child_proto,
                   parent_frame_name,
                   child_frame_name):

    fts_dict = dict(frame_tree_snapshot.child_to_parent_edge_map)
    response = bosdyn.client.frame_helpers.add_edge_to_tree(frame_tree_snapshot = fts_dict,
                                             parent_tform_child = parent_tform_child_proto,
                                             parent_frame_name = parent_frame_name,
                                             child_frame_name = child_frame_name)
    fts_with_new_frame = geometry_pb2.FrameTreeSnapshot(child_to_parent_edge_map = fts_dict)

    return fts_with_new_frame

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

    prev_frame_tree_snapshot = robot.get_frame_tree_snapshot()
    prev_loc_response = graph_nav_client.get_localization_state()

    prev_waypoint_tform_body_proto = prev_loc_response.localization.waypoint_tform_body
    prev_waypoint_tform_body_se3 = bosdyn.client.math_helpers.SE3Pose.from_proto(                                                                           prev_waypoint_tform_body_proto)
    prev_body_tform_waypoint_proto = prev_waypoint_tform_body_se3.inverse().to_proto()

    prev_waypoint_id = prev_loc_response.localization.waypoint_id

    prev_fts_with_waypoint = real_frame_add(prev_frame_tree_snapshot,
                                        prev_body_tform_waypoint_proto,
                                        'body',
                                        'waypoint')

    vision_tform_waypoint_se3 = get_a_tform_b(prev_fts_with_waypoint, 'vision', 'waypoint')

    prev_graph_state = {'prev_waypoint_id':prev_waypoint_id,
                        'vision_tform_waypoint_se3':vision_tform_waypoint_se3}

    graph_nav_client.clear_graph()

    return prev_graph_state

def graph_localize_saved_state(graph_nav_client, robot, prev_graph_state):

    prev_waypoint_id = prev_graph_state['prev_waypoint_id']
    vision_tform_waypoint_se3 = prev_graph_state['vision_tform_waypoint_se3']

    frame_tree_snapshot = robot.get_frame_tree_snapshot()
    fts_with_waypoint = real_frame_add(frame_tree_snapshot,
                                       vision_tform_waypoint_se3.to_proto(),
                                       'vision',
                                       'waypoint')
    waypoint_tform_body_proto = get_a_tform_b(fts_with_waypoint, 'waypoint', 'body').to_proto()

    fid_init_code_NO_FID = graph_nav_pb2.SetLocalizationRequest.FiducialInit.FIDUCIAL_INIT_NO_FIDUCIAL
    init_guess_loc = nav_pb2.Localization(waypoint_id = prev_waypoint_id,
                                          waypoint_tform_body = waypoint_tform_body_proto)
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



###__SPOT_GROUNDLIGHT_FUNCTIONS__###

import numpy as np
from groundlight import Groundlight
import glob
import cv2
import matplotlib.pyplot as plt
import matplotlib
import os
import PIL
from PIL import Image
from io import BytesIO
import logging


def gl_init():
    GROUNDLIGHT_API_TOKEN = os.environ.get("GROUNDLIGHT_API_TOKEN")
    gl = Groundlight(api_token=GROUNDLIGHT_API_TOKEN)
    return gl

def sweep_localize(gl, det_id, img, det_conf = 0.65, verbose = False):

    """
    Recursively applies a binary detector to an image to localize an object
    within the image.Detector yes/no question should be phrased as:
    "Is entire object within image?"

    Running in verbose mode will display plots of the recursive sweep.
    Returns a tree of tuples representing the path taken to find the object.
    """

    img_dims = np.array([img.shape[1],img.shape[0]])

    #Recursive Base Case, i.e. too many splits
    if img_dims[0] < 20 or img_dims[1] < 20:
        return None

    kernel_dims = (img_dims*2/3).astype(int)
    sweep_dims = (img_dims/4).astype(int)
    col_sweeps, row_sweeps = np.round(img_dims/sweep_dims).astype(int) - 1

    if verbose:
        fig, ax = plt.subplots(row_sweeps, col_sweeps, figsize = (16,12))

    slice_confidences = np.zeros((row_sweeps, col_sweeps))
    slice_dict = {}

    for col in range(col_sweeps):
        for row in range(row_sweeps):
            px_start = sweep_dims * np.array([col, row])
            pxEnd = px_start + kernel_dims

            slice_dict[(row, col)] = (px_start, pxEnd)
            slice_img_mat = img[px_start[1]:pxEnd[1],px_start[0]:pxEnd[0],:]

            image_query = mat_thru_det(gl, det_id, slice_img_mat)
            slice_label = image_query.result.label
            slice_conf = image_query.result.confidence

            if slice_label == 'PASS' and slice_conf > det_conf:
                if verbose:
                    ax[row, col].set_title('obj found, conf: '+ str(np.round(slice_conf,2)))
                    ax[row, col].imshow(slice_img_mat)
                slice_confidences[row, col] = slice_conf
            else:
                if verbose:
                    ax[row, col].imshow(cv2.cvtColor(slice_img_mat,cv2.COLOR_BGR2GRAY), cmap = 'ocean')
                slice_confidences[row, col] = 0

    max_1d_ind = np.argmax(slice_confidences)
    max_2d_ind = np.unravel_index(max_1d_ind, slice_confidences.shape)

    best_conf = slice_confidences[max_2d_ind]
    if best_conf < det_conf:
        return None

    if verbose:
        ax[max_2d_ind[0], max_2d_ind[1]].set_title(
            'OBJ FOUND, BEST CONF: ' + str(np.round(best_conf,2)), fontsize = 15, color = 'lime')
        plt.show()

    best_px_start, best_px_end = slice_dict[max_2d_ind]
    best_slice_img_mat = img[best_px_start[1]:best_px_end[1],best_px_start[0]:best_px_end[0],:]

    return (best_px_start,
            np.minimum(best_px_end, img_dims),
            sweep_localize(gl, det_id, best_slice_img_mat, det_conf = det_conf, verbose = verbose))

def mat_thru_det(gl, det_id, mat):

    """
    Passes a three channel image represented by a numpy array through a
    detector.
    """

    img_PIL = Image.fromarray(cv2.cvtColor(mat, cv2.COLOR_BGR2RGB))
    byte_io = BytesIO()

    img_PIL.save(byte_io, 'jpeg')
    jpg_buffer = byte_io.getvalue()
    byte_io.close()

    image_query = gl.submit_image_query(detector_id=det_id, image = jpg_buffer)

    return image_query

def assemble_px_tree(tree):
    """
    Takes the pixel tree produced by sweep localize and gets the location of
    the found object.
    """
    return assemble_px_tree_helper(np.array([0,0]), tree)

def assemble_px_tree_helper(oldPx, tree):
    newPx = oldPx + tree[0]
    if(tree[2]) is None:
        newPx += ((tree[1] - tree[0])/2).astype(int)
        return newPx
    return assemble_px_tree_helper(newPx, tree[2])


def plot_tree_on_image(ax, tree, img):

    """
    Creates a plot showing the recursive search progression for an object within
    an image.
    """

    ax.imshow(img)
    plot_tree_on_image_helper(ax, tree, np.array([0,0]))
    return ax

def plot_tree_on_image_helper(ax, tree, prevPx):
    cmap = matplotlib.cm.get_cmap('rainbow')

    if tree[2] is None:
        center = prevPx + ((tree[0] + tree[1])/2).astype(int)
        ax.scatter([center[0]], [center[1]], marker = 'x', s = 60, color = 'black')
        return

    color = cmap(np.random.rand())
    ax.scatter([tree[0][0] + prevPx[0]],[tree[0][1] + prevPx[1]],marker = 'o', s = 30, color = color)
    ax.scatter([tree[1][0] + prevPx[0]],[tree[1][1] + prevPx[1]],marker = 'o', s = 30, color = color)
    prevPx += tree[0]
    plot_tree_on_image_helper(ax, tree[2], prevPx)


###__OBJECT_DATA_COLLECT__###


#### CONSTANTS ####
look_at_locs = np.array([[1.1, 0, -0.5],[0, 1.1, -0.5]])

obj_pos_body = geometry_pb2.Vec3(x = 0.8, y = 0, z = -0.55)

arm_pos_bounds  = {'x': {'lower':  0.6, 'upper': 0.9},
                   'y': {'lower': -0.4, 'upper': 0.4},
                   'z': {'lower': -0.1, 'upper': 0.8}}

drop_rots = np.array([
    [1,0,0],
    [-1,0,0],
    [1,0,1],
    [-1,0,1],
    [1,0,-1],
    [-1,0,-1],
    [0,1,0],
    [0,1,1],
    [0,1,2],
    [0,1,3]
])

#clear_waypoints_shop = np.array([55, 59, 62])
clear_waypoints_office = np.array([3, 26, 4, 27, 28])

clear_waypoints = clear_waypoints_office

obj_search_cam_coords = [{'moveTo':np.array([0.6, 0.0, 0.5]),
                          'lookAt':np.array([1.0, 0.0, -0.5])}]

def random_rot_drop(command_client, manipulation_api_client):
    drop_rot_ind = np.random.randint(drop_rots.shape[0])
    drop_rot = drop_rots[drop_rot_ind, :]

    rotObj = sciRot.from_euler('xyz',np.pi/2*drop_rot)
    rot_mtx = rotObj.as_matrix()
    rot_quat = bosdyn.client.math_helpers.Quat.from_matrix(rot_mtx)
    drop_pose_body_frame_proto = geometry_pb2.SE3Pose(
        position = geometry_pb2.Vec3(x = 0.8, y = 0, z = -0.3),
        rotation = rot_quat.to_proto())
    drop_pose_body_frame_se3 = bosdyn.client.math_helpers.SE3Pose.from_proto(drop_pose_body_frame_proto)
    arm_cmd = RobotCommandBuilder.arm_pose_command(x = drop_pose_body_frame_se3.x,
                                               y = drop_pose_body_frame_se3.y,
                                               z = drop_pose_body_frame_se3.z,
                                               qw = drop_pose_body_frame_se3.rotation.w,
                                               qx = drop_pose_body_frame_se3.rotation.x,
                                               qy = drop_pose_body_frame_se3.rotation.y,
                                               qz = drop_pose_body_frame_se3.rotation.z,
                                               frame_name = 'body')

    body_cmd = RobotCommandBuilder.synchro_stand_command(body_height = -0.3)
    cmd = RobotCommandBuilder.build_synchro_command(arm_cmd, body_cmd)
    cmd_id = command_client.robot_command(cmd, end_time_secs = time.time()+timeout_sec)
    bdcrc.block_until_arm_arrives(command_client, cmd_id, timeout_sec = time.time() + 3.0)
    time.sleep(0.5)

    gripper_slow_open(command_client)
    time.sleep(0.5)

    body_cmd = RobotCommandBuilder.synchro_stand_command(body_height = 0)
    cmd_id = command_client.robot_command(body_cmd, end_time_secs = time.time()+timeout_sec)
    time.sleep(1)

    success = blocking_stow(command_client, timeout_sec = 3)
    override_response = stowable_override(manipulation_api_client)
    time.sleep(2)

def rand_imgs_thru_det(command_client, image_client, gl, det_id):
    for j in range(len(look_at_locs)):
        look_at = look_at_locs[j]
        for i in range(3):
            #Move Arm to random position
            x_pos_rand = np.random.uniform(arm_pos_bounds['x']['lower'],arm_pos_bounds['x']['upper'])
            y_pos_rand = np.random.uniform(arm_pos_bounds['y']['lower'],arm_pos_bounds['y']['upper'])
            z_pos_rand = np.random.uniform(arm_pos_bounds['z']['lower'],arm_pos_bounds['z']['upper'])

            move_to = np.array([x_pos_rand, y_pos_rand, z_pos_rand])
            move_to_and_look_at(command_client, move_to, look_at, blocking = True)

            time.sleep(0.4)

            hand_color_img, img_response = cap_hand_image(image_client, source = 'hand_color_image')
            image_query = mat_thru_det(gl, det_id, hand_color_img)

def multi_attempt_grasp(*, manipulation_api_client, command_client, graph_nav_client, obj_pos_vision):
    for i in range(5):
        print(i)

        #grasp_params = manipulation_api_pb2.GraspParams(grasp_palm_to_fingertip = 1)
        grasp = manipulation_api_pb2.PickObject(frame_name = 'vision',
                                                object_rt_frame = obj_pos_vision)

        grasp_request = manipulation_api_pb2.ManipulationApiRequest(pick_object=grasp)
        response, success = make_grasp(grasp_request, manipulation_api_client,
                                       command_client, verbose = False)

        curState = robot_state_client.get_robot_state()
        wrist_pos_rads = np.abs(curState.kinematic_state.joint_states[-1].position.value)

        if wrist_pos_rads < np.pi/36 or not success:
#            print(response, success)
            blocking_stow(command_client, timeout_sec = 15)
            time.sleep(3)
            continue
        else:
            return True


            print('GRIPPER IS CLOSED. FAILED TO GRAB CUBE')

    logging.warning('FAILED TO GRAB CUBE')
    return False

def smart_multi_attempt_grasp(*,
                              manipulation_api_client,
                              command_client,
                              image_client,
                              robot_state_client,
                              robot,
                              gl,
                              obj_det_id,
                              num_attempts = 5,
                              verbose = False):

    frame_tree_snapshot = robot.get_frame_tree_snapshot()
    vision_tform_body = get_vision_tform_body(frame_tree_snapshot)
    flat_robot_pose_vision_frame_se2 = bdcmh.SE2Pose.flatten(vision_tform_body)
    flat_robot_pose_vision_frame_se2.to_proto()

    grasp_responses = []

    for i in range(num_attempts):
        logging.info('attempt number: {}'.format(i))

        move_noise = np.random.rand(3)*0.4 -0.2
        look_noise = np.random.rand(3)*0.4 -0.2
        move_to_and_look_at(command_client,
                obj_search_cam_coords[0]['moveTo'] + move_noise,
                obj_search_cam_coords[0]['lookAt'] + look_noise)

        time.sleep(0.5)
        hand_color_img, image_response = cap_hand_image(image_client, 'hand_color_image')
        tree = sweep_localize(gl, obj_det_id, hand_color_img, verbose = verbose)
        if tree is None:
            logging.warning('Could not find cube, will attempt to localize again')
            continue

        pxLoc = assemble_px_tree(tree)
        pick_vec = geometry_pb2.Vec2(x=pxLoc[0], y=pxLoc[1])
        grasp = manipulation_api_pb2.PickObjectInImage(
            pixel_xy=pick_vec, transforms_snapshot_for_camera=image_response.shot.transforms_snapshot,
            frame_name_image_sensor=image_response.shot.frame_name_image_sensor,
            camera_model=image_response.source.pinhole)
        grasp_request = manipulation_api_pb2.ManipulationApiRequest(pick_object_in_image=grasp)
        response, success = make_grasp(grasp_request, manipulation_api_client,
                                       command_client,
                                       verbose = verbose)
        grasp_responses.append(response)
        curState = robot_state_client.get_robot_state()
        wrist_pos_rads = np.abs(curState.kinematic_state.joint_states[-1].position.value)

        blocking_stow(command_client, timeout_sec = timeout_sec)

        if wrist_pos_rads < np.pi/36 or not success:
#            print(response, success)
            gripper_open(command_client)

            cmd = RobotCommandBuilder.synchro_se2_trajectory_command(
                goal_se2 = flat_robot_pose_vision_frame_se2.to_proto(),
                frame_name = 'vision'
            )
            cmd_id = command_client.robot_command(cmd, end_time_secs = time.time()+timeout_sec)
            bdcrc.block_for_trajectory_cmd(command_client, cmd_id, timeout_sec = timeout_sec)

            continue
        else:
            return True, grasp_responses


            print('GRIPPER IS CLOSED. FAILED TO GRAB CUBE')

    logging.warning('FAILED TO GRAB CUBE')
    return False, grasp_responses

def drop_cap_grasp(*, manipulation_api_client, command_client, image_client, robot, robot_state_client, gl, det_id):
    random_rot_drop(command_client, manipulation_api_client)

    #get_a_tform_b returns transformation such that a_coords = tform * b_coords
    frame_tree_snapshot = robot.get_frame_tree_snapshot()
    body_to_vision = bosdyn.client.frame_helpers.get_a_tform_b(frame_tree_snapshot, frame_a = 'vision', frame_b ='body')
    obj_pos_body_se3_proto = geometry_pb2.SE3Pose(
        position = obj_pos_body,
        rotation = bosdyn.client.math_helpers.Quat(w = 0,
                                               x = 0,
                                               y = -(1/2)**0.5,
                                               z = (1/2)**0.5).to_proto())
    obj_pos_body_se3 = bosdyn.client.math_helpers.SE3Pose.from_proto(obj_pos_body_se3_proto)
    obj_pos_vision_se3 = body_to_vision * obj_pos_body_se3

    move_robot_relative(command_client, robot, -0.3, 0, 0)

#     rand_imgs_thru_det(command_client, image_client, gl, det_id)

    blocking_stow(command_client)

    smart_multi_attempt_grasp(manipulation_api_client = manipulation_api_client,
                      command_client = command_client,
                      image_client = image_client,
                      robot_state_client = robot_state_client,
                      robot = robot,
                      obj_det_id = det_id,
                      gl = gl,
                      num_attempts = 5,
                      verbose = False)

    blocking_stow(command_client)

def wait_until_has_object(command_client, robot_state_client):
    """
    Robot will repeatedly open and close gripper jaws until it grabs an object
    """
    while True:
        gripper_close(command_client)
        time.sleep(0.5)
        curState = robot_state_client.get_robot_state()
        wrist_pos_rads = np.abs(curState.kinematic_state.joint_states[-1].position.value)
        if wrist_pos_rads > 2/180*np.pi:
            break
        gripper_open(command_client)
        time.sleep(3)
        logging.warning("OBJECT NOT IN POSSESION, PLEASE GIVE OBJECT TO ROBOT")
