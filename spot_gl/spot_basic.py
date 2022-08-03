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
