### MOVEMENT ###

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
