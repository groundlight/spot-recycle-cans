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
