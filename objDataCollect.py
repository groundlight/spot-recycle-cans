from spot_lib import *

parser = argparse.ArgumentParser()
parser.add_argument('--hostname', help = 'ROBOT IP ADDR, 192.168.50.3 from Jetson and 192.168.80.3 from mac over spot wifi', required = True)
parser.add_argument('--GROUNDLIGHT_API_TOKEN', help = 'Your groundlight api token', required = True)
parser.add_argument('--DETECTOR_ID', help = 'ID for a binary detector of object presence', required = True)
parser.add_argument('--GRAPH_PATH', help = 'Path to top level graph directory on Jetson', required = True)

options = parser.parse_args(sys.argv[1:])


ROBOT_IP = options.hostname
obj_det_id = options.DETECTOR_ID
graph_upload_filepath = options.GRAPH_PATH
GROUNDLIGHT_API_TOKEN = options.GROUNDLIGHT_API_TOKEN


sdk, robot, id_client, \
robot_state_client, command_client, \
image_client, graph_nav_client, \
world_object_client, manipulation_api_client = init_robot(ROBOT_IP)

lease_client, lease, lease_keep_alive = get_lease(robot)

#gl = Groundlight(api_token=os.environ.get("GROUNDLIGHT_API_TOKEN"))
gl = Groundlight(api_token=GROUNDLIGHT_API_TOKEN)

batt_state = get_batt_info(robot_state_client)

print('battery %:', batt_state[0], 'runtime remaining (sec):', batt_state[1])

success = power_on(robot)
bdcrc.blocking_selfright(command_client, timeout_sec = 20)
bdcrc.blocking_sit(command_client, timeout_sec = 20)
bdcrc.blocking_stand(command_client, timeout_sec = timeout_sec)

response = upload_graph_and_snapshots(graph_nav_client, graph_upload_filepath)
success = graph_localize_fiducial(command_client, graph_nav_client, robot_state_client, robot)
waypoint_list = create_waypoint_list(graph_nav_client)
prev_graph_state = graph_del_save_state(graph_nav_client, robot)

wait_until_has_object(command_client, robot_state_client)

#for i in range(len(clear_waypoints)):
for i in range(len(waypoint_list)):
    upload_graph_and_snapshots(graph_nav_client, graph_upload_filepath)
    response = graph_localize_saved_state(graph_nav_client, robot, prev_graph_state)


    wp = waypoint_list[i]#waypoint_list[clear_waypoints[i]]
    nav_to_cmd_id = graph_nav_client.navigate_to(wp, cmd_duration = 20)
    wait_until_finished_nav(graph_nav_client, nav_to_cmd_id)

    prev_graph_state = graph_del_save_state(graph_nav_client, robot)

    drop_cap_grasp(manipulation_api_client = manipulation_api_client,
           command_client = command_client,
           image_client = image_client,
           robot_state_client = robot_state_client,
           robot = robot,
           gl = gl,
           det_id = obj_det_id)

    wait_until_has_object(command_client, robot_state_client)

blocking_stow(command_client)
upload_graph_and_snapshots(graph_nav_client, graph_upload_filepath)
success = graph_localize_fiducial(command_client, graph_nav_client, robot_state_client, robot)
waypoint_list = create_waypoint_list(graph_nav_client)

nav_to_cmd_id = graph_nav_client.navigate_to(waypoint_list[0], cmd_duration = 15)
wait_until_finished_nav(graph_nav_client, nav_to_cmd_id)

bdcrc.blocking_sit(command_client)

lease_client.return_lease(lease)
