## spot tools
Key tools available to Spot users.
Files copied from https://github.com/boston-dynamics/spot-sdk/tree/master/python/examples

To use the following tools create a new terminal and follow the instructions for "Spot Jetson Connection" in the top-level directory README then navigate to the tools folder via ```cd ~/groundlight/bdspot/spot/tools/```

### spot_estop:
An estop allows the Spot's user to instantly cut power to Spot in the event of a failure. Instructions to set up an estop are as follows.

1. Run the estop script via ```python3 spot_estop.py. -i [YOUR_COMPUTER_IP] 192.168.50.3```
2. Arm Spot by physically pressing the circular button on the back of the robot to the right of the power button. A red light around this button will be visible when the robot is armed.
3. Hit 'r' to release the estop. The terminal will display a green 'NOT ESTOPPED'.
4. **Hit SPACE to activate the estop.** This will immediately cut power to the robot's motors and it will collapse.
5. Run a script or notebook of interest in a separate terminal while the spot_estop script is running.

### spot_wasd:
1. Run the wasd script via ```python3 spot_wasd.py. 192.168.50.3```
2. Deactivate the estop with 'SPACE'.
3. Power on the robot with "SHIFT+p".
4. Self-right Spot with 'r'.
5. Stand Spot with 'f'. Spot must be upright in order to stand.
6. Control Spot with w, a, s, d, q, e as desired.
7. Once finished, sit Spot with 'v'.
8. Optionally flip Spot for a battery change with 'b'.
9. Power off Spot with 'SHIFT P'.
10. Hit 'l' to return the robot lease so another script can control the robot.
11. At any point **Hit 'SPACE' to activate the estop.** This will immediately cut power to the robot's motors and it will collapse.
12. Hit 'TAB' or CTRL^C to close the spot_wasd script.
13. Note that this script has a built in estop and does not require spot_estop.py to be run in parallel.

### Mapping a space with Spot:
1. **Place Fiducials:** 
    - Place 3-12 fiducials within 4m of Spot's projected path through the space of interest and less than 1m above the ground. 
    - These fiducials must be 146mm square AprilTags of type 36h11. 
    - Pre-sized, ready to print fiducials can be found in the files section in the top right of this page: https://support.bostondynamics.com/s/article/About-Fiducials
    - Laminating fiducials before placement is highly reccomended.
    
    
2. **Initalize robot position:** 
    - Follow the instructions above in the "spot_wasd" section to start up keyboard control of spot and drive the robot to a desired starting position where it can see a fiducial. 
    - Leave the robot in a standing position.
    
    
3. **Initialize Map Recording CLI:** 
    - Open a new terminal, connect to Spot's Jetson and navigate to the tools folder. 
    - Run the Map Recording CLI via ```python3 recording_command_line.py 192.168.50.3```
    - Clear the map by typing ```0``` + ```ENTER```
    - Start recording by typing ```1``` + ```ENTER```
    - Verify that recording started by typing ```3``` + ```ENTER```
    s
    
4. **Navigate Robot through space:**
    - Use keyboard commands to navigate the robot through the space of interest.
    - Driving Spot through space along two or more unique intersecting paths will yield the best results and allow for the most loop closures once the map is created.
    - Once the space has been sufficiently explored, sit the robot down and power it off and close the spot_wasd script.
    
    
5. **Cleanup + Download Map:**
    - Verify that recording occured by typing ```6``` + ```ENTER```
    - Expected output of the above command is a list of waypoints.
    - Close the path by typing ```8``` + ```ENTER```
    - Perform loop-closure to improve the accuracy of the map by typing ```9``` + ```ENTER``` followed by ```0``` + ```ENTER```
    - Optimize map by typing ```a``` + ```ENTER```
    - Stop recording by typing ```2``` + ```ENTER```
    - Download the map by typing ```5``` + ```ENTER```
    - Move/rename the downloaded graph folder in ~/groundlight/bdspot/spot/tools/ to the desired location
    
    
6. **Viewing the Map:** 
    - The map cannot be viewed on the Jetson so copy it to your computer via:  
    ```scp -r gluser@10.44.2.255:[JETSON_PATH_TO_MAP_FOLDER] [LOCAL_PATH_TO_PLACE_MAP]```
    - Follow the instructions in the setup folder README to clone the repo and create the conda environment locally.
    - Activate the conda environment with ```conda activate spotenv```
    - Navigate to the spot/tools/ folder locally.
    - View the map by running ```python3 view_map.py [LOCAL_PATH_TO_MAP_FOLDER]```
    - Verify that fiducial placements and point clouds are reasonable.