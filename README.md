# Spot
Groundlight's repository of tools + demos for Spot the robot dog.

## Code Organization:
* `spot_setup`: View this first and get setup to run code on spot through the Jetson Backpack or locally.
* `spot_tools`: Scripts to drive spot with the keyboard and map a space so Spot can autonomously navigate.
* `spot_gl`: Library of useful functions developed for spot. Includes initialization, movement, object detection, and navigation of spaces.
* `spot_demos`: Applications of Spot. Scripts here draw upon spot_gl as well as maps created with spot_tools.

## Spot Basics
#### Spot Bootup:
1. Plug a battery into spot.
2. Hold power button near the back of spot until fans turn on.

#### Spot Connection through Jetson:
1. Connect to Jetson Backpack via "ssh gluser@10.44.2.255". The is ip configured as static. Access password from 1Password.
2. Execute "cas". This is a bash alias for "conda activate spotenv".
3. Execute "exps". This is a bash alias for exporting several relevant environment variables.
4. Execute "ping "192.168.50.3" and verify that you get an appropriate response from the robot.
5. Run all code with 192.168.50.3 as the Robot IP.

#### Spot Connection over Spot wifi (not preferred):
1. Connect to spot-BD-10110016 wifi. The password can be found inside the battery cavity of spot.
2. Execute "ping "192.168.80.3" and verify that you get an appropriate response from the robot.
3. Run all code with 192.168.80.3 as the Robot IP.
4. Note that this is a different address from the one used by the Jetson to communicate with the robot.

### Autonomous Grasping Tool:
1. Connect to spot through Jetson.
2. Call ```python3 objDataCollect.py --hostname [ROBOT_IP] --GROUNDLIGHT_API_TOKEN [YOUR_GROUNDLIGHT_API_TOKEN] --DETECTOR_ID [OBJECT_DETECTOR_ID] --GRAPH_PATH [PATH TO GRAPH OF SPACE]```
3. Example (will run on Jetson using a tape roll detector): python3 objDataCollect.py --hostname 192.168.50.3 --GROUNDLIGHT_API_TOKEN $GROUNDLIGHT_API_TOKEN --DETECTOR_ID det_2CH9fxrXociWrb05tDmPblqoedW --GRAPH_PATH ~/groundlight/bdspot/spot/graphs/office_graph_8_1/
4. Robot will walk through waypoints present in graph, and repeatedly drop an object, localize it with the provided detector, and attempt to grasp it. It will re-attempt a grasp 5 times before asking for a human to return the object to the gripper via a console warning.
5. Object detector id's are provided here incase testing on different objects is desired:
    - Metal Wrench: det_2CoeZWa0MKfNIDGxneBmVtAvcVp
    - Blue Paper Towel Roll: det_2CHAr7ANClu8stvhbvAnzFiTK9m
    - Aluminum Can: det_2CH6dRdu8XlXiSCQmfYudZcs3ev
    - Rubik's Cube: det_2CE1cYK7CzWVixNEj0SQlptb9ZF
    - Whiteboard Eraser: det_2BMUhuS29G9JTZcCw1NrMleuqSy
    

#### Spot Shutdown:
1. Close all scripts/notebooks running on spot 
2. Execute "sudo shutdown now" in Jetson terminal
3. Lock out the motors by pressing button next to power button and ensuring the red light is OFF.
4. Wait until green Jetson power LED is off.
4. Power down robot by holding power button for two seconds until fans power down.
5. **REMOVE THE BATTERY!**