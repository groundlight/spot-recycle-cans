## spot
Groundlight's repository of tools + demos for Spot the robot dog.

### Spot Bootup:
1. Plug a battery into spot.
2. Hold power button near the back of spot until fans turn on.

### Spot Connection through Jetson:
1. Connect to Jetson Backpack via "ssh gluser@10.44.2.255". The is ip configured as static. Ask Arya for the password.
2. Execute "cas". This is a bash alias for "conda activate spotenv".
3. Execute "exps". This is a bash alias for exporting several relevant environment variables.
4. Execute "ping "192.168.50.3" and verify that you get an appropriate response from the robot.
5. Run all code with 192.168.50.3 as the Robot IP.

### Spot Connection over Spot wifi (not preferred):
1. Connect to spot-BD-10110016 wifi. The password can be found inside the battery cavity of spot.
2. Execute "ping "192.168.80.3" and verify that you get an appropriate response from the robot.
3. Run all code with 192.168.80.3 as the Robot IP.

### Spot Shutdown:
1. Close all scripts/notebooks running on spot 
2. Execute "sudo shutdown now" in Jetson terminal
3. Lock out the motors by pressing button next to power button and ensuring the red light is OFF.
4. Wait until green Jetson power LED is off.
4. Power down robot by holding power button for two seconds until fans power down.
5. **REMOVE THE BATTERY!**

