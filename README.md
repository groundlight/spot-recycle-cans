# spot
Groundlight's repository of tools + demos for Spot the robot dog.

## How to boot up Spot
1. Plug a battery into spot
2. Hold power button near the back of spot until fans turn on
3. Connect to Jetson Backpack via "ssh gluser@10.44.2.255". The is ip configured as static. Ask Arya for the password.
4. Type "cas". This is a bash alias for "conda activate spotenv".
5. Type "exps". This is a bash alias for exporting several relevant environment variables.
6. Type "ping "192.168.50.3" and verify that you get an appropriate response from the robot

## How to shut down spot
1. Close all scripts running on spot 
2. Type "sudo shutdown now" in Jetson terminal
3. Lock out the motors by pressing button next to power button and ensuring red light is OFF.
4. Power down robot by holding power button for two seconds until fans power down.
5. **REMOVE THE BATTERY**
