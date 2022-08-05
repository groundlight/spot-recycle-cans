## spot_demos

### Run a demo
1. Navigate to the spot folder on the Jetson Backpack via "cd ~/groundlight/bdspot/spot/
2. Run the spot_estop script from the spot_tools folder in a separate terminal.
3. Ensure that the motors are locked and the red light on the button at the back of the robot is OFF.
4. Drag the robot into position next to a fidicual so it can initialize the graph upon standing. 
5. Unlock motors by pressing button next to power button and ensuring the red light is ON.
6. Create an SSH tunnel to access the jupyter notebook from your computer with: "ssh gluser@10.44.2.255 -N -f -L 8888:localhost:8888".
7. Open a jupyter notebook on the Jetson by typing "jn". This is a bash alias for "jupyter notebook".
8. In your browser go to "localhost:8888" and open the demo notebook.
9. Run the cells in order. DO NOT run the setup + lease cells more than once.
10. When finished, close the notebook, hit CTRL^C on the estop script and the robot will slowly sit down.
11. Shutdown spot.
