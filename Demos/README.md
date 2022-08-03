## How to run a demo
1. Navigate to the spot folder on the Jetson Backpack via "cd ~/groundlight/bdspot/spot/
2. Type "python3 myEstop.py -i [YOUR_COMPUTER_IP] 192.168.50.3" to run the estop script for spot. For example "python3 myEstop.py -i 10.44.2.227 192.168.50.3".
3. Drag the robot into position next to a fidicual so it can initialize the graph upon standing. Ensure that the motors are locked and the red light on the button at the back of the robot is OFF.
3. Unlock motors by pressing button next to power button and ensuring the red light is ON.
4. Create an SSH tunnel to access the jupyter notebook from your computer with: "ssh gluser@10.44.2.255 -N -f -L 8888:localhost:8888".
5. Open a jupyter notebook on the Jetson by typing "jn". This is a bash alias for "jupyter notebook".
6. In your browser go to "localhost:8888" and open the demo notebook.
7. Run the cells in order. DO NOT run the setup + lease cells more than once.
8. When finished, close the notebook, hit CTRL^C on the estop script and the robot will slowly sit down.
9. Shutdown spot.
