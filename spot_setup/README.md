## spot setup

### Jetson Setup: 
In case the Jetson in Spot's backpack must be swapped follow these instructions:

**NOTE**: Issues replicating environments across Jetsons. Likely requires further investigation.

1. Connect to Jetson via USB and follow new Jetson setup guide here: https://github.com/positronix-ai/jetsons/blob/main/rosie/README.md
2. Install miniforge onto the Jetson here: https://github.com/conda-forge/miniforge
3. Clone this repo to the Jetson and navigate to the setup folder.
4. Create the conda environment via ```conda env create -f spotJetsonEnv.yml```
5. Physically swap Jetsons and ensure that ethernet and power jack are properly connected to new Jetson.
6. Connect to Jetson over USB again, power on Spot, and wait for Spot to finish powering on.
7. Use nmtui to configure static IP addresses of Jetson to:
    |Network|IP|
    |-------|------------|
    |eth0   |10.44.2.255 |
    |wlan0  |192.168.50.5|
    
    
### Mac Setup:
If you would like to run Spot code locally without using the Jetson follow these instructions:

**NOTE**: Issues replicating environments across Macs. Likely requires further investigation.

1. Install miniforge locally here: https://github.com/conda-forge/miniforge
2. Clone this repo to the Jetson and navigate to the setup folder.
3. Create the conda environment via ```conda env create -f spotMacEnv.yml```