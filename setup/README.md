## spot setup

### Jetson Setup: 
In case the Jetson in Spot's backpack must be swapped follow these instructions:

1. Follow new Jetson setup guide here: https://github.com/positronix-ai/jetsons/blob/main/rosie/README.md
2. Install miniforge onto the Jetson here: https://github.com/conda-forge/miniforge
3. Clone this repo to the Jetson and navigate to the setup folder.
4. Create the conda environment via ```conda env create -f spotJetsonEnv.yml```
5. Open the .bash_aliases from setup in a text editor and change the Groundlight API token on line 4 in the "exps" alias to your Groundlight API token.
6. Copy .bash_aliases and .bashrc from setup into ~/ directory via ```cp .bash_aliases .bashrc ~/``` (or copy aliases into existing .bash_aliases and .bashrc).
7. Swap Jetsons and ensure that ethernet and power jack are properly connected to new Jetson.

### Mac Setup:
If you would like to run Spot code locally without using the Jetson follow these instructions:

1. Install miniforge locally here: https://github.com/conda-forge/miniforge
2. Clone this repo to the Jetson and navigate to the setup folder.
3. Create the conda environment via ```conda env create -f spotMacEnv.yml```
4. Open the .bash_aliases from setup in a text editor and change the Groundlight API token on line 4 in the "exps" alias to your Groundlight API token.
5. Copy .bash_aliases and .bashrc from setup into ~/ directory via ```cp .bash_aliases .bashrc ~/``` (or copy aliases into existing .bash_aliases and .bashrc or .bash_profile or .zshrc depending upon your terminal preferences).