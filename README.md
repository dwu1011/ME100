# ME100
ENGR 100-400 Fall 2023 Final Project Code 


## Files
We thought our program would be easier to debug and comprehend if we split it up into sections, each charged with their own tasks. To divy them up, we used files.
- ### Maze_Control.py (main file)
  - Contains all the high level logic to operate the drone using the resources created in the other files
- ### ME100Lib.py
  - Our personal utility file that contains all abstract functions and classes
  - Includes: PID, Low-Pass Filters, Helper Functions (i.e. clamp and get_custom_lidar)
- ### Constants.py
  - Contains all constants to our PID and Low-Pass Filter tunings
- ### E100_functions.pyc
  - Compiled library that contains bindings to the AirSim server-client setup
  - Used in almost all of our code
- ### Extra Resources Directory
  - Contains the map we used for our testing
  - Contains the settings.json file that our program uses (includes the custom top-side LiDAR)
- ### Old Versions Directory
  - We iterated a *LOT*, so we stored our iterations of code in this directory
