### Instructions to execute the 'Pick and Learn' exercise on Baxter through a configured computer:
1. Setup the ROS environment for baxter
 - `./baxter.sh`

2. Enable baxter:
 - `rosrun baxter_tools enable_robot.py -e`

3. Launch the pick and learn demo:
 - `roslaunch baxter_pickandlearn baxter_pandl.launch`
