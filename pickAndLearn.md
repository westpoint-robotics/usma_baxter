### Instructions to execute the 'Pick and Learn' exercise on Baxter through a configured computer:
1. Setup the ROS environment for baxter
 - `./baxter.sh`

2. Enable baxter:
 - `rosrun baxter_tools enable_robot.py -e`

3. Launch the pick and learn demo:
 - `roslaunch baxter_pickandlearn baxter_pandl.launch`

4. Set objects:
- Using the "wrist" of the baxter, manipulate the gripper such that the object is between them, the bottom of the grippers are at the desired level for gripping, and the object is outlined in blue with the red cross inside the blue area. Once these conditions are met, press 'A' on the controller to register and pick up the object.

5. Place objects:
- With the object gripped, move the gripper using the wrist to the desired drop location. Once in this position, press 'A' to release the object.

6. Searching for objects:
- Once all objects have been registered, press 'B' on the controller to go to search mode. While in this mode, the baxter arm will navigate to its neutral position and evaluate the shapes around it. If a shape is detected that has been registered, that shape will light up green. To pick that shape up and move it to the dropoff point, press 'A' on the controller.

7. Resetting the objects:
- To delete all objects from memory and start over, press the 'X' button on the controller
