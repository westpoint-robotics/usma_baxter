# usma_baxter
### Instructions on how to setup a workstation and bringup the EECS Baxter robot

#### 1. Follow instructions on [Rethink Robotics wiki](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup) until Step 6

#### 2. After Step 6, perform the following before proceeding to Step 7.
1. Connect Baxter to your workstation using a CAT5 cable and set up a new wired connection as follows:
 - Click on 'Edit Connections' in the networking menu and edit the active wired connection.
 - Name the connection 'baxter'
 - Under the 'General' tab, uncheck 'Automatically connect to this network when it is available'.
 - Under the 'IPv4 Settings' tab, select 'Manual' for Method instead of 'DHCP'. 
 - Under the 'IPv4 Settings' tab, add IP address 169.254.11.xx, subnet mask 255.255.255.0
 - Save and close Network Connections.
 
2. Edit the hosts file and add baxter's IP address with its serial number(which is the hostname).
 - `gksu gedit /etc/hosts`
 - `169.254.11.86 <tab> 011504P0022`
 
#### 3. You may now proceed to [Step 7](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup) to verify your environment setup.

#### 4. And then onto [communicating with Baxter](http://sdk.rethinkrobotics.com/wiki/Hello_Baxter).


#### For instructions on running the 'Pick and Learn' demo, click [here](https://github.com/westpoint-robotics/usma_baxter/blob/master/pickAndLearn.md).

### Some useful commands:
1. If the sonar sensors are on when Baxter startsup you can turn them off with:
`rostopic pub /robot/sonar/head_sonar/set_sonars_enabled std_msgs/UInt16 0`

2. To tuck Baxter's arm for storage: 
  * Enable Baxter if not already enabled:
  - `rosrun baxter_tools enable_robot.py -e`
  * Send the command to tuck arms:
  - `rosrun baxter_tools tuck_arms.py -t`
  * To untuck the arms:
  -`rosrun baxter_tools tuck_arms.py -u`
  * To disable Baxter: 
  - `rosrun baxter_tools enable_robot.py -e`



