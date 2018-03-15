/*
This file is part of Baxter Pick and Learn.

Baxter Pick and Learn is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Nagen is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <http://www.gnu.org/licenses/>.

Copyright 2014 Charles Hubain <charles.hubain@haxelion.eu>
*/

#include <ros/ros.h>
#include <vector>
#include <math.h>
#include "baxtercontroller.h"
#include "camera.h"

#include "sensor_msgs/Joy.h"

#define MAX_PASS 4

class RobotState {
 public:
  RobotState() {
    joy_wheel_clicked = false;
    joy_home_clicked = false;
    joy_reset = false;
    last_button0 = 0;
    last_button1 = 0;
    last_button2 = 0;
  }
  bool joy_wheel_clicked;
  bool joy_home_clicked;
  bool joy_reset;
  void joyCallback(const sensor_msgs::Joy::ConstPtr &msg) {
    if ((last_button0 == 0) && (msg->buttons[0] == 1)) {
      joy_wheel_clicked = true;
    } else {
      joy_wheel_clicked = false;
    }
    if ((last_button1 == 0) && (msg->buttons[1] == 1)) {
      joy_home_clicked = true;
    } else {
      joy_home_clicked = false;
    }
    if ((last_button2 == 0) && (msg->buttons[2] == 1)) {
      joy_reset = true;
    } else {
      joy_reset = false;
    }
    last_button0 = msg->buttons[0];
    last_button1 = msg->buttons[1];
    last_button2 = msg->buttons[2];
//    ROS_INFO("Button 0 is %d and joywheel is %d", msg->buttons[0],
//             joy_wheel_clicked);
  }

 private:
  int last_button0;
  int last_button1;
  int last_button2;
};

int main(int argc, char **argv) {
  RobotState robotState;
  // Initialise ROS
  ros::init(argc, argv, "baxter_pickandlearn");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  ros::Subscriber sub =
      nh.subscribe("joy", 1, &RobotState::joyCallback, &robotState);
  spinner.start();
  BaxterController *robot = new BaxterController(nh);
  std::vector<Piece> pieces;
  Camera *camera = new Camera(Camera::RIGHT_HAND, nh, pieces);
  ros::Rate loop_rate(10);
  bool reset = true;
  bool learning = false;
  int state = 0;
  float obs_position[3] = {0.55, -0.4, -0.05};
  float obs_orientation[4] = {1, 0, 0, 0};
  float position[3], obj_position[3], orientation[4], obj_orientation[4];
  int pass, match;

  sensor_msgs::Joy lastJoy;
  while (ros::ok()) {
    // Reseting =====================================================
    if (reset) {
      robot->moveTo(obs_position, obs_orientation);
      std::cout << "Resetting arm to Observation Position" << std::endl;
      if (robot->distanceToSetPosition() < 0.01) {
        pieces.clear();  // Clear the list of peices for retraining
        state = 0;
        robot->release();
        reset = false;
        learning = true;
      }
    }
    else if (learning) {
    // Learning =====================================================
      BaxterController::ITBInput input = robot->getInput();
      robot->getPosition(position);
      robot->getOrientation(orientation);
      if (state == 0) {
        if ((input == BaxterController::INPUT_WHEEL_CLICKED) or
            (robotState.joy_wheel_clicked)) {
          camera->request(Camera::REQUEST_SELECTED_SHAPE);
          state = 1;
        }
      } else if (state == 1 && camera->isResultAvailable()) {
        std::vector<std::vector<cv::Point> > *result = camera->getResult();
        if (result->size() == 0 || (*result)[0].size() == 0) {
          std::cout << "No result" << std::endl;
          state = 0;
        } else {
          state = 2;
          char buffer[32];
          snprintf(buffer, 32, "Piece %d", int(pieces.size()) + 1);
          pieces.push_back(Piece((*result)[0], position[2], orientation,
                                 std::string(buffer)));
          std::cout << pieces.back().getName() << " saved:" << std::endl;
          robot->grip();
        }
        delete result;
      } else if (state == 2 &&
                     (input == BaxterController::INPUT_WHEEL_CLICKED) or
                 (robotState.joy_wheel_clicked)) {
        pieces.back().setDropPosition(position, orientation);
        state = 0;
        robot->release();
      }
      if ((input == BaxterController::INPUT_HOME_CLICKED) or
          (robotState.joy_home_clicked)) {
        learning = false;
        state = 0;  // This is the do nothing state until button pressed.
      }
      // TODO tell baxter to put the arm in the current pos.
      robot->moveTo(position, orientation);

      std::cout << "Learning state: " << state
                << " joy_wheel_clicked: " << robotState.joy_wheel_clicked
                << ", joy_home_clicked: " << robotState.joy_home_clicked
                << std::endl;
    } else if (not reset) { 
    // Sorting ===================================================== 
      BaxterController::ITBInput input = robot->getInput();
      if (state == 0) { // Move to observation position
        robot->moveTo(obs_position, obs_orientation);
        if (robot->distanceToSetPosition() < 0.01) {
          pass = 1;
          state = 1;
        }
      } else if (state == 1) { // Wait for button to continue and open gripper
        robot->release();
        if ((input == BaxterController::INPUT_WHEEL_CLICKED) or (robotState.joy_wheel_clicked)) {
            state = 2;
          }
      } else if (state == 2) { // Request shapes from camera
        camera->request(Camera::REQUEST_SHAPES);
        state = 3;
      } else if (state == 3 && camera->isResultAvailable()) { // We have shapes so move to shape
        std::vector<std::vector<cv::Point> > *result = camera->getResult();
        if (result->size() == 0 || (*result)[0].size() == 0) { // If shape lost go back to state 2 and request shapes
          std::cout << "No shape found" << std::endl;
          state = 2;
        } else {
          robot->getPosition(position);
          if (camera->getClosestMatchApproach(result, pieces, position[2],
                                              obj_position, obj_orientation,
                                              match)) { // No peice found go back to state 2 and request shapes
            std::cout << "No piece found" << std::endl;
            state = 2;
          } else { // We have a piece 
            for (int i = 0; i < 3; i++) obj_position[i] += position[i]; 
            if (pass == MAX_PASS) { // Tried to many times
              if (robot->moveTo(obj_position, obj_orientation))
                state = 2; // not at objective location, go back to state 2
              else
                state = 4; // at objective go to state 4
            } else {// Do smooth approach
              obj_position[0] -= 0.1 - 0.08 * pass / MAX_PASS;
              obj_position[2] += 0.1 - 0.08 * pass / MAX_PASS;
              if (robot->moveTo(obj_position, obs_orientation))
                state = 2; // not at objective location, go back to state 2
              else
                state = 4; // at objective go to state 4
            }
          }
        }
        delete result;
      } else if (state == 4) { // At objective location
        if (robot->distanceToSetPosition() < 0.01) {
          pass++;
          if (pass <= MAX_PASS)
            state = 2;
          else {
            state = 5;
            if (robot->moveTo(obj_position, obj_orientation)) state = 0;
          }
        }
      } else if (state == 5) { // At peice location close gripper
        if (robot->distanceToSetPosition() < 0.01) {
          robot->grip();
          ros::Duration(0.5).sleep();
          obj_position[2] += 0.2;
          state = 6;
          if (robot->moveTo(obj_position, obj_orientation)) state = 0;
        }
      } else if (state == 6) {
        if (robot->distanceToSetPosition() < 0.01) {
          state = 7;
          pieces[match].getDropPosition(obj_position);
          pieces[match].getDropOrientation(obj_orientation);
          if (robot->moveTo(obj_position, obj_orientation)) state = 0;
        }
      } else if (state == 7) {
        if (robot->distanceToSetPosition() < 0.01) {
          robot->release();
          state = 0;
        }
      }
      std::cout << "Sorting state: " << state
                << " pass: " << pass
                << " joy_wheel_clicked: " << robotState.joy_wheel_clicked
                << ", joy_home_clicked: " << robotState.joy_home_clicked
                << std::endl;
    }
    
      if (robotState.joy_reset) {
        reset = true;
      }
    loop_rate.sleep();
  }
  spinner.stop();
  delete robot;
  delete camera;
  return 0;
}

