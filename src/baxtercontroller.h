#include <time.h>
#include <math.h>
#include <ros/ros.h>
#include <baxter_core_msgs/ITBState.h>
#include <baxter_core_msgs/EndEffectorState.h>
#include <baxter_core_msgs/EndEffectorCommand.h>
#include <baxter_core_msgs/EndpointState.h>
#include <baxter_core_msgs/SolvePositionIK.h>
#include <baxter_core_msgs/JointCommand.h>
#include <sensor_msgs/Range.h>

const clock_t INPUT_BLOCKING_TIME = CLOCKS_PER_SEC/2;

class BaxterController
{
public:
    enum ITBInput {INPUT_NOTHING, INPUT_WHEEL_CLICKED, INPUT_BACK_CLICKED, INPUT_HOME_CLICKED};
    BaxterController(ros::NodeHandle nh);
    ~BaxterController();
    void itbCallback(const baxter_core_msgs::ITBStateConstPtr &msg);
    void gripperCallback(const baxter_core_msgs::EndEffectorStateConstPtr &msg);
    void irCallback(const sensor_msgs::RangeConstPtr &msg);
    void endpointCallback(const baxter_core_msgs::EndpointStateConstPtr &msg);
    ITBInput getInput();
    float getRange();
    void getPosition(float position[]);
    void getOrientation(float orientation[]);
    void grip();
    void release();
    int move(float position[], float orientation[]);
    int moveTo(float position[], float orientation[]);
    void stop();
    float distanceToSetPosition();

private:
    ros::NodeHandle nh;
    ros::Subscriber itb_sub, gripper_sub, ir_sub, endpoint_sub;
    ros::Publisher gripper_pub, joint_pub;
    ros::ServiceClient ik_client;
    baxter_core_msgs::JointCommand joint_cmd;
    ITBInput input;
    clock_t last_input_time;
    unsigned int gripper_hid;
    float range;
    double position[3], set_position[3];
    double orientation[4], set_orientation[4];
    bool has_to_move;
};
