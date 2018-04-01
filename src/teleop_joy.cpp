//ROS
#include <ros/ros.h>

//ROS JOY package
#include <sensor_msgs/Joy.h>

//Round function
#include <math.h> 

//Mavros messages
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/OverrideRCIn.h>

using namespace std;

//Class to send /mavros/rc/override messages
class Teleop
{
    public:
        //Default Constructor
        Teleop();
        //Function to keep program from closing
        void spin();

    private:
        //callback for joy messages
        void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
        //Arm ardusub
        void setArming(bool arm);
        //Change rover mode
        void setMode(string mode);
        //map function to get value between 1000-2000
        int map(float x);
   
        //ROS node handle, subs, pubs, service clients
        ros::NodeHandle nh;
        ros::Subscriber joy_sub_;
        ros::Publisher rc_override_pub;
        ros::ServiceClient cmd_client;
        ros::ServiceClient set_mode;
        ros::Publisher actuator_controls_pub;

        //Array to store override message values
        int RCmsg[8];
        //bool flags to change left joy stick controls
        bool arm_status;
        bool flag_lr;
        bool flag_ud;
        //Enum for arming message value
        enum {COMPONENT_ARM_DISARM=400};
};

/*
    Defult constructor initialzes RCmsg, joy stick flags, subs and pubs
        Inputs: None
        Returns: None
*/
Teleop::Teleop()
{
  //Initialize subs, pubs and ServiceClient
  joy_sub_ = nh.subscribe<sensor_msgs::Joy>("joy", 10, &Teleop::joyCallback, this);
  rc_override_pub = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1);
  cmd_client = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");

  //Initialize RCmsg array
  RCmsg[0] = 1500;
  RCmsg[1] = 1500;
  RCmsg[2] = 1500;
  RCmsg[3] = 1500;
  RCmsg[4] = 1500;
  RCmsg[5] = 1500;
  RCmsg[6] = 1500;
  RCmsg[7] = 1500;
  
  //Initialize bool flags
  flag_lr = false;
  flag_ud = false;
  arm_status = false;
}

/*
    Spin function to stop the program from closing and send override message at 20Hz. 
    Inputs: None
    Returns: None
    
*/
void Teleop::spin() 
{
  //set publish rate
  ros::Rate loop(20);

  //check if the node is running
  while(ros::ok()) 
  {
    //create override message object and fill values from RCmsg array
    mavros_msgs::OverrideRCIn msg;
    msg.channels[0] = RCmsg[0];
    msg.channels[1] = RCmsg[1];
    msg.channels[2] = RCmsg[2];
    msg.channels[3] = RCmsg[3];
    msg.channels[4] = RCmsg[4];
    msg.channels[5] = RCmsg[5];
    msg.channels[6] = RCmsg[6];
    msg.channels[7] = RCmsg[7];
    
    //Publish override message
    rc_override_pub.publish(msg);
  
    // call all waiting callbacks
    ros::spinOnce();

    // enforce a max publish rate
    loop.sleep();
  }
}

/*
    Callback function runs when a joy message is recieved. It fills the RCmsg array 
    with the appropriate values based on joystick inputs.
    
    Inputs: pointer to joy message from ROS
    Returns: None
*/
void Teleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    //Check if change view/disarm button was pushed
    if(joy->buttons[6])
    {
        //Disarm
        setArming(false);
    }
 
    //check if menu/arm button was pushed
    if(joy->buttons[7])
    {
        //Arm
        setArming(true);
    }
 
    //Check if X button was pushed and if the left joy was not mapped to roll
    if(joy->buttons[2] && flag_lr!=true)
    {
            flag_lr = true;
            ROS_INFO("Roll");
	}
 
 
    //Check if B button was pushed and if the left joy was not mapped to Lateral
    if(joy->buttons[1] && flag_lr!=false)
    {
            flag_lr = false;
            ROS_INFO("Lateral");
	}
  
    //Check if L-R flag is true
    if(flag_lr)
        //if true use L Joystick's LR values to roll
        RCmsg[1] = map(joy->axes[0]);
    else
        //if false use L Joystick's LR values to Lateral
        RCmsg[5] = map(joy->axes[0]);
  
    //set pitch values from left joy's U/D movement
	RCmsg[0] = map(joy->axes[1]);
   
    //Set R Joystick's UD values to throttle
    RCmsg[2] = map(-joy->axes[4]);
    
    //Set R Joystick's LR values to yaw
    RCmsg[3] = map(-joy->axes[3]);
    
    //Set forward to average of both trigger values
    RCmsg[4] = round((map(joy->axes[2]) + map(joy->axes[5]*-1))/2);
}

/*
    Function to arm rover
    Inputs: bool value True = arm, False = disarm
    Returns: None
*/
void Teleop::setArming(bool arm) 
{
    // Arm/disarm method following:
    // https://github.com/mavlink/qgroundcontrol/issues/590
    // https://pixhawk.ethz.ch/mavlink/#MAV_CMD_COMPONENT_ARM_DISARM

	if (arm_status != arm){
		// generate request
		mavros_msgs::CommandLong srv;
		srv.request.command = COMPONENT_ARM_DISARM;
		srv.request.param1 = (arm ? 1 : 0);
		srv.request.param2 = 21196; // force disarm (see GCS_Mavlink.cpp)

		// send request
		if(cmd_client.call(srv)) 
		{
		    ROS_INFO(arm ? "Armed" : "Disarmed");
		    arm_status = arm;
		}
		else 
		{
		    ROS_ERROR("Failed to update arming");
		}
    }
}

/*
    Function to transform input value from input range to new range 1000-2000
    Inputs: Value to be transformed
    Returns: value in the new range
*/
int Teleop::map(float x)
{
    float inmin = -1;
    float inmax = 1;
    float outmin = 1250;
    float outmax = 1750;

    return round(float((x - inmin) * (outmax - outmin) / (inmax - inmin) + outmin));
}

//Main
int main(int argc, char** argv)
{
    //Initialize ros node with "teleop" as the name of the node
    ros::init(argc, argv, "teleop");
    
    //Create object of type teleop
    Teleop teleop;

    //call spin function from teleop class
    teleop.spin();
    
    return 0;
}
