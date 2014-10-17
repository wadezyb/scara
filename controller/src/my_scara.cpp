/*
 * Node: my_scara.cpp
 * 1.Subscripe /angle topic and convert the angles to pulse
 * 2.Judge the starting point and generate a path from current pose to starting pose
 * 3.Subscribe to /robotInfo, get Joint Position feedback.
 * 4 Generate a path from current pose to starting pose, length depends on the distance between current pose to starting pose.
 * 5 Publish the /serialSend topic at 1000Hz
*/
#include "ros/ros.h"
#include "canopen.h"
#include "controller/canMessage.h"
#include "controller/JointInfo.h"
#include "controller/JointCommand.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "geometry_msgs/Pose.h"

#define PI (3.14159265359)
#define K (PI/180.0)

/* Struct */
typedef struct
{
  int counter;
  int data[2000][4];
}PosBufType;

/* Public Variables */
controller::JointInfo JointInfo;
controller::JointCommand JointCmd;
std_msgs::Int32MultiArray serialMsg;
std_msgs::Float32MultiArray jointAngleMsg;
geometry_msgs::Pose robotCurrentPose;

int L[]={225,125};//Arm length
float JRes[]={0.0010986*K,0.0017578*K,0.0032552,0.0077097*K};// ras/pulse, rad/pulse, mm/pulse, rad/pulse
int JZero[]={(109958-106449)/2,(57480-111044)/2,-3957,3290};
PosBufType PB;//Position Buffer
int startPubFlag = 0;
float jointCurrentAngle[4];
float pitch = 16.0/PI;
/*
 * Callback Fuction
 * 1. Get Driver Information and save to JointInfo.
*/
void callback( const std_msgs::Int32MultiArray::ConstPtr& msg )
{
  jointCurrentAngle[0] = (msg->data[0]-JZero[0])*JRes[0]+PI/2;
  jointCurrentAngle[1] = (msg->data[1]-JZero[1])*JRes[1];
  jointCurrentAngle[2] = (msg->data[2]-JZero[2])*JRes[2];
  jointCurrentAngle[3] = (msg->data[3]-JZero[3])*JRes[3];
  jointAngleMsg.data[0] =   jointCurrentAngle[0]/K;
  jointAngleMsg.data[1] =   jointCurrentAngle[1]/K;
  jointAngleMsg.data[2] =   jointCurrentAngle[2];
  jointAngleMsg.data[3] =   jointCurrentAngle[3]/K;
  robotCurrentPose.position.x = L[0]*cos(jointCurrentAngle[0])+L[1]*cos(jointCurrentAngle[0]+jointCurrentAngle[1]);
  robotCurrentPose.position.y = L[0]*sin(jointCurrentAngle[0])+L[1]*sin(jointCurrentAngle[0]+jointCurrentAngle[1]);
  robotCurrentPose.position.z = jointCurrentAngle[2]-jointCurrentAngle[4]*pitch;
  robotCurrentPose.orientation.x = 0;
  robotCurrentPose.orientation.y = 0;
  robotCurrentPose.orientation.z = 1;
  robotCurrentPose.orientation.w =  jointCurrentAngle[0]+jointCurrentAngle[1]+jointCurrentAngle[3];
}
/*
 * Callback Function 2;
 * 1. In this function, we receive messages from /angle topic
 * 2. Convert it from joint command angle to joint command position using zero pos.
 * 3. Save the data to a data buffer.
 * Todo:
 * 4. Compare the starting point with the robot current position
 * 5. Generate a path from current position to the starting position
 */
void callback2( const std_msgs::Float32MultiArray::ConstPtr& msg )
{
  //  ROS_INFO("Publish Joint");
  float angle[4];
  int pos[4];
  int i=0;

  if( msg->data[4] != 0 )
    {
      PB.counter =  msg->data[4];
      //      ROS_INFO("counter is %d",PB.counter);
      for(i=0;i<4;i++)
	{
	  angle[i] = msg->data[i];
	  pos[i] = angle[i]/JRes[i]+JZero[i];
	  //	  ROS_INFO("%d J%d is %d",PB.counter-1,i,pos[i]);
	  PB.data[PB.counter-1][i]=pos[i];
	}
      //      ROS_INFO("data is %d",PB.data[PB.counter-1][3]);
    }
  else
    {
      ROS_INFO("Receive All Data!!");
      // Judge current end effector position and the starting point
      // Here we need the data from controller: JointPos
      startPubFlag = 1;
      //      PB.counter=0;
    }
}
/*
 * In Main Function: 
 * We receive CAN Messages from can_bus Node and Parse them, then publish them to /JointInfo topic
 * At the same time, We receive JointCommand from Controller and Parse them, then publish them to /canSend topic
 */
int main( int argc, char **argv )
{
  int counter = 0;
  jointAngleMsg.data.resize(4);
  serialMsg.data.resize(5);
  ros::init(argc,argv,"my_scara");
  ros::NodeHandle n;
  ros::Subscriber canopen_sub = n.subscribe("robotInfo", 10,callback);
  ros::Subscriber command_sub = n.subscribe("angle", 10,callback2);
  ros::Publisher jointAngle_pub = n.advertise<std_msgs::Float32MultiArray>("jointAngle",10);
  ros::Publisher robotCurrentPose_pub = n.advertise<geometry_msgs::Pose>("robotCurrentPose",10);
  ros::Publisher serialSend_pub = n.advertise<std_msgs::Int32MultiArray>("serialSend",10);
  ros::Rate loop_rate(1000);
  ROS_INFO("Run at 1000Hz");
  int j=0;
  while(ros::ok())
    {	
      
      if( startPubFlag == 1 )
	{
	  serialMsg.data[0] = 1;
	  serialMsg.data[1] = PB.data[j][0];
	  serialMsg.data[2] = PB.data[j][1];
	  serialMsg.data[3] = PB.data[j][2];
	  serialMsg.data[4] = PB.data[j][3];
	  //	  ROS_INFO("%d 4st Data is %d",PB.counter,serialMsg.data[4]);

	  serialSend_pub.publish(serialMsg);
	  j++;
	  //	  ROS_INFO("counter is %d",PB.counter);
	  if( j == PB.counter )
	    {
	      // ROS_INFO("Total %d data.",j

		       );
	      startPubFlag = 0;
	      PB.counter=0;
	      j=0;
	      //	      ROS_INFO("SIZE %d",sizeof(PB.data));

	    }
	}
      if( counter >= 10 )
	{
	  counter = 0;
	  jointAngle_pub.publish(jointAngleMsg);
	  robotCurrentPose_pub.publish(robotCurrentPose);

	}

      counter++;
      ros::spinOnce();
      loop_rate.sleep();
    }
  return 0;
}
