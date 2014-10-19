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
PosBufType prePB;// for moving from current to starting point
int num =0;// number of point to be insert to the path from current to starting point
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
  jointAngleMsg.data[2] =   jointCurrentAngle[2];//
  jointAngleMsg.data[3] =   jointCurrentAngle[3]/K;
  robotCurrentPose.position.x = L[0]*cos(jointCurrentAngle[0])+L[1]*cos(jointCurrentAngle[0]+jointCurrentAngle[1]);
  robotCurrentPose.position.y = L[0]*sin(jointCurrentAngle[0])+L[1]*sin(jointCurrentAngle[0]+jointCurrentAngle[1]);
  robotCurrentPose.position.z = jointCurrentAngle[2]-pitch*jointCurrentAngle[4];
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
  geometry_msgs::Pose startPose;
  float startAngle[4];
  float distance = 0;

  float radius;
  float dx;
  float dy;
  float dz;
  float dtheta;

  float tempx;
  float tempy;
  float tempz;
  float temp_theta;

  float temp_theta0;
  float temp_theta1;
  float temp_theta2;
  float temp_theta3;
  float temp_theta4;

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
    /*
     * Generate a path from current pose to target pose
     */
    {
      ROS_INFO("Receive All Data!!");
      startPubFlag = 1;
      startAngle[0] = (PB.data[0][0]-JZero[0])*JRes[0]+PI/2.0;
      startAngle[1] = (PB.data[0][1]-JZero[1])*JRes[1];
      startAngle[2] = (PB.data[0][2]-JZero[2])*JRes[2];
      startAngle[3] = (PB.data[0][3]-JZero[3])*JRes[3];
      ROS_INFO("startingAngle0 is %f",startAngle[0]);
      startPose.position.x = L[0]*cos(startAngle[0])+L[1]*cos(startAngle[0]+startAngle[1]);
      startPose.position.y = L[0]*sin(startAngle[0])+L[1]*sin(startAngle[0]+startAngle[1]);
      startPose.position.z = startAngle[2]-pitch*startAngle[3];
      startPose.orientation.w = startAngle[0]+startAngle[1]+startAngle[3];

      ROS_INFO("start(x,y,z,theta) is (%f,%f,%f,%f)",startPose.position.x,startPose.position.y,startPose.position.z,startPose.orientation.w);
      ROS_INFO("current(x,y,z,theta) is (%f,%f,%f,%f)",robotCurrentPose.position.x,robotCurrentPose.position.y,robotCurrentPose.position.z,robotCurrentPose.orientation.w);

      distance = sqrt(
(startPose.position.x - robotCurrentPose.position.x)*(startPose.position.x - robotCurrentPose.position.x)
+
(startPose.position.y - robotCurrentPose.position.y)*(startPose.position.y - robotCurrentPose.position.y)
+
(startPose.position.z - robotCurrentPose.position.z)*(startPose.position.z - robotCurrentPose.position.z)
//+
//2*(startPose.orientation.w - robotCurrentPose.orientation.w)*(startPose.orientation.w - robotCurrentPose.orientation.w)
);
      ROS_INFO("distance is %f",distance);
      num = distance*2;
      ROS_INFO("plan %d points in the path",num);
      dx = (startPose.position.x - robotCurrentPose.position.x)/num;
      dy = (startPose.position.y - robotCurrentPose.position.y)/num;
      dz = (startPose.position.z - robotCurrentPose.position.z)/num;
      dtheta = (startPose.orientation.w - robotCurrentPose.orientation.w)/num;
      if( num < 2000 )
	for(i=0;i<=num;i++)
	  {
	    tempx = robotCurrentPose.position.x + dx*i;
	    tempy = robotCurrentPose.position.y + dy*i;
	    temp_theta = robotCurrentPose.orientation.w+dtheta*i;
	    tempz = robotCurrentPose.position.z + dz*i;
	    // inverse to joint position
	    radius = sqrt(tempx*tempx + tempy*tempy);
	    temp_theta0 = PI/2 - atan(tempx/tempy);
	    temp_theta1 = temp_theta0 - acos((radius*radius+L[0]*L[0]-L[1]*L[1])/2.0/radius/L[0]);
	    temp_theta2 = PI - acos((L[0]*L[0]+L[1]*L[1]-radius*radius)/2.0/L[0]/L[1]);
	    temp_theta4 = temp_theta - temp_theta1 - temp_theta2;
	    temp_theta3 = tempz + pitch*temp_theta4;
	    
	    prePB.data[i][0] = (temp_theta1-PI/2.0)/JRes[0]+JZero[0];
	    prePB.data[i][1] = (temp_theta2/JRes[1]+JZero[1]);
	    prePB.data[i][2] = temp_theta3/JRes[2]+JZero[2];
	    prePB.data[i][3] = temp_theta4/JRes[3]+JZero[3];
	  }
      else
	{
	  ROS_INFO("too far away!!");
	}
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

	  if( j < num )
	    {
	      serialMsg.data[0] = 1;
	      serialMsg.data[1] = prePB.data[j][0];
	      serialMsg.data[2] = prePB.data[j][1];
	      serialMsg.data[3] = prePB.data[j][2];
	      serialMsg.data[4] = prePB.data[j][3];
	    }
	  else if( j < num + PB.counter )
	    {
	      serialMsg.data[0] = 1;
	      serialMsg.data[1] = PB.data[j-num][0];
	      serialMsg.data[2] = PB.data[j-num][1];
	      serialMsg.data[3] = PB.data[j-num][2];
	      serialMsg.data[4] = PB.data[j-num][3];
	    }
	  else if( j < num +PB.counter+1)
	    {
	      serialMsg.data[0] = 2;
	      serialMsg.data[1] = PB.data[j-num-1][0];
	      serialMsg.data[2] = PB.data[j-num-1][1];
	      serialMsg.data[3] = PB.data[j-num-1][2];
	      serialMsg.data[4] = PB.data[j-num-1][3];  
	    }
	  serialSend_pub.publish(serialMsg);
	  j++;
	  //	  ROS_INFO("counter is %d",PB.counter);
	  if( j > num + PB.counter+1 )
	    {
	      ROS_INFO("Total %d data.",j-1);
	      startPubFlag = 0;
	      PB.counter=0;
	      j=0;
	      num = 0;
	    }
	}
      /* publish topic at 100Hz */
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
