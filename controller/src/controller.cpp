#include "ros/ros.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/String.h"
#include "controller/JointInfo.h"
#include "controller/JointCommand.h"
#include "canopen.h"

controller::JointInfo JointInfo;
controller::JointCommand Joint;

void callback( const controller::JointInfo::ConstPtr& msg )
{
  //ROS_INFO("JointInfo Callback");
  int id=0;
  for(id=0;id<5;id++)
    {
      JointInfo.Status[id]=msg->Status[id];
      JointInfo.Pos[id]=msg->Pos[id];      
      JointInfo.Vel[id]=msg->Vel[id];
      JointInfo.Torque[id]=msg->Torque[id];
    }
}

int main( int argc, char **argv )
{
  ros::init(argc,argv,"controller");
  ros::NodeHandle n;
  n.setParam("SCARA_RUN",0);
  ros::Subscriber sub = n.subscribe("JointInfo", 10,callback);
  ros::Publisher controller_pub = n.advertise<controller::JointCommand>("JointCommand",5);
  /*
    while(pub.getNumSubscribers() == 0)
    {
    sleep(1);
    ROS_INFO("Waiting for subscriber...");
    }
  */
  ros::Rate loop_rate(100);
  ROS_INFO("Run at 100Hz");
  int A=20;
  float w=0.05;
  int y;
  int t;
  int scara_run;
  JointInfo.Status.resize(5);
  JointInfo.Pos.resize(5);
  JointInfo.Vel.resize(5);
  JointInfo.Torque.resize(5);
  Joint.Mode.resize(5);
  Joint.Operation.resize(5);
  Joint.CmdValue.resize(5);
  Joint.Mode[1]=7;
  Joint.Operation[1]=0;
  Joint.Mode[2]=7;
  Joint.Operation[2]=0;
  Joint.Mode[3]=7;
  Joint.Operation[3]=0;
  Joint.Mode[4]=7;
  Joint.Operation[4]=0;
  while(ros::ok())
    {
      y=A*sin(w*t);
      t++;
      n.getParam("SCARA_RUN",scara_run);
      if(scara_run==1)
	{
	  Joint.Operation[1]=1;
	  Joint.Operation[2]=1;
	  Joint.Operation[3]=1;
	  Joint.Operation[4]=1;
	  Joint.CmdValue[4]=Joint.CmdValue[4]+50;
	  //Joint.CmdValue[2]=JointInfo.Pos[2];
	  //Joint.CmdValue[3]=JointInfo.Pos[3];
	  //Joint.CmdValue[4]=JointInfo.Pos[4];
	}
      else
	{
	  Joint.Operation[1]=0;
	  Joint.Operation[2]=0;
	  Joint.Operation[3]=0;
	  Joint.Operation[4]=0;
	  Joint.CmdValue[1]=JointInfo.Pos[1];
	  Joint.CmdValue[2]=JointInfo.Pos[2];
	  Joint.CmdValue[3]=JointInfo.Pos[3];
	  Joint.CmdValue[4]=JointInfo.Pos[4];
	}

      controller_pub.publish(Joint);

      ros::spinOnce();
      loop_rate.sleep();
    }
   return 0;
}


