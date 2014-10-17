#include "ros/ros.h"
#include "canopen.h"
#include "controller/canMessage.h"
#include "controller/JointInfo.h"
#include "controller/JointCommand.h"

#define IDLE_STATE (0X00)
#define MODE_SET_STATE (0X01)
#define OPERATION_SET_STATE (0X02)
#define TARGET_SET_STATE (0X03)

controller::JointInfo JointInfo;
controller::canMessage canMsg;
controller::JointCommand JointCmd;
int id;
void SetCANMessage(int ID, int PreIndex, int Index, int SubIndex, int Data)
{
  canMsg.ID = ID;
  canMsg.PreIndex = PreIndex;
  canMsg.Index = Index;
  canMsg.SubIndex = SubIndex;
  canMsg.Data = Data;
}
void callback( const controller::canMessage::ConstPtr& msg )
{
  // Actual Joint Position
  if( msg->Index== POSITION_ACTUAL_VALUE_INDEX )
    {
      //JointPosition[msg->PreIndex]=msg->Data;
      //ROS_INFO("Joint%dPostion=%d",msg->PreIndex,msg->Data);
      JointInfo.Pos[msg->PreIndex]=msg->Data;
    }
  // Joint Driver Status
  else if(msg->Index==STATUSWORD_INDEX)
    {
      //JointStatus[msg->PreIndex]=msg->Data;
      JointInfo.Status[msg->PreIndex]=msg->Data;
      //ROS_INFO("Joint%dStatus=%d",msg->PreIndex,msg->Data);
    }
}
/*
 * Callback Function 2;
 * In this function, we receive messages from /JointCommand topic
 * And Send them to CAN BUS
 */
void callback2( const controller::JointCommand::ConstPtr& msg )
{
  //  ROS_INFO("Publish Joint");
  int i=0;
  for(i=0;i<5;i++)
    {
      JointCmd.Mode[i] = msg->Mode[i];
      JointCmd.Operation[i] = msg->Operation[i];
      JointCmd.CmdValue[i] = msg->CmdValue[i];
    }
}
/*
 * In Main Function: 
 * We receive CAN Messages from can_bus Node and Parse them, then publish them to /JointInfo topic
 * At the same time, We receive JointCommand from Controller and Parse them, then publish them to /canSend topic
 */
int main( int argc, char **argv )
{
  ros::init(argc,argv,"canopen");
  ros::NodeHandle n;
  ros::Subscriber canopen_sub = n.subscribe("canReceive", 10,callback);
  ros::Subscriber command_sub = n.subscribe("JointCommand", 10,callback2);
  ros::Publisher JointInfo_pub = n.advertise<controller::JointInfo>("JointInfo",10);
  ros::Publisher canSend_pub = n.advertise<controller::canMessage>("canSend",10);
  ros::Rate loop_rate(1000);

  int counter=0;
  int sendState[5]={MODE_SET_STATE,MODE_SET_STATE,MODE_SET_STATE,MODE_SET_STATE,MODE_SET_STATE};
  int t=0;
  ROS_INFO("Run at 100Hz");

  JointInfo.Pos.resize(5);
  JointInfo.Status.resize(5);
  JointInfo.Vel.resize(5);
  JointInfo.Torque.resize(5);
  
  JointCmd.Mode.resize(5);
  JointCmd.Operation.resize(5);
  JointCmd.CmdValue.resize(5);

  JointCmd.Mode[1]=-1;
  JointCmd.Mode[2]=-1;
  JointCmd.Mode[3]=-1;
  JointCmd.Mode[4]=-1;
  SetCANMessage(0x05,CAN_NODE_MASTER,CONTROLWORD_INDEX,0,0x01);
  canSend_pub.publish(canMsg);
  ROS_INFO("RUN DEMO");
  // canopen node run at 1000Hz

  while(ros::ok())
    {	
      counter++;
      if(counter>=10)
	{
	  counter = 0;
	  JointInfo_pub.publish(JointInfo);
	}

      // if(t<1000)
      // 	{
      // 	  SetCANMessage(0x05,CAN_NODE_MASTER,CONTROLWORD_INDEX,0,0x01);
      // 	  canSend_pub.publish(canMsg);
      // 	  t++;
      // 	}


      ros::spinOnce();
      loop_rate.sleep();
    }
  return 0;
}


/*
 // Set Mode - First 
	  // Check Status bit and set Operation Mode
	  if( (
	       (JointCmd.Mode[id] != NewJointCmd.Mode[id])
	       ||
	       ((JointInfo.Status[id]&STATUSWORD_READY_BIT)!=STATUSWORD_READY_BIT)
	       )
	      &&
	      ((JointInfo.Status[id]&STATUSWORD_SWITCH_BIT)!=STATUSWORD_SWITCH_BIT) )
	    {
	      JointCmd.Mode[id] = NewJointCmd.Mode[id];
	      SetCANMessage(id,CAN_NODE_MASTER,MODES_OF_OPERATION_INDEX,0,JointCmd.Mode[id]);
	      canSend_pub.publish(canMsg);
	    }

	  // Set Operation - Second
	  else if( (JointCmd.Operation[id] != NewJointCmd.Operation[id])||((JointInfo.Status[id]&STATUSWORD_SWITCH_BIT)!=STATUSWORD_SWITCH_BIT) )  
	    {
	      JointCmd.Operation[id]=NewJointCmd.Operation[id];      
	      SetCANMessage(id,CAN_NODE_MASTER,CONTROLWORD_INDEX,0,JointCmd.Operation[id]);
	      canSend_pub.publish(canMsg);
	    }
	  // Set Target Value - Third
	  else //if(JointCmd.CmdValue[id] != NewJointCmd.CmdValue[id])
	    {
	      JointCmd.CmdValue[id]=NewJointCmd.CmdValue[id];
	      switch(JointCmd.Mode[id])
		{
		case PROFILE_POSITION_MODE:
		  SetCANMessage(id,CAN_NODE_MASTER,TARGET_POSITION_INDEX,0,JointCmd.CmdValue[id]);
		  canSend_pub.publish(canMsg);
		  //sleep(messageDelay);
		  break;
		case VELOCITY_MODE:
		  SetCANMessage(id,CAN_NODE_MASTER,TARGET_VELOCITY_INDEX,0,JointCmd.CmdValue[id]);
		  canSend_pub.publish(canMsg);
		  ROS_INFO("Send:%d",JointCmd.CmdValue[1]);
		  //		  sleep(messageDelay);
		  break;
		case PROFILE_VELOCITY_MODE:
		  SetCANMessage(id,CAN_NODE_MASTER,TARGET_VELOCITY_INDEX,0,JointCmd.CmdValue[id]);
		  canSend_pub.publish(canMsg);
		  //sleep(messageDelay);
		  break;
		case TORQUE_PROFILE_MODE:
		  SetCANMessage(id,CAN_NODE_MASTER,TARGETTORQUE_INDEX,0,JointCmd.CmdValue[id]);
		  canSend_pub.publish(canMsg);
		  //sleep(messageDelay);
		  break;
		case HOMING_MODE:
		  //	      SetCANMessage(id,CAN_NODE_MASTER,TARGET_VELOCITY_INDEX,0,JointCmd.CmdValue[id]);
		  canSend_pub.publish(canMsg);
		  //sleep(messageDelay);
		  break;
		case INTERPOLATED_POSITION_MODE:
		  SetCANMessage(id,CAN_NODE_MASTER,TARGET_POSITION_INDEX,0,JointCmd.CmdValue[id]);
		  canSend_pub.publish(canMsg);
		  //sleep(messageDelay);
		  break;
		default:
		  canSend_pub.publish(canMsg);
		  ROS_INFO("Wrong CAN Message Published");
		  // sleep(messageDelay);
		  break;
		} 
	    }
*/
