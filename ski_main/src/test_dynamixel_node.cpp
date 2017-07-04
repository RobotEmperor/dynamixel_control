#include <ros/ros.h>
#include "../include/test_dynamixel/dynamixel_MX106.h"


using namespace dynamixel_controller;
using namespace dynamixel;



DynamixelController::DynamixelController()
{

  portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  pbulkread = new GroupBulkRead(portHandler,packetHandler);


}

int DynamixelController::initDynamixel()
{
  if(portHandler->openPort())
  {
    printf("Succeeded to open the port!\n");

  }
  else
  {
    printf("Failed to open!\n");
    return 0;
  }

  if(portHandler->setBaudRate(BAUDRATE))
  {
    printf("\n Succeeded to change the baudrate!\n");
  }

  else
  {
    printf("Failed to change the baudrate!\n ");
    return 0;
  }



}

void DynamixelController::closeDynamixel(int8_t dxl_id[])
{

  setTorque(dxl_id,TORQUE_OFF);
  portHandler->closePort();
}

void DynamixelController::setTorque(int8_t dxl_id[], bool onoff)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  for(int i=1;i<all_DXL;i++)
  {
    if(dxl_id[i] != 0)
    {
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], ADDR_MX_TORQUE_ENABLE, onoff, &dxl_error);

      if(dxl_comm_result !=COMM_SUCCESS)
      {
        packetHandler->printTxRxResult(dxl_comm_result);
      }
      else if (dxl_error !=0)
      {
        packetHandler->printRxPacketError(dxl_error);
      }
      else
      {
        if(onoff == true)
          printf("ID: %d dynamixel torque on\n", dxl_id[i]);
        else if(onoff == false)
          printf("ID: %d dynamixel torque off\n", dxl_id[i]);  
      }
    }
    else dxl_id[i]=0;

  }

}

void DynamixelController::readResister(uint8_t id, uint16_t addr, uint16_t length)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  int8_t  value8  = 0;
  int16_t value16 = 0;
  int32_t value32 = 0;

  if (length == 1)
  {
    dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, id, addr, (uint8_t*)&value8, &dxl_error);
  }
  else if (length == 2)
  {
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, id, addr, (uint16_t*)&value16, &dxl_error);
  }
  else if (length == 4)
  {
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, id, addr, (uint32_t*)&value32, &dxl_error);
  }

  if (dxl_comm_result == COMM_SUCCESS)
  {
    if (dxl_error != 0) packetHandler->printRxPacketError(dxl_error);

    if (length == 1)
    {
      ROS_INFO("[ID] %u, [Present Value] %d", id, value8);

    }
    else if (length == 2)
    {
      ROS_INFO("[ID] %u, [Present Value] %d", id, value16);
      //value = value16;
    }
    else if (length == 4)
    {
      ROS_INFO("[ID] %u, [Present Value] %d", id, value32);
    }
  }
  else
  {
    packetHandler->printTxRxResult(dxl_comm_result);
    ROS_ERROR("[ID] %u, Fail to read!", id);
  }

}

void DynamixelController::writeResister(uint8_t id, uint16_t addr, uint16_t length, int32_t value)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  if (length == 1)
  {
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, addr, (int8_t)value, &dxl_error);
  }
  else if (length == 2)
  {
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, addr, (int16_t)value, &dxl_error);
  }
  else if (length == 4)
  {
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, id, addr, (int32_t)value, &dxl_error);
  }

  if (dxl_comm_result == COMM_SUCCESS)
  {
    if (dxl_error != 0)
      packetHandler->printRxPacketError(dxl_error);
  }
  else
  {
    packetHandler->printTxRxResult(dxl_comm_result);
    ROS_ERROR("[ID] %u, Fail to write!",id);
  }
}


void DynamixelController::sync_Write(int8_t dxl_id[], int position[], int speed[])
{
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_MX_PROFILE_VELOCITY , 8);

  bool dxl_addparam_result = false;
  int dxl_comm_result = COMM_TX_FAIL;
  uint8_t param_goal_position[8];





  for(int i=1; i<all_DXL; i++)
  { 
    param_goal_position[0]= DXL_LOBYTE(DXL_LOWORD(speed[i]));
    param_goal_position[1]= DXL_HIBYTE(DXL_LOWORD(speed[i]));
    param_goal_position[2]= DXL_LOBYTE(DXL_HIWORD(speed[i]));
    param_goal_position[3]= DXL_HIBYTE(DXL_HIWORD(speed[i]));

    param_goal_position[4]= DXL_LOBYTE(DXL_LOWORD(position[i]));
    param_goal_position[5]= DXL_HIBYTE(DXL_LOWORD(position[i]));
    param_goal_position[6]= DXL_LOBYTE(DXL_HIWORD(position[i]));
    param_goal_position[7]= DXL_HIBYTE(DXL_HIWORD(position[i]));



    if(dxl_id[i] != 0)
    {
      dxl_addparam_result = groupSyncWrite.addParam(dxl_id[i], param_goal_position);

      if(dxl_addparam_result != true)
      {
        fprintf(stderr,"[ID: %d] groupSyncWrite addparam failed\n", i);
      }
    }
    else dxl_id[i] = 0;

  }


  dxl_comm_result = groupSyncWrite.txPacket();
  if(dxl_comm_result != COMM_SUCCESS) packetHandler->printTxRxResult(dxl_comm_result);





}

void DynamixelController::sync_Read(int8_t dxl_id[], uint16_t start_addr, uint8_t length)
{
  dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, start_addr, length);

  bool dxl_addparam_result = false;
  bool dxl_getdata_result = false;
  int dxl_comm_result = COMM_TX_FAIL;



  for(int i=1;i<all_DXL;i++)
  {
    if(dxl_id[i] != 0)
    {

      dxl_addparam_result = groupSyncRead.addParam(i);
      if(dxl_addparam_result != true)
      {
        fprintf(stderr,"[ID: %d] groupSyncRead addparam failed\n", i);
      }  
    }
    else dxl_id[i] = 0;
  }

  dxl_comm_result = groupSyncRead.txRxPacket();
  if(dxl_comm_result != COMM_SUCCESS) packetHandler->printTxRxResult(dxl_comm_result);

  for(int i=1;i<all_DXL;i++)
  {
    if(dxl_id[i] != 0)
    {
      dxl_getdata_result = groupSyncRead.isAvailable(dxl_id[i], start_addr, 4);
      if(dxl_getdata_result !=true)
      {
        fprintf(stderr, "[ID : %d] groupSyncRead getdata Faild!\n", dxl_id[i]);
      }

      dxl_getdata_result = groupSyncRead.isAvailable(dxl_id[i], start_addr+4, 4);
      if(dxl_getdata_result !=true)
      {
        fprintf(stderr, "[ID : %d] groupSyncRead getdata Faild!\n", dxl_id[i]);
      }

      real_speed[i]    =  groupSyncRead.getData(dxl_id[i], start_addr, 4);// get velocity data
      real_position[i] =  groupSyncRead.getData(dxl_id[i], start_addr+4, 4);//get position data 

      if(real_speed[i]>1024)
      {
        real_speed[i] = ~real_speed[i];
      }
      else  real_speed[i] = real_speed[i]*1;

      ROS_INFO("[ID: %d]position   %d   ::  speed    %d", dxl_id[i], real_position[i], real_speed[i]);
    }
    else dxl_id[i] = 0;
  }
}



void DynamixelController::bulk_txpacket(int8_t dxl_id[], uint16_t start_addr_read, uint8_t length)
{


  bool dxl_addparam_result = false;
  bool dxl_getdata_result = false;
  int dxl_comm_result = COMM_TX_FAIL;


  for(int i=1;i<all_DXL;i++)
  {

    if(dxl_id[i] != 0)
    {
      dxl_addparam_result = pbulkread->addParam(dxl_id[i], start_addr_read, length);
      if (dxl_addparam_result != true)
      {
        fprintf(stderr, "[ID:%03d] grouBulkRead addparam failed", dxl_id[i]);
      }
    }
    else dxl_id[i] = 0;
  }

  dxl_comm_result = pbulkread->txPacket();
  if (dxl_comm_result != COMM_SUCCESS) packetHandler->printTxRxResult(dxl_comm_result);

}


void DynamixelController::bulk_rxpacket(int8_t dxl_id[], uint16_t start_addr_read, uint8_t length)
{


  bool dxl_addparam_result = false;
  bool dxl_getdata_result = false;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = pbulkread->rxPacket();
  if (dxl_comm_result != COMM_SUCCESS) packetHandler->printTxRxResult(dxl_comm_result);

  for(int i=1;i<all_DXL;i++)
  {

    if(dxl_id[i] != 0)
    {

      dxl_getdata_result = pbulkread->isAvailable(dxl_id[i], start_addr_read, length);
      if (dxl_getdata_result != true)
      {
        fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed", dxl_id[i]);
      }

      real_speed[i]    =  pbulkread->getData(dxl_id[i], start_addr_read, 4);// get velocity data
      real_position[i] =  pbulkread->getData(dxl_id[i], start_addr_read+4, 4);//get position data

      if(real_speed[i]>1024)
      {
        real_speed[i] = ~real_speed[i];
      }
      else  real_speed[i] = real_speed[i]*1;

      ROS_INFO("[ID: %d]position   %d   ::  speed    %d", dxl_id[i], real_position[i], real_speed[i]);
    }
    else dxl_id[i] = 0;


  }
  pbulkread->clearParam();

}
void DynamixelController::sync_Write_torque(int8_t dxl_id[], bool torque)
{
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler,ADDR_MX_TORQUE_ENABLE , 1);

  bool dxl_addparam_result = false;
  int dxl_comm_result = COMM_TX_FAIL;
  uint8_t param_goal_position[8];



  for(int i=1; i<all_DXL; i++)
  {
    param_goal_position[0]= DXL_LOBYTE(DXL_LOWORD(torque));

    if(dxl_id[i] != 0)
    {
      dxl_addparam_result = groupSyncWrite.addParam(dxl_id[i], param_goal_position);

      if(dxl_addparam_result != true)
      {
        fprintf(stderr,"[ID: %d] groupSyncWrite addparam failed\n", i);
      }
    }
    else dxl_id[i] = 0;

  }


  dxl_comm_result = groupSyncWrite.txPacket();
  if(dxl_comm_result != COMM_SUCCESS) packetHandler->printTxRxResult(dxl_comm_result);

}




