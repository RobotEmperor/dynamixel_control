#ifdef __linux__
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include "std_msgs/Bool.h"


#include "dynamixel_sdk.h"                                 // Uses Dynamixel SDK library

// Control table address

#define ADDR_MX_DRIVE_MODE            10
#define ADDR_MX_OPERATING_MODE        11
#define ADDR_MX_MOVING_THRESHOLD      24
#define ADDR_MX_TEMPERATURE_LIMIT     31
#define ADDR_MX_MAX_VOLTAGE_LIMIT     32
#define ADDR_MX_MIN_VOLTAGE_LIMIT     34
#define ADDR_MX_PWM_LIMIT             36
#define ADDR_MX_CURRENT_LIMIT         38
#define ADDR_MX_ACCELERATION_LIMIT    40
#define ADDR_MX_VELOCITY_LIMIT        44
#define ADDR_MX_MAX_POSITION_LIMIT    48
#define ADDR_MX_MIN_POSITION_LIMIT    52




#define ADDR_MX_TORQUE_ENABLE           64                  // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_PWM                100
#define ADDR_MX_GOAL_CURRENT            102
#define ADDR_MX_GOAL_VELOCITY           104
#define ADDR_MX_PROFILE_ACCELERATION    108
#define ADDR_MX_PROFILE_VELOCITY        112
#define ADDR_MX_GOAL_POSITION           116

#define ADDR_MX_MOVING                  122



#define ADDR_MX_PRESENT_CURRENT         126
#define ADDR_MX_PRESENT_VELOCITY        128
#define ADDR_MX_PRESENT_POSITION        132            
#define ADDR_MX_PRESENT_TEMPERATURE     146


#define ADDR_INDIRECT_1                 168 
#define ADDR_INDIRECT_2                 170  
#define ADDR_INDIRECT_3                 172
#define ADDR_INDIRECT_4                 174
//////////////////////////////////////////// 
#define ADDR_INDIRECT_DATA_1            224 
//////////////////////////////////////////// 

#define ADDR_INDIRECT_5                 176
#define ADDR_INDIRECT_6                 178
#define ADDR_INDIRECT_7                 180
#define ADDR_INDIRECT_8                 182
//////////////////////////////////////////// 




// Data Byte Length
#define LEN_MX_GOAL_POSITION            4
#define LEN_MX_PRESENT_POSITION         4

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define ID1                         1                   // Dynamixel#1 ID: 1
#define ID2                         2                   // Dynamixel#2 ID: 2

#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
// ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

#define TORQUE_ON                   1                   // Value for enabling the torque
#define TORQUE_OFF                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      0                 // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      4095                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b
#define all_DXL                         15 // real the number of dynamixel + 1



namespace dynamixel_controller 
{
class DynamixelController{

private:
  dynamixel::PortHandler *portHandler;
  dynamixel::PacketHandler *packetHandler;
  dynamixel::GroupBulkRead *pbulkread;


public:
  DynamixelController();

  int initDynamixel();
  void closeDynamixel(int8_t dxl_id[]);

  void setTorque(int8_t dxl_id[], bool onoff);

  void readResister(uint8_t id, uint16_t addr, uint16_t length);
  void writeResister(uint8_t id, uint16_t addr, uint16_t length, int32_t value);

  void sync_Write(int8_t dxl_id[], int position[], int speed[]);
  void sync_Read(int8_t dxl_id[], uint16_t start_addr, uint8_t length);
  void bulk_txpacket(int8_t dxl_id[], uint16_t start_addr_read, uint8_t length);
  void bulk_rxpacket(int8_t dxl_id[], uint16_t start_addr_read, uint8_t length);
  void sync_Write_torque(int8_t dxl_id[], bool torque);


  int position[all_DXL];
  int speed[all_DXL];

  uint16_t real_position[all_DXL];
  uint16_t real_speed[all_DXL];
};
}
