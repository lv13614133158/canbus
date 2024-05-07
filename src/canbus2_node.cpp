#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include "canbus/controlcan.h"
#include "std_msgs/msg/int32.hpp"
#include <ctime>
#include <cstdlib>
#include "unistd.h"
#include <mutex>

//#include "builtin_interfaces/msg/Time.idl"

#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/gear_command.hpp"
#include "tier4_vehicle_msgs/msg/vehicle_emergency_stamped.hpp"
#include "autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp"
#include "tier4_vehicle_msgs/msg/actuation_command_stamped.hpp"

VCI_BOARD_INFO pInfo;//用来获取设备信息。
int count=0;//数据列表中，用来存储列表序号。
int count1=0;
VCI_BOARD_INFO pInfo1 [50];
int num=0;
std::mutex mtx_130;
std::mutex mtx_131;
std::mutex mtx_132;
std::mutex mtx_140;

VCI_CAN_OBJ canbus_Ctrl_130[1];
VCI_CAN_OBJ canbus_Ctrl_131[1];
VCI_CAN_OBJ canbus_Ctrl_132[1];
VCI_CAN_OBJ canbus_Ctrl_140[1];
struct Ctrl_130 {
    uint64_t DriverEnCtrl : 1;
	uint64_t kong1 :1;
	uint64_t DriverModeCtrl : 2;
	uint64_t GearCtrl : 4;
	uint64_t SpeedCtrl : 16;
	uint64_t ThrottlePdlTarget : 10;
	uint64_t kong2 :14;
	uint64_t DriveLifeSig : 4;
	uint64_t kong3 : 4;
	uint64_t CheckSum_130 :8;
};
struct Ctrl_131 {
    uint64_t BrakeEn : 1;
	uint64_t kong :3; 
	uint64_t AebCtrl : 1;
	uint64_t kong2 :3;
	uint64_t BrakePdlTarget : 10;
	uint64_t kong3 : 6 ;
	uint64_t EpbCtrl :2;
	uint64_t kong4 : 22;
	uint64_t LifeSig : 4;
	uint64_t kong5 : 4;
	uint64_t CheckSum_131  : 8;

};

struct Ctrl_132 {
    uint64_t SteerEnCtrl : 1;
	uint64_t kong :3;
	uint64_t SteerModeCtrl : 4;
	uint64_t SteerAngleTarget : 16;
	uint64_t SteerAngleRearTarget : 16 ;
	uint64_t SteerAngleSpeedCtrl :8;
	uint64_t kong1 :8;
	uint64_t CheckSum_132 : 8;

};

struct Ctrl_140 {
    uint64_t WorkEnableCtrl : 1;
	uint64_t SweepModeCtrl :2;
	uint64_t FanModeCtrl : 2;
	uint64_t VehicleCtrlModeCtrl : 2;
	uint64_t EnableCtrl : 1;
	uint64_t VehiclePosLampCtrl : 1;
	uint64_t VehicleHeadLampCtrl : 1;
	uint64_t VehicleLeftLampCtrl: 1;
	uint64_t VehicleRightLampCtrl:1;
	uint64_t VehicleHighBeamCtrl:1;
	uint64_t VehicleFogLampCtrl:1;
	uint64_t VehicleHazardWarLampCtrl:1;

	uint64_t VehicleFrontHornCtrl :1;
	uint64_t VehicleWorkLampCtrl :1;
	uint64_t VehicleWiperCtrl :1;
	uint64_t GarbageWashingCtrl :1;
 	uint64_t UnloadingCtrl :1;
	uint64_t WashGunCtrl :1;
	uint64_t BackDoorCtrl :2;
	uint64_t GarbageCtrl:2;
	uint64_t ModeCtrl :2;
	uint64_t SweepCtrl :2;
	uint64_t GreenLightCtrl :1;
	uint64_t YellowLightCtrl :1;
	uint64_t RedLightCtrl :1;
	uint64_t ArrowLightCtrl:1;
	uint64_t SweepWaterSprayCtrl :1;
	uint64_t AlarmBuzzerCtrl:1;
	uint64_t DustVibrtionCtrl:1;
	uint64_t DryWetModeCtrl:1;
	uint64_t NozzleCtrl:1;
	uint64_t SweepDebugModeCtrl :1;
	uint64_t FanDebugModeCtrl :1 ;
	uint64_t Life1 : 8;

};

struct State_530{
	uint64_t ChassisDriverEnSta:1;
	uint64_t ChassisDiverSlopover:1;
	uint64_t ChassisDriverModeSta:2;
	uint64_t ChassisGearFb:2;
	uint64_t kong1:2;
	uint64_t ChassisSpeedFb:16;
	uint64_t ChassisThrottlePaldFb:10;
};

struct State_540 {
    uint64_t WorkEnableFb : 1;
    uint64_t SweepModeFb :2;
    uint64_t FanModeFb :2;
    uint64_t VehicleCtrlModeFb : 2;
    uint64_t CtrlEnableFb : 1;
    uint64_t VehiclePosLampFb : 1;
    uint64_t VehicleHeadLampFb :1;
    uint64_t VehicleLeftLampFb :1;
    uint64_t VehicleRightLampFb :1;
    uint64_t VehicleHighBeamFb:1;
    uint64_t VehicleFogLampFb:1;
    uint64_t VehicleHazardWarLampFb :1;
    uint64_t VehicleFrontHornFb :1;
    uint64_t VehicleWorkLampFb:1;
    uint64_t VehicleWiperFb :2;
    uint64_t GarbageONFb :1;
    uint64_t GarbageWashingFb :1;
    uint64_t WashGunFb :1;
    uint64_t BackdoorInplace :1;
    uint64_t kong_1: 1;
    uint64_t BackdoorFb :2;
    uint64_t GarbageFb :2;
    uint64_t KeyStatusFb :2;
    uint64_t ControlModeFb :2;
    uint64_t VehicleEmergency :1;
    uint64_t NozzleStatusFb :1;
    uint64_t SweepWaterSprayFb :1;
    uint64_t ArrowLightFb:1;
    uint64_t kong_2:2;
    uint64_t AlarmBuzzerFb : 1;
    uint64_t WaterStatusFb :1; 
    uint64_t FaultStatus :8; 
    uint64_t DustVibrtionStatusFb :1;
    uint64_t DryWetModeStatusFb :1;
    uint64_t UnloadingStatusFb :1;
    uint64_t SweepDebugModeFb:1;
    uint64_t FanDebugModeFb:1;
    uint64_t DustbinOverflowStatusFb :1;
    uint64_t kong_3 :2;
    uint64_t Life0 :8;

};


struct Ctrl_130 ctrl_130;
struct Ctrl_131 ctrl_131;
struct Ctrl_132 ctrl_132;
struct Ctrl_140 ctrl_140;
struct State_530 state_530;
struct State_540 state_540;

void can_init(); 
void canbus_write_msg_init();

 //接收线程。
void *receive_func(void* param) 
{
	int reclen=0;
	VCI_CAN_OBJ rec[3000];//接收缓存，设为3000为佳。
	int i,j;
	
	int *run=(int*)param;//线程启动，退出控制。
	//unsigned char buff[8]={0};
	unsigned char *buff_ptr;
	
	while((*run)&0x0f)
	{
		if((reclen=VCI_Receive(VCI_USBCAN2,0,0,rec,3000,100))>0)//调用接收函数，如果有数据，进行数据处理显示。
		{

			
			for(j=0;j<reclen;j++)
			{


				//rec[j].ID;//ID
				switch(rec[j].ID)
				{

				case 0x540:
					buff_ptr=(unsigned char *)&state_540;
					for(i = 0; i < rec[j].DataLen; i++)
					{
						*buff_ptr=rec[j].Data[i];
						buff_ptr++;
					}

					break;
				case 0x530:
					buff_ptr=(unsigned char *)&state_530;
					for(i = 0; i < rec[j].DataLen; i++)
					{
						*buff_ptr=rec[j].Data[i];
						buff_ptr++;
					}

					break;

				 default:
		            std::cout << "no find id." << rec[j].ID<<std::endl;
		            break;
				}
			}
			
		}	
	}
	printf("run thread exit\n");//退出接收线程	
	pthread_exit(0);
}

void *send_func(void* param) 
{

    VCI_CAN_OBJ send[1];
    int *run=(int*)param;//线程启动，退出控制。
	while((*run)&0x0f)
	{
		mtx_130.lock();
        send[0]=canbus_Ctrl_130[0];
        mtx_130.unlock();

		if(VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) != 1)
		{
			printf("VCI_Transmit 0x130 error\n");
		}
		mtx_131.lock();
		send[0]=canbus_Ctrl_131[0];
		mtx_131.unlock();
		if(VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) != 1)
		{
			printf("VCI_Transmit 0x131 error\n");
		}
		
    	mtx_132.lock();
        send[0]=canbus_Ctrl_132[0];
        mtx_132.unlock();
		if(VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) != 1)
		{
			printf("VCI_Transmit 0x132 error\n");
		}
		

 		mtx_140.lock();
        send[0]=canbus_Ctrl_140[0];
        mtx_140.unlock();
		if(VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) != 1)
		{
			printf("VCI_Transmit 0x133 error\n");
		}
	

		usleep(20000);
	}
    usleep(100000);//延时100ms。
	VCI_ResetCAN(VCI_USBCAN2, 0, 0);//复位CAN1通道。
	usleep(100000);//延时100ms。
	VCI_CloseDevice(VCI_USBCAN2,0);//关闭设备。
    pthread_exit(0);
}



void send_132_SteerAngleTarget_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "I callback  data : '%d'", msg->data);
	int data=msg->data;
	mtx_132.lock();
	ctrl_132.SteerAngleTarget=data;
	unsigned char * buf=(unsigned char *)&ctrl_132;

	for(int i=0 ;i<8;i++)
	{
		canbus_Ctrl_132[0].Data[i]=*buf;
		//printf("data= %d\n",canbus_Ctrl_133[0].Data[i]);
		buf++;
	}
	mtx_132.unlock();

}


void Control_cmd_callback(const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg)
{

	mtx_130.lock();
	ctrl_130.SpeedCtrl=msg->longitudinal.speed*10;//*100 ；分辨率100

	unsigned char * buf=(unsigned char *)&ctrl_130;
	printf("Control_cmd_callback data=   ");
	
	for(int i=0 ;i<8;i++)
	{
		canbus_Ctrl_130[0].Data[i]=*buf;
		printf(" %d ",canbus_Ctrl_130[0].Data[i]);

		buf++;
	}
	
	printf("\n ");
	mtx_130.unlock();
}
void  Gear_cmd_callback(const autoware_auto_vehicle_msgs::msg::GearCommand::SharedPtr msg)
{
	unsigned int Gear_data=0;
	unsigned char * data_130;
	unsigned char * data_131;
	int msg_data=msg->command;

	// if(msg_data==GearCommand_Constants::NEUTRAL)Gear_data=2;//N 1
	// else if(msg_data==GearCommand_Constants::DRIVE)Gear_data=1;//D 2
	//  else if(msg_data==GearCommand_Constants::REVERSE)Gear_data=3;//R 20
	// else  if(msg_data==GearCommand_Constants::PARK)Gear_data=4;//P 22

	if(msg_data==1)Gear_data=2;//N 1
	else if(msg_data==2)Gear_data=1;//D 2
 	else if(msg_data==20)Gear_data=3;//R 20
	else  if(msg_data==22)Gear_data=4;//P 22



	if (Gear_data==state_530.ChassisGearFb)
	{
		return ;
	}

	if (Gear_data!=4)
	{
	mtx_130.lock();
	ctrl_130.GearCtrl=Gear_data;
	ctrl_131.BrakePdlTarget=30;//分辨率0.1
	ctrl_131.EpbCtrl=1;


	}else{
	ctrl_130.GearCtrl=2;
	ctrl_131.BrakePdlTarget=30;//分辨率0.1
	ctrl_131.EpbCtrl=1;	
	}

	//踩刹车
	mtx_131.lock();
	data_131=(unsigned char *)&ctrl_131;
	for(int i=0 ;i<8;i++)
	{
		canbus_Ctrl_131[0].Data[i]=*data_131++;

	}	
	mtx_131.unlock();
	usleep(2000);
	//间隔20毫秒
	//换档
	mtx_130.lock();
	data_130=(unsigned char *)&ctrl_130;
	for(int i=0 ;i<8;i++)
	{
		canbus_Ctrl_130[0].Data[i]=*data_130++;
	}	
	mtx_130.unlock();
	//松刹车
	mtx_131.lock();
	ctrl_131.BrakePdlTarget=0;
	data_131=(unsigned char *)&ctrl_131;
	for(int i=0 ;i<8;i++)
	{
		canbus_Ctrl_131[0].Data[i]=*data_131++;

	}	
	mtx_131.unlock();
}


// void Emergency_cmd_callback(const autoware_auto_vehicle_msgs::msg::VehicleEmergencyStamped::SharedPtr msg)
// {
// //builtin_interfaces/Time stamp
// //bool emergency
// }


void Turn_indicators_cmd_callback(const autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::SharedPtr msg)
{
//	ctrl_140.VehicleLeftLampCtrl=0;
//	ctrl_140.VehicleRightLampCtrl=0;
	int data=msg->command;
	unsigned char * data_140;
	//if (data==TurnIndicatorsCommand_Constants::NO_COMMAND)//0
	if (data==0)
	{
		//保持
	}//else if (data==TurnIndicatorsCommand_Constants::DISABLE)//1
	else if (data==1)
	{
		ctrl_140.VehicleLeftLampCtrl=0;
		ctrl_140.VehicleRightLampCtrl=0;
	}//else if(data==TurnIndicatorsCommand_Constants::ENABLE_LEFT)//2
	else if(data==2)
	{

		ctrl_140.VehicleLeftLampCtrl=1;
		ctrl_140.VehicleRightLampCtrl=0;
	}//else if (data==TurnIndicatorsCommand_Constants::ENABLE_RIGHT)//3
	else if (data==3)
	{
		ctrl_140.VehicleLeftLampCtrl=0;
		ctrl_140.VehicleRightLampCtrl=1;
	}

	data_140=(unsigned char *)&ctrl_140;
	mtx_140.lock();
	for(int i=0 ;i<8;i++)
	{
		canbus_Ctrl_140[0].Data[i]=*data_140++;

	}	
	mtx_140.unlock();
}

void Hazard_lights_cmd_callback(const autoware_auto_vehicle_msgs::msg::HazardLightsCommand::SharedPtr msg)
{
	//ctrl_140.VehicleHazardWarLampCtrl=0;
	int data=msg->command;
	unsigned char * data_140;
	//if (data==HazardLightsCommand_Constants::NO_COMMAND)//0
	if (data==0)//0
	{
		/* code */
	//}else if(data==HazardLightsCommand_Constants::NO_COMMAND)//1
	}else if(data==1)
	{
		ctrl_140.VehicleHazardWarLampCtrl=0;
	// }else if(data==HazardLightsCommand_Constants::NO_COMMAND)//2
	}else if(data==2)//2
	{
		ctrl_140.VehicleHazardWarLampCtrl=0;
	}
	data_140=(unsigned char *)&ctrl_140;
	mtx_140.lock();
	for(int i=0 ;i<8;i++)
	{
		canbus_Ctrl_140[0].Data[i]=*data_140++;

	}	
	mtx_140.unlock();
}


void Actuation_cmd_callback(const tier4_vehicle_msgs::msg::ActuationCommandStamped::SharedPtr msg)
{
	int data=msg->actuation.steer_cmd;
	mtx_132.lock();
	ctrl_132.SteerAngleTarget=data;
	unsigned char * buf=(unsigned char *)&ctrl_132;

	for(int i=0 ;i<8;i++)
	{
		canbus_Ctrl_132[0].Data[i]=*buf;
		//printf("data= %d\n",canbus_Ctrl_133[0].Data[i]);
		buf++;
	}
	mtx_132.unlock();
}

int main(int argc, char** argv)
{

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("canbus_node");

	printf(">>this is hello !\r\n");//指示程序已运行

	can_init();
	
	int m_run0=1;
    int m_run1=1;
	pthread_t threadid,threadid1;
    canbus_write_msg_init();
	pthread_create(&threadid,NULL,receive_func,&m_run0);	
    pthread_create(&threadid1,NULL,send_func,&m_run1);
  

	auto sub = node->create_subscription<std_msgs::msg::Int32>(
        "1132", 10, send_132_SteerAngleTarget_callback);


   auto sub_control_cmd = node->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>("/control/command/control_cmd",  1, Control_cmd_callback);//线速度 角速度
   auto sub_gear_cmd = node->create_subscription<autoware_auto_vehicle_msgs::msg::GearCommand>("/control/command/gear_cmd",  1, Gear_cmd_callback);//档位
   //auto sub_gate_mode = node->create_subscription<tier4_control_msgs::msg::GateMode>("/control/current_gate_mode",  1, Gate_mode_callback);//是否控制autoware
   //auto sub_Emergency_cmd = node->create_subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>("/control/command/emergency_cmd",  1, Emergency_cmd_callback);//紧急信息
   auto sub_Turn_indicators_cmd = node->create_subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>("/control/command/turn_indicators_cmd",  1, Turn_indicators_cmd_callback);//转向信号
   auto sub_Hazard_lights_cmd = node->create_subscription<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>("/control/command/hazard_lights_cmd",  1, Hazard_lights_cmd_callback);//危险转向灯
   auto sub_Actuation_cmd = node->create_subscription<tier4_vehicle_msgs::msg::ActuationCommandStamped>("/control/command/actuation_cmd",  1, Actuation_cmd_callback);//油门踏板，TYPE B 控制车辆

   // auto battery_charge_pub=node->create_publisher<tier4_vehicle_msgs::msg::BatteryStatus>("/vehicle/status/battery_charge", rclcpp::QoS{1});//电池信息
   // auto control_mode_pub=node->create_publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>("/vehicle/status/control_mode", rclcpp::QoS{1});//控制模式
   // auto gear_status_pub=node->create_publisher<autoware_auto_vehicle_msgs::msg::GearReport>("/vehicle/status/gear_status", rclcpp::QoS{1});//当前档位
   // auto hazard_lights_status_pub=node->create_publisher<autoware_auto_vehicle_msgs::msg::HazardLightsReport>("/vehicle/status/hazard_lights_status", rclcpp::QoS{1});//危险灯状态
   // auto turn_indicators_status_pub=node->create_publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>("/vehicle/status/turn_indicators_status", rclcpp::QoS{1});//转向状态
   // auto steering_status_pub=node->create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>("/vehicle/status/steering_status", rclcpp::QoS{1});//转向状态
   // auto velocity_Status_pub=node->create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>("/vehicle/status/velocity_Status", rclcpp::QoS{1});//速度状态
   rclcpp::spin(node);
   rclcpp::shutdown();
}

void can_init()
{

    num=VCI_FindUsbDevice2(pInfo1);

	printf(">>USBCAN DEVICE NUM:");printf("%d", num);printf(" PCS");printf("\n");

		for(int i=0;i<num;i++)
		{
		printf("Device:");printf("%d", i);printf("\n");
                printf(">>Get VCI_ReadBoardInfo success!\n");
		
		printf(">>Serial_Num:%c", pInfo1[i].str_Serial_Num[0]);
		printf("%c", pInfo1[i].str_Serial_Num[1]);
		printf("%c", pInfo1[i].str_Serial_Num[2]);
		printf("%c", pInfo1[i].str_Serial_Num[3]);
		printf("%c", pInfo1[i].str_Serial_Num[4]);
		printf("%c", pInfo1[i].str_Serial_Num[5]);
		printf("%c", pInfo1[i].str_Serial_Num[6]);
		printf("%c", pInfo1[i].str_Serial_Num[7]);
		printf("%c", pInfo1[i].str_Serial_Num[8]);
		printf("%c", pInfo1[i].str_Serial_Num[9]);
		printf("%c", pInfo1[i].str_Serial_Num[10]);
		printf("%c", pInfo1[i].str_Serial_Num[11]);
		printf("%c", pInfo1[i].str_Serial_Num[12]);
		printf("%c", pInfo1[i].str_Serial_Num[13]);
		printf("%c", pInfo1[i].str_Serial_Num[14]);
		printf("%c", pInfo1[i].str_Serial_Num[15]);
		printf("%c", pInfo1[i].str_Serial_Num[16]);
		printf("%c", pInfo1[i].str_Serial_Num[17]);
		printf("%c", pInfo1[i].str_Serial_Num[18]);
		printf("%c", pInfo1[i].str_Serial_Num[19]);printf("\n");

		printf(">>hw_Type:%c", pInfo1[i].str_hw_Type[0]);
		printf("%c", pInfo1[i].str_hw_Type[1]);
		printf("%c", pInfo1[i].str_hw_Type[2]);
		printf("%c", pInfo1[i].str_hw_Type[3]);
		printf("%c", pInfo1[i].str_hw_Type[4]);
		printf("%c", pInfo1[i].str_hw_Type[5]);
		printf("%c", pInfo1[i].str_hw_Type[6]);
		printf("%c", pInfo1[i].str_hw_Type[7]);
		printf("%c", pInfo1[i].str_hw_Type[8]);
		printf("%c", pInfo1[i].str_hw_Type[9]);printf("\n");	

		printf(">>Firmware Version:V");
		printf("%x", (pInfo1[i].fw_Version&0xF00)>>8);
		printf(".");
		printf("%x", (pInfo1[i].fw_Version&0xF0)>>4);
		printf("%x", pInfo1[i].fw_Version&0xF);
		printf("\n");
	}
	printf(">>\n");
	printf(">>\n");
	printf(">>\n");
	if(VCI_OpenDevice(VCI_USBCAN2,0,0)==1)//打开设备
	{
		printf(">>open deivce success!\n");//打开设备成功
	}else
	{
		printf(">>open deivce error!\n");
		exit(1);
	}
	if(VCI_ReadBoardInfo(VCI_USBCAN2,0,&pInfo)==1)//读取设备序列号、版本等信息。
	{
                printf(">>Get VCI_ReadBoardInfo success!\n");
		

		printf(">>Serial_Num:%c", pInfo.str_Serial_Num[0]);
		printf("%c", pInfo.str_Serial_Num[1]);
		printf("%c", pInfo.str_Serial_Num[2]);
		printf("%c", pInfo.str_Serial_Num[3]);
		printf("%c", pInfo.str_Serial_Num[4]);
		printf("%c", pInfo.str_Serial_Num[5]);
		printf("%c", pInfo.str_Serial_Num[6]);
		printf("%c", pInfo.str_Serial_Num[7]);
		printf("%c", pInfo.str_Serial_Num[8]);
		printf("%c", pInfo.str_Serial_Num[9]);
		printf("%c", pInfo.str_Serial_Num[10]);
		printf("%c", pInfo.str_Serial_Num[11]);
		printf("%c", pInfo.str_Serial_Num[12]);
		printf("%c", pInfo.str_Serial_Num[13]);
		printf("%c", pInfo.str_Serial_Num[14]);
		printf("%c", pInfo.str_Serial_Num[15]);
		printf("%c", pInfo.str_Serial_Num[16]);
		printf("%c", pInfo.str_Serial_Num[17]);
		printf("%c", pInfo.str_Serial_Num[18]);
		printf("%c", pInfo.str_Serial_Num[19]);printf("\n");

		printf(">>hw_Type:%c", pInfo.str_hw_Type[0]);
		printf("%c", pInfo.str_hw_Type[1]);
		printf("%c", pInfo.str_hw_Type[2]);
		printf("%c", pInfo.str_hw_Type[3]);
		printf("%c", pInfo.str_hw_Type[4]);
		printf("%c", pInfo.str_hw_Type[5]);
		printf("%c", pInfo.str_hw_Type[6]);
		printf("%c", pInfo.str_hw_Type[7]);
		printf("%c", pInfo.str_hw_Type[8]);
		printf("%c", pInfo.str_hw_Type[9]);printf("\n");

		printf(">>Firmware Version:V");
		printf("%x", (pInfo.fw_Version&0xF00)>>8);
		printf(".");
		printf("%x", (pInfo.fw_Version&0xF0)>>4);
		printf("%x", pInfo.fw_Version&0xF);
		printf("\n");	
	}else
	{
		printf(">>Get VCI_ReadBoardInfo error!\n");
		exit(1);
	}

	//初始化参数，严格参数二次开发函数库说明书。
	VCI_INIT_CONFIG config;
	config.AccCode=0;
	config.AccMask=0xFFFFFFFF;
	config.Filter=1;//接收所有帧
	config.Timing0=0x0;/*波特率125 Kbps  0x03  0x1C*/
	config.Timing1=0x1C;
	config.Mode=0;//正常模式		
	
	if(VCI_InitCAN(VCI_USBCAN2,0,0,&config)!=1)
	{
		printf(">>Init CAN1 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);
	}

	if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1)
	{
		printf(">>Start CAN1 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);

	}


}


void canbus_write_msg_init()
{
//模式控制
    canbus_Ctrl_130[0].ID = 0x130;  // CAN 帧 ID
	
	canbus_Ctrl_130[0].SendType=0;
	canbus_Ctrl_130[0].RemoteFlag=0;
	canbus_Ctrl_130[0].ExternFlag=0;
	canbus_Ctrl_130[0].DataLen=8;
    ctrl_130.DriverEnCtrl= 0x01;
    ctrl_130.DriverModeCtrl = 0x00;
    ctrl_130.GearCtrl = 0x01;
    ctrl_130.SpeedCtrl = 0x00;
    ctrl_130.ThrottlePdlTarget = 0x00;
    ctrl_130.DriveLifeSig = 0x00;
    ctrl_130.CheckSum_130 = 0x00;
	unsigned char * buf=(unsigned char *)&ctrl_130;
	for(int i=0 ;i<8;i++)
	{
		canbus_Ctrl_130[0].Data[i]=*buf++;
	}
    //制动控制
    canbus_Ctrl_131[0].ID = 0x131;  // CAN 帧 ID

	canbus_Ctrl_131[0].SendType=0;
	canbus_Ctrl_131[0].RemoteFlag=0;
	canbus_Ctrl_131[0].ExternFlag=0;
	canbus_Ctrl_131[0].DataLen=8;
	ctrl_131.BrakeEn= 0x01;
    ctrl_131.AebCtrl = 0x00;
    ctrl_131.BrakePdlTarget = 0x00;
    ctrl_131.EpbCtrl = 0x02;
    ctrl_131.LifeSig = 0x00;
    ctrl_131.CheckSum_131 = 0x00;

	buf=(unsigned char *)&ctrl_131;
	for(int i=0 ;i<8;i++)
	{
		canbus_Ctrl_131[0].Data[i]=*buf++;
	}


    //转向控制
    canbus_Ctrl_132[0].ID = 0x132;  // CAN 帧 ID

	canbus_Ctrl_132[0].SendType=0;
	canbus_Ctrl_132[0].RemoteFlag=0;
	canbus_Ctrl_132[0].ExternFlag=0;
	canbus_Ctrl_132[0].DataLen=8;

	ctrl_132.SteerEnCtrl= 0x01;
    ctrl_132.SteerModeCtrl = 0x00;
    ctrl_132.SteerAngleTarget = 0x00;
    ctrl_132.SteerAngleRearTarget = 0x00;
    ctrl_132.SteerAngleSpeedCtrl = 0x00;
    ctrl_132.CheckSum_132 = 0x00;

	buf=(unsigned char *)&ctrl_132;
	for(int i=0 ;i<8;i++)
	{
		canbus_Ctrl_132[0].Data[i]=*buf++;
	}

    //上装控制
    canbus_Ctrl_140[0].ID = 0x140;  // CAN 帧 ID
	canbus_Ctrl_140[0].SendType=0;
	canbus_Ctrl_140[0].RemoteFlag=0;
	canbus_Ctrl_140[0].ExternFlag=0;
	canbus_Ctrl_140[0].DataLen=8;

 ctrl_140.WorkEnableCtrl=0;
	ctrl_140.SweepModeCtrl=0;
	ctrl_140.FanModeCtrl=0;
	ctrl_140.VehicleCtrlModeCtrl=3;
	ctrl_140.EnableCtrl=0;
	ctrl_140.VehiclePosLampCtrl=0;
	ctrl_140.VehicleHeadLampCtrl=0;
	ctrl_140.VehicleLeftLampCtrl=0;
	ctrl_140.VehicleRightLampCtrl=0;
	ctrl_140.VehicleHighBeamCtrl=0;
	ctrl_140.VehicleFogLampCtrl=0;
	ctrl_140.VehicleHazardWarLampCtrl=0;
	ctrl_140.VehicleFrontHornCtrl=0;
	ctrl_140.VehicleWorkLampCtrl=0;
	ctrl_140.VehicleWiperCtrl=0;
	ctrl_140.GarbageWashingCtrl=0;
 	ctrl_140.UnloadingCtrl=0;
	ctrl_140.WashGunCtrl=0;
	ctrl_140.BackDoorCtrl=0;
	ctrl_140.GarbageCtrl=0;
	ctrl_140.ModeCtrl=0;
	ctrl_140.SweepCtrl=0;
	ctrl_140.GreenLightCtrl=0;
	ctrl_140.YellowLightCtrl=0;
	ctrl_140.RedLightCtrl=0;
	ctrl_140.ArrowLightCtrl=0;
	ctrl_140.SweepWaterSprayCtrl=0;
	ctrl_140.AlarmBuzzerCtrl=0;
	ctrl_140.DustVibrtionCtrl=0;
	ctrl_140.DryWetModeCtrl=0;
	ctrl_140.NozzleCtrl=0;
	ctrl_140.SweepDebugModeCtrl=0;
	ctrl_140.FanDebugModeCtrl=0;
	ctrl_140.Life1=0;

	buf=(unsigned char *)&ctrl_140;
	for(int i=0 ;i<8;i++)
	{
		canbus_Ctrl_140[0].Data[i]=*buf++;;

	}

}