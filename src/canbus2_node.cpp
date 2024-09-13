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

#include "autoware_control_msgs/msg/control.hpp"

#include "tier4_vehicle_msgs/msg/vehicle_emergency_stamped.hpp"
#include "tier4_vehicle_msgs/msg/battery_status.hpp"

#include "tier4_vehicle_msgs/msg/actuation_command_stamped.hpp"
#include "autoware_vehicle_msgs/msg/gear_command.hpp"
#include "autoware_vehicle_msgs/msg/velocity_report.hpp"
#include "autoware_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_vehicle_msgs/msg/control_mode_report.hpp"
#include "autoware_vehicle_msgs/msg/gear_report.hpp"
#include "autoware_vehicle_msgs/msg/hazard_lights_report.hpp"
#include "autoware_vehicle_msgs/msg/turn_indicators_report.hpp"
#include "autoware_vehicle_msgs/msg/turn_indicators_command.hpp"
#include "autoware_vehicle_msgs/msg/hazard_lights_command.hpp"
#include "autoware_adapi_v1_msgs/msg/operation_mode_state.hpp"

#include <visualization_msgs/msg/marker.hpp>
#include "mycan_msgs/msg/hwcan.hpp"
#include "ars408_msg/msg/ars408.hpp"
#include "ars408_msg/msg/my_objects.hpp"
#include "std_msgs/msg/header.hpp"
rclcpp::Publisher<tier4_vehicle_msgs::msg::BatteryStatus>::SharedPtr battery_charge_pub;
rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr control_mode_pub;
rclcpp::Publisher<autoware_vehicle_msgs::msg::GearReport>::SharedPtr gear_status_pub;
rclcpp::Publisher<autoware_vehicle_msgs::msg::HazardLightsReport>::SharedPtr hazard_lights_status_pub;
rclcpp::Publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr turn_indicators_status_pub;
rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_status_pub;
rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_Status_pub;
rclcpp::Publisher<mycan_msgs::msg::Hwcan>::SharedPtr hw_can_msgs_pub;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
rclcpp::Publisher<ars408_msg::msg::ARS408>::SharedPtr ars408_pub;
VCI_BOARD_INFO pInfo;//用来获取设备信息。
int count=0;//数据列表中，用来存储列表序号。
int count1=0;
VCI_BOARD_INFO pInfo1 [50];

int num=0;

int i=0;
std::mutex mtx_130;
std::mutex mtx_131;
std::mutex mtx_132;
std::mutex mtx_140;
std::mutex mtx_ptr;
std::mutex mtx_408;

VCI_CAN_OBJ canbus_Ctrl_130[1];
VCI_CAN_OBJ canbus_Ctrl_131[1];
VCI_CAN_OBJ canbus_Ctrl_132[1];
VCI_CAN_OBJ canbus_Ctrl_140[1];
struct Ctrl_130 {
    uint64_t DriverEnCtrl : 1;        //加速使能
	uint64_t kong1 :1;                //占位
	uint64_t DriverModeCtrl : 2;      //驱动模式控制 0速度 1油门
 	uint64_t GearCtrl : 4;            //档位

	uint64_t SpeedCtrl : 16;          //速度控制

	uint64_t ThrottlePdlTarget : 10;  //油门控制
	uint64_t kong2 :14;               //占位

	uint64_t DriveLifeSig : 4;        //循环计数
	uint64_t kong3 : 4;               //占位

	uint64_t CheckSum_130 :8;         //校验
};
struct Ctrl_131 {
    uint64_t BrakeEn : 1;             //刹车使能
	uint64_t kong :3;                 //占位
	uint64_t AebCtrl : 1;             //aeb使能
	uint64_t kong2 :3;                //占位
	uint64_t BrakePdlTarget : 10;     //刹车控制0～100 0.1分辨率
	uint64_t kong3 : 6 ;              //占位
	uint64_t EpbCtrl :2;              //驻车控制 0默认 1刹车 2 释放
	uint64_t kong4 : 22;              //占位
	uint64_t LifeSig : 4;             //循环计数
	uint64_t kong5 : 4;               //占位
	uint64_t CheckSum_131  : 8;       //校验

};

struct Ctrl_132 {
    uint64_t SteerEnCtrl : 1;         //转向使能
	uint64_t kong :3;                 //占位
	uint64_t SteerModeCtrl : 4;       //转向模式控制
	uint64_t SteerAngleTarget : 16;   //转向控制前
	uint64_t SteerAngleRearTarget:16; //转向控制后
	uint64_t SteerAngleSpeedCtrl :8;  //方向盘角速度控制
	uint64_t kong1 :8;                //占位
	uint64_t CheckSum_132 : 8;        //校验

};
struct Ctrl_133 {
    uint64_t kong : 24;//占位
	uint64_t ChassisSpeedLimiteMode :3;//限速控制  0:default  1:limit
	uint64_t ChassisSpeedLimiteVal : 4;//速度限制值  1-20 m/s
	uint64_t CheckSumEn : 16;//校验模式使能(预留)
};
struct Ctrl_13A {
	uint64_t kong;

};

struct Ctrl_140 {
    uint64_t WorkEnableCtrl : 1;//上装清扫使能
	uint64_t SweepModeCtrl :2;//扫盘模式控制
	uint64_t FanModeCtrl : 2;//风机模式控制
	uint64_t VehicleCtrlModeCtrl : 2;//车辆模式控制
	uint64_t EnableCtrl : 1;//上装控制使能

	uint64_t VehiclePosLampCtrl : 1;//位置灯
	uint64_t VehicleHeadLampCtrl : 1;//近光灯
	uint64_t VehicleLeftLampCtrl: 1;//左转向灯
	uint64_t VehicleRightLampCtrl:1;//右转向灯
	uint64_t VehicleHighBeamCtrl:1;//远光灯
	uint64_t VehicleFogLampCtrl:1;//雾灯
	uint64_t VehicleHazardWarLampCtrl:1;//危险警示灯
	uint64_t VehicleFrontHornCtrl :1;//前喇叭

	uint64_t VehicleWorkLampCtrl :1;//作业警示灯控制
	uint64_t VehicleWiperCtrl :2;//雨刷
	uint64_t GarbageWashingCtrl :1;//上装清洗控制
 	uint64_t UnloadingCtrl :1;//上装卸料控制
	uint64_t WashGunCtrl :1;//上装洗枪控制
	uint64_t kong1: 2;//

	uint64_t BackDoorCtrl :2;//上装后门控制
	uint64_t GarbageCtrl:2;//上装箱体控制
	uint64_t ModeCtrl :2;//底盘模式
	uint64_t SweepCtrl :2;//上装扫盘

	uint64_t GreenLightCtrl :1;//绿色状态灯
	uint64_t YellowLightCtrl :1;//黄色状态灯
	uint64_t RedLightCtrl :1;//红色状态灯
	uint64_t ArrowLightCtrl:1;//箭头灯
	uint64_t kong2: 1; //
	uint64_t SweepWaterSprayCtrl:1;//扫盘喷水
	uint64_t AlarmBuzzerCtrl:1;//报警蜂鸣器
	uint64_t DustVibrtionCtrl:1;//机械振尘

	uint64_t DryWetModeCtrl:1;//干湿扫模式
	uint64_t NozzleCtrl:1;//上装吸嘴挡板控制
	uint64_t SweepDebugModeCtrl :1;//上装扫盘调试模式
	uint64_t FanDebugModeCtrl :1 ;//上装风机调试模式
	uint64_t kong3: 4 ;//

	uint64_t kong4: 8 ;//

	uint64_t Life1 : 8;//计数器

};

struct State_530{
	uint64_t ChassisDriverEnSta:1;//驱动使能状态
	uint64_t ChassisDiverSlopover:1;//驱动控制越界提醒
	uint64_t ChassisDriverModeSta:2;//驱动模式反馈
	uint64_t ChassisGearFb:2;//档位反馈 1d 2n 3r
	uint64_t kong1:2;//占位

	uint64_t ChassisSpeedFb:16;//车速反馈     0.01

	uint64_t ChassisThrottlePaldFb:10;//油门请求反馈    0.1
 };

struct State_531{
	uint64_t ChassisBrakeEnSta:1;//制动使能状态
	uint64_t VehicleBrakeLampFb:1;//制动灯状态反馈
	uint64_t ChassisEpbFb:2;//驻车状态
	uint64_t kong:4;//占位

	uint64_t ChassisBrakePedalValFb:10;//制动踏板踩下实际反馈  0.1
	uint64_t kong1:6;//占位

	uint64_t ChassisBrakePressureFb:8;//制动压力实际反馈
};

struct State_532{
	uint64_t ChassisSteerEnSta:1;//转向使能状态
	uint64_t ChassisSteerSlopover:1;//转向控制越界提醒
	uint64_t ChassisSteerModeFb:4;//转向模式反馈
	uint64_t kong:2;//占位

	uint64_t ChassisSteerAngleFb:16;//前转向方向盘转角反馈
	uint64_t ChassisSteerAngleRearFb:16;//后转向方向盘转角反馈
	uint64_t ChassisSteerAngleSpeedFb:8;//设置的转向转角速度反馈     2
};

struct State_534{
	uint64_t DrivingModeFb:2;//驾驶模式反馈
	uint64_t ChassisPowerStaFb:2;//车辆上电状态反馈
	uint64_t ChassisPowerDcSta:2;//DC工作状态
	uint64_t kong:2;

	uint64_t ChassisSpeedLimitedModeFb:1;//车辆限速状态
	uint64_t kong1:7;

	uint64_t ChassisSpeedLimitedValFb:16;//车辆限速值反馈      0.1
	uint64_t ChassisLowPowerVoltSta:8;//低压蓄电池电压         0.1

	uint64_t ChassisEStopStaFb:4;//紧急停车状态反馈
	uint64_t CrashFrontSta:1;  //车辆前碰撞传感器反馈
	uint64_t CrashRearSta:1;  //车辆后碰撞传感器反馈
	uint64_t kong2:2;

	uint64_t Life:4;  //VCU循环计数
	uint64_t kong3:4;

	uint64_t CheckSum:8;  //校验

};

struct State_535{
	uint64_t ChassisBmsReserved:4;//预留
	uint64_t ChassisPowerChargeSta:2;//车辆充电状态
	uint64_t ChassisPowerChargeSockSta:1;//充电枪连接状态
	uint64_t kong:1;

	uint64_t ChassisPowerSocFb:8;//车辆动力电池电量
	uint64_t ChassisPowerVoltFb:16;//车辆动力电池电压    分辨率0.1
	uint64_t ChassisPowerCurrFb:16;//车辆动力电池电流    分辨率0.1
	uint64_t ChassisBmsMaxTemp:8;//BMS最高单体温度
	uint64_t ChassisBmsReserved1:8; //预留

};

struct State_539{
	uint64_t ChassisWheelRpmLf:16;//左前轮转速
	uint64_t ChassisWheelRpmRf:16;//右前轮转速
	uint64_t ChassisWheelRpmLr:16;//左后轮转速
	uint64_t ChassisWheelRpmRr:16;//右后轮转速

};

struct State_53A{
	uint64_t VehicleODO:24;//总里程
	uint64_t VehicleTrip:24;//单次里程(每次下电清零) 分辨率0.01
	
};
struct State_53B{
	uint64_t CAN_TimeoutFb:8;//总里程
	uint64_t VehicleTrip:16;//单次里程(每次下电清零) 分辨率0.01
};
struct State_540 {
    uint64_t WorkEnableFb : 1;			//上装工作使能状态反馈
    uint64_t SweepModeFb :2;			//扫盘模式状态反馈
    uint64_t FanModeFb :2;				//风机模式状态反馈
    uint64_t VehicleCtrlModeFb : 2;		//车辆控制模式状态反馈
    uint64_t CtrlEnableFb : 1;          //上装控制使能反馈

    uint64_t VehiclePosLampFb : 1;      // 位置灯状态反馈
    uint64_t VehicleHeadLampFb :1;      //近光灯状态反馈
    uint64_t VehicleLeftLampFb :1;      //左转向灯状态反馈
    uint64_t VehicleRightLampFb :1;     //右转向灯状态反馈
    uint64_t VehicleHighBeamFb:1;       //远光灯状态反馈
    uint64_t VehicleFogLampFb:1;        //雾灯状态反馈
    uint64_t VehicleHazardWarLampFb :1; //危险警示灯开关状态
    uint64_t VehicleFrontHornFb :1;     //前喇叭状态反馈

    uint64_t VehicleWorkLampFb:1;       //作业警示灯状态反馈
    uint64_t VehicleWiperFb :2;         //雨刮状态反馈
    uint64_t GarbageONFb :1;            //上装箱体下降到位状态反馈
    uint64_t GarbageWashingFb :1;       //上装箱体清洗状态
    uint64_t WashGunFb :1;              //上装洗车枪状态反馈
    uint64_t BackdoorInplace :1;        //上装后门关闭到位状态反馈
    uint64_t kong_1: 1;                 //占位 

    uint64_t BackdoorFb :2;             //上装后门状态反馈
    uint64_t GarbageFb :2;              //上装箱体状态反馈
    uint64_t KeyStatusFb :2;            //车辆钥匙开关状态反馈
    uint64_t ControlModeFb :2;          //底盘控制模式反馈

    uint64_t VehicleEmergency :1;       //整车急停状态反馈
    uint64_t NozzleStatusFb :1;         //上装吸嘴挡板状态反馈
    uint64_t SweepWaterSprayFb :1;      //上装扫盘喷水状态反馈
    uint64_t ArrowLightFb:1;            //箭头灯状态反馈
    uint64_t kong_2:2;                  //占位 
    uint64_t AlarmBuzzerFb : 1;         //报警蜂鸣器状态反馈
    uint64_t WaterStatusFb :1;          //清水箱水位状态反馈

    uint64_t FaultStatus :8;            //上装故障状态

    uint64_t DustVibrtionStatusFb :1;   //机械振尘状态反馈
    uint64_t DryWetModeStatusFb :1;     //干湿扫模式状态反馈
    uint64_t UnloadingStatusFb :1;      //上装一键卸料状态反馈
    uint64_t SweepDebugModeFb:1;        //上装扫盘调试模式状态反馈
    uint64_t FanDebugModeFb:1;          //上装风机调试模式状态反馈
    uint64_t DustbinOverflowStatusFb :1;//上装垃圾箱满溢状态反馈
    uint64_t kong_3 :2;                 //占位

    uint64_t Life0 :8;                  //福龙马计数器0

};
struct State_541{
	uint64_t TotalMileage:16;//总里程
	uint64_t SubtotalMileage:16;//小计里程
	uint64_t VIN_1:8;
	uint64_t VIN_2:8;
	uint64_t VIN_3:8;
	uint64_t VIN_4:8;

};

struct State_542{
	uint64_t VIN_5:8;
	uint64_t VIN_6:8;
	uint64_t VIN_7:8;
	uint64_t VIN_8:8;
	uint64_t VIN_9:8;
	uint64_t VIN_10:8;
	uint64_t VIN_11:8;
};
struct State_543{
	uint64_t FaultDiagnosis:8;
	uint64_t McuEableSta:1;
	uint64_t DCDCEableSta:1;
	uint64_t ReadySta:1;
	uint64_t BatConSta:1;
	uint64_t kong:4;
	uint64_t EPSTorqueFb:8;
	uint64_t ModeChangeFb:8;
	uint64_t Bty_SystemFault:16;
	uint64_t MCU_SystemFault:8;
	uint64_t Life3:8;

};
struct State_548{
	uint64_t LeftFrontTirePressure:8;//左前轮胎压
	uint64_t RightFrontTirePressure:8;//右前轮胎压
	uint64_t RightRearTirePressure:8;//右后轮胎压
	uint64_t LeftRearTirePressure:8;//左后轮胎压
	uint64_t WaterlevelRatio:8;//清水箱水量反馈
	uint64_t Life8:8;//
};

struct State_60A{
    uint64_t Object_NofObjects:8;
    uint64_t Object_MeasCounter1:8;
    uint64_t Object_MeasCounter2:8;
    uint64_t Reserved:4;
    uint64_t Object_InterfaceVersion:4;
  };
struct State_60B{
    uint64_t Object_ID:8;
    uint64_t Object_DistLong1:8;
    uint64_t Object_DistLat1:3;
    uint64_t Object_DistLong2:5;
    uint64_t Object_DistLat2:8;
    uint64_t Object_VrelLong1:8;
    uint64_t Object_VrelLat1:6;
    uint64_t Object_VrelLong2:2;
    uint64_t Object_DynProp:3;
    uint64_t Reserved:2;
    uint64_t Object_VrelLat2:3;
    uint64_t Object_RCS:8;
  };
struct State_60C{
    uint64_t Object_ID:8;
    uint64_t Obj_DistLat_rms1:3;
    uint64_t Obj_DistLong_rms:5;
    uint64_t Obj_VrelLat_rms1:1;
    uint64_t Obj_VrelLong_rms:5;
    uint64_t Obj_DistLat_rms2:2;
    uint64_t Obj_ArelLong_rms1:4;
    uint64_t Obj_VrelLat_rms2:4;
    uint64_t Obj_Orientation_rms1:2;
    uint64_t Obj_ArelLat_rms:5;
    uint64_t Obj_ArelLong_rms2:1;
    uint64_t Reserved1:5;
    uint64_t Obj_Orientation_rms2:3;
    uint64_t Reserved2:2;
    uint64_t Obj_MeasState:3;
    uint64_t Obj_ProbOfExist:3;
  };
struct State_60D{
    uint64_t Object_ID:8;
    uint64_t Object_ArelLong1:8;
    uint64_t Object_ArelLat1:5;
    uint64_t Object_ArelLong2:3;
    uint64_t Object_Class:3;
    uint64_t Reserved:1;
    uint64_t Object_ArelLat2:4;
    uint64_t Object_OrientationAngle1:8;
    uint64_t Reserved2:6;
    uint64_t Object_OrientationAngle2:2;
    uint64_t Object_Length:8;
    uint64_t Object_Width:8;
  };
struct ARS_408_Data
{
	//0b
	int object_id;
	double long_dist;
	double lat_dist;
	double long_rel_vel;
	int dyn_prop;
	double lat_rel_vel;
	double object_rcs;
	//0c
	double long_dist_rms;
	double long_rel_vel_rms;
	double lat_dist_rms;
	double lat_rel_vel_rms;
	double long_rel_accel_rms;
	double lat_rel_accel_rms;
	double orientation_rms;
	int meas_state;
	std::string prob_of_exist;
	//0d
	double long_rel_accel;
	double lat_rel_accel;
	std::string object_class;
	double orientation_angle;
	double length;
	double width;

};
struct ARS_408
{
	int objects_number; //id
	int measurement_cycle_counter; 
	int interface_version;
	std::map<int, struct ARS_408_Data> data;
	std::list<int> objects_id;
	//struct ARS_408_Data data[100];
};

struct Ctrl_130 ctrl_130;
struct Ctrl_131 ctrl_131;
struct Ctrl_132 ctrl_132;
struct Ctrl_140 ctrl_140;
struct State_530 state_530;
struct State_531 state_531;
struct State_532 state_532;
struct State_534 state_534;
struct State_535 state_535;
struct State_539 state_539;
struct State_540 state_540;
struct State_541 state_541;
struct State_542 state_542;
struct State_543 state_543;
struct State_548 state_548;
struct State_53A state_53A;
struct State_53B state_53B;

struct State_60A state_60A;
struct State_60B state_60B;
struct State_60C state_60C;
struct State_60D state_60D;
struct ARS_408 ars_408;
 

void can_init(); 
void canbus_write_msg_init();


 //接收线程。
void *recv1_func(void* param) 
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

				//mtx_ptr.lock();

					// printf("CAN%d RX ID:0x%08X", 1, rec[j].ID);//ID
					// if(rec[j].ExternFlag==0) printf(" Standard ");//帧格式：标准帧
					// if(rec[j].ExternFlag==1) printf(" Extend   ");//帧格式：扩展帧
					// if(rec[j].RemoteFlag==0) printf(" Data   ");//帧类型：数据帧
					// if(rec[j].RemoteFlag==1) printf(" Remote ");//帧类型：远程帧
					// printf("DLC:0x%02X",rec[j].DataLen);//帧长度
					// printf(" data:0x");	//数据
					// for(i = 0; i < rec[j].DataLen; i++)
					// {
					// 	printf(" %02X", rec[j].Data[i]);
					// }
					// printf(" TimeStamp:0x%08X",rec[j].TimeStamp);//时间标识。
					// printf("\n");



				switch(rec[j].ID)
				{
				case 0x530:
					buff_ptr=(unsigned char *)&state_530;
					for(i = 0; i < rec[j].DataLen; i++)
					{
						*buff_ptr=rec[j].Data[i];
						buff_ptr++;
					}

					break;
				case 0x531:
					buff_ptr=(unsigned char *)&state_531;
					for(i = 0; i < rec[j].DataLen; i++)
					{
						*buff_ptr=rec[j].Data[i];
						buff_ptr++;
					}

					break;
				case 0x532:
					buff_ptr=(unsigned char *)&state_532;
					for(i = 0;

					 i < rec[j].DataLen; i++)
					{
						*buff_ptr=rec[j].Data[i];
						buff_ptr++;
					}

					break;
				case 0x534:
					buff_ptr=(unsigned char *)&state_534;
					for(i = 0; i < rec[j].DataLen; i++)
					{
						*buff_ptr=rec[j].Data[i];
						buff_ptr++;
					}

					break;
				case 0x535:
					buff_ptr=(unsigned char *)&state_535;
					for(i = 0; i < rec[j].DataLen; i++)
					{
						*buff_ptr=rec[j].Data[i];
						buff_ptr++;
					}

					break;
				case 0x539:
					buff_ptr=(unsigned char *)&state_539;
					for(i = 0; i < rec[j].DataLen; i++)
					{
						*buff_ptr=rec[j].Data[i];
						buff_ptr++;
					}

				 	break;
				case 0x540:
					buff_ptr=(unsigned char *)&state_540;
					for(i = 0; i < rec[j].DataLen; i++)
					{
						*buff_ptr=rec[j].Data[i];
						buff_ptr++;
					}

					break;
				case 0x541:
					buff_ptr=(unsigned char *)&state_541;
					for(i = 0; i < rec[j].DataLen; i++)
					{
						*buff_ptr=rec[j].Data[i];
						buff_ptr++;
					}

					break;
				case 0x542:
					buff_ptr=(unsigned char *)&state_542;
					for(i = 0; i < rec[j].DataLen; i++)
					{
						*buff_ptr=rec[j].Data[i];
						buff_ptr++;
					}

					break;
				case 0x543:
					buff_ptr=(unsigned char *)&state_543;
					for(i = 0; i < rec[j].DataLen; i++)
					{
						*buff_ptr=rec[j].Data[i];
						buff_ptr++;
					}

					break;
				case 0x548:
					buff_ptr=(unsigned char *)&state_548;
					for(i = 0; i < rec[j].DataLen; i++)
					{
						*buff_ptr=rec[j].Data[i];
						buff_ptr++;
					}

					break;
				case 0x53A:
					buff_ptr=(unsigned char *)&state_53A;
					for(i = 0; i < rec[j].DataLen; i++)
					{
						*buff_ptr=rec[j].Data[i];
						buff_ptr++;
					}
					break;
				case 0x53B:
					buff_ptr=(unsigned char *)&state_53B;
					for(i = 0; i < rec[j].DataLen; i++)
					{
						*buff_ptr=rec[j].Data[i];
						buff_ptr++;
					}
					break;
				 default:
		            //std::cout << "no find id:" << std::hex<<rec[j].ID<<std::endl;
		            break;
				}
				// mtx_ptr.unlock();
			}
			
		}	
	}
	printf("run thread exit\n");//退出接收线程	
	pthread_exit(0);
}

 //接收线程。
void *recv2_func(void* param) 
{
	int reclen=0;
	VCI_CAN_OBJ rec[3000];//接收缓存，设为3000为佳。
	int i,j;
	int *run=(int*)param;//线程启动，退出控制
	
	//unsigned char buff[8]={0
	unsigned char *buff_ptr;
	std::string ProbOfExis;
	std::string  object_class;
	while((*run)&0x0f)
	{
		if((reclen=VCI_Receive(VCI_USBCAN2,0,1,rec,3000,100))>0)//调用接收函数，如果有数据，进行数据处理显示。
		{

			for(j=0;j<reclen;j++)
			{

					// printf("CAN%d RX ID:0x%08X", 1, rec[j].ID);//ID
					// if(rec[j].ExternFlag==0) printf(" Standard ");//帧格式：标准帧
					// if(rec[j].ExternFlag==1) printf(" Extend   ");//帧格式：扩展帧
					// if(rec[j].RemoteFlag==0) printf(" Data   ");//帧类型：数据帧
					// if(rec[j].RemoteFlag==1) printf(" Remote ");//帧类型：远程帧
					// printf("DLC:0x%02X",rec[j].DataLen);//帧长度
					// printf(" data:0x");	//数据
					// for(i = 0; i < rec[j].DataLen; i++)
					// {
					// 	printf(" %02X", rec[j].Data[i]);
					// }
			 		// printf(" TimeStamp:0x%08X",rec[j].TimeStamp);//时间标识。
					// printf("\n");
					
			
			switch(rec[j].ID)
				{
				case 0x60A:
					mtx_408.lock();
					buff_ptr=(unsigned char *)&state_60A;
					for(i = 0; i < rec[j].DataLen; i++)
					{
						*buff_ptr=rec[j].Data[i];
						buff_ptr++;
					}
				
				
					//std::cout<<"0x60A ::"<<std::endl;
					//std::cout <<" NofObjects == "<<ars_408.objects_number<<std::endl;
					//id
					//std::cout<<" MeasCounter == "<<ars_408.measurement_cycle_counter<<std::endl;
					//取测量周期计数，从传感器启动开始向上计数，当 > 65535 时从 0 重新开始
					//std::cout<<" InterfaceVersion =="<<ars_408.interface_version<<std::endl;
					//CAN 接口版本号。在未更改Object标识符之前，它始终为“1”。
					
					 if (!ars_408.objects_id.empty())
					{
						ars_408.objects_id.sort();
						
						ars408_msg::msg::ARS408 ars480_msg;
						ars480_msg.header.stamp = rclcpp::Clock().now();  // 设置当前时间
						ars480_msg.header.frame_id = "base_link";  // 设置参考坐标系
						ars480_msg.objects_number=ars_408.objects_number;
						ars480_msg.measurement_cycle_counter=ars_408.measurement_cycle_counter;
						ars480_msg.interface_version=ars_408.interface_version;
						//int id=0;
						 for (auto i :ars_408.objects_id)
						 {
						 	ars408_msg::msg::MyObjects Objects_msg;
							// std::cout<<" i == "<< i<<std::endl;
							// std::cout<<"--------------------------------------------------------------------"<<std::endl;
							// std::cout<<" id == "<<ars_408.data[i].object_id<<std::endl;
							Objects_msg.object_id=ars_408.data[i].object_id;

							//std::cout<<" long_dist == "<<ars_408.data[i].long_dist<<std::endl;
							Objects_msg.long_dist=ars_408.data[i].long_dist;

							//std::cout<<" lat_dist == "<<ars_408.data[i].lat_dist<<std::endl;
							Objects_msg.lat_dist=ars_408.data[i].lat_dist;

							//std::cout<<" long_rel_vel == "<<ars_408.data[i].long_rel_vel<<std::endl;
							Objects_msg.long_rel_vel=ars_408.data[i].long_rel_vel;

							//std::cout<<" dyn_prop == "<<ars_408.data[i].dyn_prop<<std::endl;
							Objects_msg.dyn_prop=ars_408.data[i].dyn_prop;

							//std::cout<<" long_rel_vel == "<<ars_408.data[i].long_rel_vel<<std::endl;
							Objects_msg.long_rel_vel=ars_408.data[i].long_rel_vel;

							//std::cout<<" lat_rel_vel == "<<ars_408.data[i].lat_rel_vel<<std::endl;
							Objects_msg.lat_rel_vel=ars_408.data[i].lat_rel_vel;

							//std::cout<<" object_rcs == "<<ars_408.data[i].object_rcs<<std::endl;
							Objects_msg.object_rcs=ars_408.data[i].object_rcs;

							//std::cout<<" long_dist_rms == "<<ars_408.data[i].long_dist_rms<<std::endl;
							Objects_msg.long_dist_rms=ars_408.data[i].long_dist_rms;

							//std::cout<<" long_rel_vel_rms == "<<ars_408.data[i].long_rel_vel_rms<<std::endl;
							Objects_msg.long_rel_vel_rms=ars_408.data[i].long_rel_vel_rms;

							//std::cout<<" lat_dist_rms == "<<ars_408.data[i].lat_dist_rms<<std::endl;
							Objects_msg.lat_dist_rms=ars_408.data[i].lat_dist_rms;

							//std::cout<<" lat_rel_vel_rms == "<<ars_408.data[i].lat_rel_vel_rms<<std::endl;
							Objects_msg.lat_rel_vel_rms=ars_408.data[i].lat_rel_vel_rms;

							//std::cout<<" long_rel_accel_rms == "<<ars_408.data[i].long_rel_accel_rms<<std::endl;
							Objects_msg.long_rel_accel_rms=ars_408.data[i].long_rel_accel_rms;

							//std::cout<<" lat_rel_accel_rms == "<<ars_408.data[i].lat_rel_accel_rms<<std::endl;
							Objects_msg.lat_rel_accel_rms=ars_408.data[i].lat_rel_accel_rms;

							//std::cout<<" orientation_rms == "<<ars_408.data[i].orientation_rms<<std::endl;
							Objects_msg.orientation_rms=ars_408.data[i].orientation_rms;

							//std::cout<<" meas_state == "<<ars_408.data[i].meas_state<<std::endl;
							Objects_msg.meas_state=ars_408.data[i].meas_state;

							//std::cout<<" prob_of_exist == "<<ars_408.data[i].prob_of_exist<<std::endl;
							Objects_msg.prob_of_exist=ars_408.data[i].prob_of_exist;

							//std::cout<<" long_rel_accel == "<<ars_408.data[i].long_rel_accel<<std::endl;
							Objects_msg.long_rel_accel=ars_408.data[i].long_rel_accel;

							//std::cout<<" lat_rel_accel == "<<ars_408.data[i].lat_rel_accel<<std::endl;
							Objects_msg.lat_rel_accel=ars_408.data[i].lat_rel_accel;

							//std::cout<<" object_class == "<<ars_408.data[i].object_class<<std::endl;
							Objects_msg.object_class=ars_408.data[i].object_class;

							//std::cout<<" orientation_angle == "<<ars_408.data[i].orientation_angle<<std::endl;
							Objects_msg.orientation_angle=ars_408.data[i].orientation_angle;

							//std::cout<<" length == "<<ars_408.data[i].length<<std::endl;
							Objects_msg.length=ars_408.data[i].length;

							//std::cout<<" width == "<<ars_408.data[i].width<<std::endl;
							Objects_msg.width=ars_408.data[i].width;


							// std::cout<<"--------------------------------------------------------------------"<<std::endl;
							// std::cout<<""<<std::endl;
							// std::cout<<""<<std::endl;

							visualization_msgs::msg::Marker marker; 
							marker.header.frame_id = "/my_marker";
               				rclcpp::Clock::SharedPtr clock = rclcpp::Clock::make_shared();
    						marker.header.stamp = clock->now();
               				marker.ns = ars_408.data[i].prob_of_exist;
                			marker.id = ars_408.data[i].object_id;
                			marker.type = visualization_msgs::msg::Marker::CUBE;
                			marker.action = visualization_msgs::msg::Marker::ADD;
                			marker.pose.position.x = ars_408.data[i].long_dist;
			                marker.pose.position.y = ars_408.data[i].lat_dist;
			                marker.pose.position.z = 1;
			                marker.pose.orientation.x = 0.01;
			                marker.pose.orientation.y = 0.01;
			                marker.pose.orientation.z = 0.01;
			                marker.pose.orientation.w = 0.01;
			                marker.scale.x = ars_408.data[i].length;
			                marker.scale.y = ars_408.data[i].width;
			                marker.scale.z = 0.1;
			                //设定颜色----- 确保将  alpha  设置为非零值
			                marker.color.r = 0.0f;
			                marker.color.g = 1.0f;
			                marker.color.b = 0.0f;
			                marker.color.a = 1;

			               
    						marker.lifetime = rclcpp::Duration::from_seconds(0.1);
			                marker_pub->publish(marker);
			                ars480_msg.objects.push_back(Objects_msg);
						 }
						 ars408_pub->publish(ars480_msg);
					 //std::cout<<"---------------------------0x60A--------end----------------------------"<<std::endl;
					// std::cout<<""<<std::endl;
					 //std::cout<<""<<std::endl;
					 
					}
						
					
					ars_408.objects_number=state_60A.Object_NofObjects;
					ars_408.measurement_cycle_counter=(state_60A.Object_MeasCounter1<<8 |state_60A.Object_MeasCounter2);
					ars_408.interface_version=state_60A.Object_InterfaceVersion;
					ars_408.objects_id.clear();
					//std::cout<<"---------------------------0x60A-----------"<<ars_408.objects_number<<"-------------------------"<<std::endl;
					mtx_408.unlock();
					break;
				case 0x60B:
					buff_ptr=(unsigned char *)&state_60B;
					for(i = 0; i < rec[j].DataLen; i++)
					{
						*buff_ptr=rec[j].Data[i];
						buff_ptr++;
					}
					//std::cout<<"0x60B ::"<<std::endl;
					//std::cout<<" 0x60B id == "<<state_60B.Object_ID<<std::endl;
					//id
					//std::cout<<" object_long_dist == "<<(state_60B.Object_DistLong1<<5|state_60B.Object_DistLong2) * 0.2 - 500.0<<std::endl;
					//Longitudinal(x)坐标	单位：m
					//std::cout<<" object_lat_dist == "<<(state_60B.Object_DistLat1<<8|state_60B.Object_DistLat2)* 0.2 - 204.6<<std::endl;
					//获取Lateral(y)坐标	单位：m
					//std::cout<<" object_long_rel_vel == "<<(state_60B.Object_VrelLong1<<2|state_60B.Object_VrelLong2)* 0.25 - 128.0<<std::endl;
					//获取Longitudinal(x)方向相对速度	单位：m/s
					//std::cout<<" object_lat_rel_vel == "<<(state_60B.Object_VrelLat1<<3|state_60B.Object_VrelLat2)* 0.25 - 64.0<<std::endl;
					//获取Lateral(y)方向相对速度	单位：m/s
					//std::cout<<" object_dyn_prop == "<<state_60B.Object_DynProp<<std::endl;
					//object的动态属性，表示object是运动的还是静止的（
					//std::cout<<" object_rcs == "<<state_60B.Object_RCS* 0.5 - 64.0<<std::endl;
					//RCS(Radar cross section)雷达散射截面	单位：dBm2
					ars_408.objects_id.push_back(state_60B.Object_ID);
					ars_408.data[state_60B.Object_ID].object_id=state_60B.Object_ID;
					ars_408.data[state_60B.Object_ID].long_dist=(state_60B.Object_DistLong1<<5|state_60B.Object_DistLong2) * 0.2 - 500.0;
					ars_408.data[state_60B.Object_ID].lat_dist=(state_60B.Object_DistLat1<<8|state_60B.Object_DistLat2)* 0.2 - 204.6;
					ars_408.data[state_60B.Object_ID].long_rel_vel=(state_60B.Object_VrelLong1<<2|state_60B.Object_VrelLong2)* 0.25 - 128.0;
					ars_408.data[state_60B.Object_ID].lat_rel_vel=(state_60B.Object_VrelLat1<<3|state_60B.Object_VrelLat2)* 0.25 - 64.0;
					ars_408.data[state_60B.Object_ID].dyn_prop=state_60B.Object_DynProp;
					ars_408.data[state_60B.Object_ID].object_rcs=state_60B.Object_RCS* 0.5 - 64.0;
					break;
				case 0x60C:
					buff_ptr=(unsigned char *)&state_60C;
					for(i = 0; i < rec[j].DataLen; i++)
					{
						*buff_ptr=rec[j].Data[i];
						buff_ptr++;
					}
					//std::cout<<"0x60C ::"<<std::endl;
					//std::cout<<" 0x60C object_id == "<<state_60C.Object_ID<<std::endl;    
					//id
					//std::cout<<" get_object_lat_dist_rms == "<<(state_60C.Obj_DistLat_rms1 << 2 | state_60C.Obj_DistLat_rms2)<<std::endl;
					//获取Lateral(y)坐标标准差	
					//std::cout<<" object_long_dist_rms == "<<(state_60C.Obj_DistLong_rms)<<std::endl;
					//获取Longitudinal(x)坐标标准差
					//std::cout<<" object_lat_rel_vel_rms == "<<   (state_60C.Obj_VrelLat_rms1 << 4 | state_60C.Obj_VrelLat_rms2)<<std::endl;
					//获取 y 方向的相对速度的标准差	单位：m/s
					//std::cout<<" object_long_rel_vel_rms == "<<   state_60C.Obj_VrelLong_rms<<std::endl;
					//获取 x 方向的相对速度的标准差	
					//std::cout<<" bject_long_rel_accel_rms == "<< (state_60C.Obj_ArelLong_rms1 << 1 | state_60C.Obj_ArelLong_rms2)<<std::endl;
					//获取 x 方向的相对加速度的标准差	单位：m/s2
					//std::cout<<" object_lat_rel_accel_rms == "<<    state_60C.Obj_ArelLat_rms <<std::endl;
					//获取 y 方向的相对加速度的标准差	单位：m/s2
					//std::cout<<" object_meas_state == "<<   state_60C.Obj_MeasState<<std::endl;
					//测量状态：在新的测量周期内 object 是否有效，是否被 clusters 确认。0x0:deleted  0x1:new  0x2:measured  0x3: predicted  0x4:deleted for merge  0x5:new from merg
					//std::cout<<" object_prob_of_exist == "<<  state_60C.Obj_ProbOfExist<<std::endl;
					//存在的概率。0x0:invalid  0x1:<25%  0x2:<50%  0x3:<75%  0x4:<90%  0x5:<99%  0x6:<99.9%  0x7:<=100%
					//std::cout<<" object_orientation_rms == "<<  (state_60C.Obj_Orientation_rms1 << 3 | state_60C.Obj_Orientation_rms2)<<std::endl;
					//获取方位角标准差	单位：deg
					//ars_408.data[state_60C.Object_ID].object_id=state_60C.Object_ID;
					ars_408.data[state_60C.Object_ID].lat_dist_rms=state_60C.Obj_DistLat_rms1 << 2 | state_60C.Obj_DistLat_rms2;
					ars_408.data[state_60C.Object_ID].long_dist_rms=state_60C.Obj_DistLong_rms;
					ars_408.data[state_60C.Object_ID].lat_rel_vel_rms=state_60C.Obj_VrelLat_rms1 << 4 | state_60C.Obj_VrelLat_rms2;
					ars_408.data[state_60C.Object_ID].long_rel_vel_rms=state_60C.Obj_VrelLong_rms;
					ars_408.data[state_60C.Object_ID].long_rel_accel_rms=state_60C.Obj_ArelLong_rms1 << 1 | state_60C.Obj_ArelLong_rms2;
					ars_408.data[state_60C.Object_ID].lat_rel_accel_rms=state_60C.Obj_ArelLat_rms;
					ars_408.data[state_60C.Object_ID].meas_state=state_60C.Obj_MeasState;
					ars_408.data[state_60C.Object_ID].orientation_rms=state_60C.Obj_Orientation_rms1 << 3 | state_60C.Obj_Orientation_rms2;
					
				
						// if(state_60C.Obj_ProbOfExist == 0x0){ProbOfExis="invalid";}
						// else if(state_60C.Obj_ProbOfExist == 0x1){ProbOfExis="10%";}
						// else if(state_60C.Obj_ProbOfExist == 0x2){ProbOfExis="25%";}
						// else if(state_60C.Obj_ProbOfExist == 0x3){ProbOfExis="50%";}
						// else if(state_60C.Obj_ProbOfExist == 0x4){ProbOfExis="75%";}
						// else if(state_60C.Obj_ProbOfExist == 0x5){ProbOfExis="90%";}
						// else if(state_60C.Obj_ProbOfExist == 0x6){ProbOfExis="99.9%";}
						// else if(state_60C.Obj_ProbOfExist == 0x7){ProbOfExis=">=100%";}
						// else{ProbOfExis="<0%";}

					ars_408.data[state_60C.Object_ID].prob_of_exist=ProbOfExis;
					break;
		
				case 0x60D:

					buff_ptr=(unsigned char *)&state_60D;
					for(i = 0; i < rec[j].DataLen; i++)
					{
						*buff_ptr=rec[j].Data[i];
						buff_ptr++;
					};
					//std::cout<<"0x60D :: "<<std::endl;
					//std::cout<<" 0x60D object_id == " <<state_60D.Object_ID<<std::endl;
					//id
					//std::cout<<" object_long_rel_accel == "<<(state_60D.Object_ArelLong1<<3|state_60D.Object_ArelLong2)* 0.01 - 10.0<<std::endl;
					//存储位置从21-23位，8-15位。<<3取的是高位部分(8-15位)数据，| Long2取的是剩下的低位部分(21-23位)数据。Offset为 -10.0，Res为 0.01
					//std::cout<<" object_lat_rel_accel == "<<(state_60D.Object_ArelLat1 << 4 |state_60D.Object_ArelLat2)* 0.01 - 2.50<<std::endl;
					//存储位置从28-31位，16-20位。<<4取的是高位部分(16-20位)数据，| Long2取的是剩下的低位部分(28-31位)数据。Offset为 -2.50，Res为 0.01
					//std::cout<<" object_orientation_angle == "<<(state_60D.Object_OrientationAngle1<< 2 |state_60D.Object_OrientationAngle2) * 0.4 - 180.0<<std::endl;
					//获取object的方位角。随着时间的推移，被追踪的障碍物的旋转运动所产生的角度变化
					//std::cout<<" object_class == "<<state_60D.Object_Class<<std::endl;
					//获取目标类别，0x0:point  0x1:car  0x2:truck  0x3:not in use  0x4:motorcycle  0x5:bicycle  0x6:wide  0x7:reserved
					//std::cout<<" object_length == "<<state_60D.Object_Length * 0.2<<std::endl;
					///获取被跟踪object的长度	
					//std::cout<<" object_width == "<<state_60D.Object_Width * 0.2<<std::endl;
					//获取被跟踪object的宽度	单位：m
					//ars_408.data[state_60D.Object_ID].object_id=state_60D.Object_ID;
					ars_408.data[state_60D.Object_ID].long_rel_accel=(state_60D.Object_ArelLong1<<3|state_60D.Object_ArelLong2)* 0.01 - 10.0;
					ars_408.data[state_60D.Object_ID].lat_rel_accel=(state_60D.Object_ArelLat1 << 4 |state_60D.Object_ArelLat2)* 0.01 - 2.5;
					ars_408.data[state_60D.Object_ID].orientation_angle=(state_60D.Object_OrientationAngle1<< 2 |state_60D.Object_OrientationAngle2) * 0.4 - 180.0;
					ars_408.data[state_60D.Object_ID].length=state_60D.Object_Length * 0.2;
					ars_408.data[state_60D.Object_ID].width=state_60D.Object_Width * 0.2;
					//
					// if(state_60D.Object_Class==0x0){object_class="point";}
					// else if(state_60D.Object_Class==0x1){object_class="car";}
					// else if(state_60D.Object_Class==0x2){object_class="truck";}
					// else if(state_60D.Object_Class==0x3){object_class="people";}
					// else if(state_60D.Object_Class==0x4){object_class="motorcycle";}
					// else if(state_60D.Object_Class==0x5){object_class="bicycle";}
					// else if(state_60D.Object_Class==0x6){object_class="wide";}
					// else if(state_60D.Object_Class==0x7){object_class="reserved";}
					// else{object_class="NULL";}				
					
					ars_408.data[state_60D.Object_ID].object_class=object_class;

					break;
				default:
		            //std::cout << "no find id:" << std::hex<<rec[j].ID<<std::endl;
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
		// 	for(int i = 0; i < 8; i++)
		// {
		// 	printf(" %02X", canbus_Ctrl_140[0].Data[i]);
		// }
		//printf("\n");
		usleep(20000);
	}
    usleep(100000);//延时100ms。
	VCI_ResetCAN(VCI_USBCAN2, 0, 0);//复位CAN1通道。
	usleep(100000);//延时100ms。
	VCI_CloseDevice(VCI_USBCAN2,0);//关闭设备。
    pthread_exit(0);
}

void *topic_func(void* param) 
{
	std_msgs::msg::Header header;
	autoware_vehicle_msgs::msg::VelocityReport velocity_msg;
	tier4_vehicle_msgs::msg::BatteryStatus  BatteryStatus_msg;
	autoware_vehicle_msgs::msg::ControlModeReport  ControlModeReport_msg;
	autoware_vehicle_msgs::msg::GearReport  GearReport_msg;
	autoware_vehicle_msgs::msg::HazardLightsReport HazardLightsReport_msg;
	autoware_vehicle_msgs::msg::TurnIndicatorsReport TurnIndicatorsReport_msg;
	autoware_vehicle_msgs::msg::SteeringReport SteeringReport_msg;
 
	int *run=(int*)param;//线程启动，退出控制。
	rclcpp::Rate rate(10);
	while((*run)&0x0f)
	{


		rclcpp::Clock::SharedPtr clock = rclcpp::Clock::make_shared();
    	header.stamp = clock->now();

		velocity_msg.header=header;
		velocity_msg.header.frame_id="base_link";
		short longitudinal_velocity_msgs=state_530.ChassisSpeedFb;
		velocity_msg.longitudinal_velocity=(double)longitudinal_velocity_msgs/100;//纵向速度
//unsigned char lateral_velocity=state_532.ChassisSteerAngleSpeedFb;
		velocity_msg.lateral_velocity=0;//横向速度
		velocity_msg.heading_rate=0.0;//航向速率
		//速度反馈
		velocity_Status_pub->publish(velocity_msg);

		BatteryStatus_msg.stamp=header.stamp;
		BatteryStatus_msg.energy_level=state_535.ChassisPowerSocFb;
		//电池反馈
		battery_charge_pub->publish(BatteryStatus_msg);

		ControlModeReport_msg.stamp=header.stamp;
		if (state_534.DrivingModeFb==1)
		{
			ControlModeReport_msg.mode=state_534.DrivingModeFb;//自动
		}else{
			ControlModeReport_msg.mode=4;//手动
		}
		//车辆控制模式反馈
		control_mode_pub->publish(ControlModeReport_msg);

		GearReport_msg.stamp=header.stamp;
		if(state_530.ChassisGearFb==1)GearReport_msg.report=2; //D
		else if(state_530.ChassisGearFb==2)GearReport_msg.report=1; //N
		else if(state_530.ChassisGearFb==3)GearReport_msg.report=20;//R
		else GearReport_msg.report=0;//no use
		//档位反馈
		gear_status_pub->publish(GearReport_msg);


		HazardLightsReport_msg.stamp=header.stamp;
		HazardLightsReport_msg.report=state_540.VehicleHazardWarLampFb+1;
		//危险警示灯反馈
		hazard_lights_status_pub->publish(HazardLightsReport_msg);



		TurnIndicatorsReport_msg.stamp=header.stamp;

		if (state_540.VehicleLeftLampFb==0&&state_540.VehicleRightLampFb==0)
		{
			TurnIndicatorsReport_msg.report=1;
		}else if(state_540.VehicleLeftLampFb==1&&state_540.VehicleRightLampFb==0)
		{
			TurnIndicatorsReport_msg.report=2;
		}else{
			TurnIndicatorsReport_msg.report=3;
		}
		//转向灯反馈
		turn_indicators_status_pub->publish(TurnIndicatorsReport_msg);


		SteeringReport_msg.stamp=header.stamp;
		short steering_tire_angle_msg=state_532.ChassisSteerAngleFb;
		double double_angle_msg;
		double_angle_msg=steering_tire_angle_msg*0.048148148*(3.1415927/ 180);//要的弧度  给的是方向盘角度
		SteeringReport_msg.steering_tire_angle=-double_angle_msg;
		//转向角度反馈
		steering_status_pub->publish(SteeringReport_msg);



		rate.sleep();
	}
	std::cout<<"topic_threadid exit"<<std::endl;
 	pthread_exit(0);

}


void *hw_msg_func(void* param) 
{

	int *run=(int*)param;//线程启动，退出控制。
	rclcpp::Rate rate(1);
	mycan_msgs::msg::Hwcan  hwcan_msgs;

	while((*run)&0x0f)
	{

	hwcan_msgs.can530.chassisdriverensta=state_530.ChassisDriverEnSta;
	hwcan_msgs.can530.chassisdiverslopover=state_530.ChassisDiverSlopover;
	hwcan_msgs.can530.chassisdrivermodesta=state_530.ChassisDriverModeSta;
	hwcan_msgs.can530.chassisgearfb=state_530.ChassisGearFb;
	hwcan_msgs.can530.chassisspeedfb=state_530.ChassisSpeedFb;
	hwcan_msgs.can530.chassisthrottlepaldfb=state_530.ChassisThrottlePaldFb;

	hwcan_msgs.can531.chassisbrakeensta=state_531.ChassisBrakeEnSta;
	hwcan_msgs.can531.vehiclebrakelampfb=state_531.VehicleBrakeLampFb;
	hwcan_msgs.can531.chassisepbfb=state_531.ChassisEpbFb;
	hwcan_msgs.can531.chassisbrakepedalvalfb=state_531.ChassisBrakePedalValFb;
	hwcan_msgs.can531.chassisbrakepressurefb=state_531.ChassisBrakePressureFb;

	hwcan_msgs.can532.chassissteerensta=state_532.ChassisSteerEnSta;
	hwcan_msgs.can532.chassissteerslopover=state_532.ChassisSteerSlopover;
	hwcan_msgs.can532.chassissteermodefb=state_532.ChassisSteerModeFb;
	hwcan_msgs.can532.chassissteeranglefb=state_532.ChassisSteerAngleFb;
	hwcan_msgs.can532.chassissteeranglerearfb=state_532.ChassisSteerAngleRearFb;
	hwcan_msgs.can532.chassissteeranglespeedfb=state_532.ChassisSteerAngleSpeedFb;


	hwcan_msgs.can534.drivingmodefb=state_534.DrivingModeFb;
	hwcan_msgs.can534.chassispowerstafb=state_534.ChassisPowerStaFb;
	hwcan_msgs.can534.chassispowerdcsta=state_534.ChassisPowerDcSta;
	hwcan_msgs.can534.chassisspeedlimitedmodefb=state_534.ChassisSpeedLimitedModeFb;
	hwcan_msgs.can534.chassisspeedlimitedvalfb=state_534.ChassisSpeedLimitedValFb;
	hwcan_msgs.can534.chassislowpowervoltsta=state_534.ChassisLowPowerVoltSta;
	hwcan_msgs.can534.chassisestopstafb=state_534.ChassisEStopStaFb;
	hwcan_msgs.can534.crashfrontsta=state_534.CrashFrontSta;
	hwcan_msgs.can534.crashrearsta=state_534.CrashRearSta;


	hwcan_msgs.can535.chassisbmsreserved=state_535.ChassisBmsReserved;
	hwcan_msgs.can535.chassispowerchargesta=state_535.ChassisPowerChargeSta;
	hwcan_msgs.can535.chassispowerchargesocksta=state_535.ChassisPowerChargeSockSta;
	hwcan_msgs.can535.chassispowersocfb=state_535.ChassisPowerSocFb;
	hwcan_msgs.can535.chassispowervoltfb=state_535.ChassisPowerVoltFb;
	hwcan_msgs.can535.chassispowercurrfb=state_535.ChassisPowerCurrFb;
	hwcan_msgs.can535.chassisbmsmaxtemp=state_535.ChassisBmsMaxTemp;
	hwcan_msgs.can535.chassisbmsreserved1=state_535.ChassisBmsReserved1;


	hwcan_msgs.can539.chassiswheelrpmlf=state_539.ChassisWheelRpmLf;
	hwcan_msgs.can539.chassiswheelrpmrf=state_539.ChassisWheelRpmRf;
	hwcan_msgs.can539.chassiswheelrpmlr=state_539.ChassisWheelRpmLr;
	hwcan_msgs.can539.chassiswheelrpmrr=state_539.ChassisWheelRpmRr;

	hwcan_msgs.can53_a.vehicleodo=state_53A.VehicleODO;
	hwcan_msgs.can53_a.vehicletrip=state_53A.VehicleTrip;

	hwcan_msgs.can53_b.can_timeoutfb=state_53B.CAN_TimeoutFb;
	hwcan_msgs.can53_b.vehicletrip=state_53B.VehicleTrip;

	hwcan_msgs.can540.workenablefb=state_540.WorkEnableFb;
	hwcan_msgs.can540.sweepmodefb=state_540.SweepModeFb;
	hwcan_msgs.can540.fanmodefb=state_540.FanModeFb;
	hwcan_msgs.can540.vehiclectrlmodefb=state_540.VehicleCtrlModeFb;
	hwcan_msgs.can540.ctrlenablefb=state_540.CtrlEnableFb;
	hwcan_msgs.can540.vehicleposlampfb=state_540.VehiclePosLampFb;
	hwcan_msgs.can540.vehicleheadlampfb=state_540.VehicleHeadLampFb;
	hwcan_msgs.can540.vehicleleftlampfb=state_540.VehicleLeftLampFb;
	hwcan_msgs.can540.vehiclerightlampfb=state_540.VehicleRightLampFb;
	hwcan_msgs.can540.vehiclehighbeamfb=state_540.VehicleHighBeamFb;
	hwcan_msgs.can540.vehiclefoglampfb=state_540.VehicleFogLampFb;
	hwcan_msgs.can540.vehiclehazardwarlampfb=state_540.VehicleHazardWarLampFb;
	hwcan_msgs.can540.vehiclefronthornfb=state_540.VehicleFrontHornFb;
	hwcan_msgs.can540.vehicleworklampfb=state_540.VehicleWorkLampFb;
	hwcan_msgs.can540.vehiclewiperfb=state_540.VehicleWiperFb;
	hwcan_msgs.can540.garbageonfb=state_540.GarbageONFb;
	hwcan_msgs.can540.garbagewashingfb=state_540.GarbageWashingFb;
	hwcan_msgs.can540.washgunfb=state_540.WashGunFb;
	hwcan_msgs.can540.backdoorfb=state_540.BackdoorInplace;
	hwcan_msgs.can540.backdoorfb=state_540.BackdoorFb;
	hwcan_msgs.can540.garbagefb=state_540.GarbageFb;
	hwcan_msgs.can540.keystatusfb=state_540.KeyStatusFb;
	hwcan_msgs.can540.controlmodefb=state_540.ControlModeFb;
	hwcan_msgs.can540.vehicleemergency=state_540.VehicleEmergency;
	hwcan_msgs.can540.nozzlestatusfb=state_540.NozzleStatusFb;
	hwcan_msgs.can540.sweepwatersprayfb=state_540.SweepWaterSprayFb;
	hwcan_msgs.can540.arrowlightfb=state_540.ArrowLightFb;
	hwcan_msgs.can540.alarmbuzzerfb=state_540.AlarmBuzzerFb;
	hwcan_msgs.can540.waterstatusfb=state_540.WaterStatusFb;
	hwcan_msgs.can540.faultstatus=state_540.FaultStatus;
	hwcan_msgs.can540.dustvibrtionstatusfb=state_540.DustVibrtionStatusFb;
	hwcan_msgs.can540.drywetmodestatusfb=state_540.DryWetModeStatusFb;
	hwcan_msgs.can540.unloadingstatusfb=state_540.UnloadingStatusFb;
	hwcan_msgs.can540.sweepdebugmodefb=state_540.SweepDebugModeFb;
	hwcan_msgs.can540.fandebugmodefb=state_540.FanDebugModeFb;
	hwcan_msgs.can540.dustbinoverflowstatusfb=state_540.DustbinOverflowStatusFb;


	hwcan_msgs.can541.totalmileage=state_541.TotalMileage;
	hwcan_msgs.can541.subtotalmileage=state_541.SubtotalMileage;
	hwcan_msgs.can541.vin_1=state_541.VIN_1;
	hwcan_msgs.can541.vin_2=state_541.VIN_2;
	hwcan_msgs.can541.vin_3=state_541.VIN_3;
	hwcan_msgs.can541.vin_4=state_541.VIN_4;
	hwcan_msgs.can542.vin_5=state_542.VIN_5;
	hwcan_msgs.can542.vin_6=state_542.VIN_6;
	hwcan_msgs.can542.vin_7=state_542.VIN_7;
	hwcan_msgs.can542.vin_8=state_542.VIN_8;
	hwcan_msgs.can542.vin_9=state_542.VIN_9;
	hwcan_msgs.can542.vin_10=state_542.VIN_10;
	hwcan_msgs.can542.vin_11=state_542.VIN_11;


	hwcan_msgs.can543.faultdiagnosis=state_543.FaultDiagnosis;
	hwcan_msgs.can543.epstorquefb=state_543.EPSTorqueFb;
	hwcan_msgs.can543.epsanglefb=state_543.ModeChangeFb;
	hwcan_msgs.can543.bty_systemfault=state_543.Bty_SystemFault;
	hwcan_msgs.can543.mcu_systemfault=state_543.MCU_SystemFault;


	hwcan_msgs.can548.leftfronttirepressure=state_548.LeftFrontTirePressure;
	hwcan_msgs.can548.rightfronttirepressure=state_548.RightFrontTirePressure;
	hwcan_msgs.can548.rightreartirepressure=state_548.RightRearTirePressure;
	hwcan_msgs.can548.leftreartirepressure=state_548.LeftRearTirePressure;
	hwcan_msgs.can548.waterlevelratio=state_548.WaterlevelRatio;



 	hw_can_msgs_pub->publish(hwcan_msgs);
	rate.sleep();
	}

 	pthread_exit(0);

}


void Control_cmd_callback(const autoware_control_msgs::msg::Control::SharedPtr msg)
{	
	unsigned char * buf;
	short speed=msg->longitudinal.velocity*100;//*100 ；分辨率100
	short  angle=msg->lateral.steering_tire_angle*(180/3.1415957)/0.0684;//弧度变角度 分辨率1
	angle=-angle;
	if (speed<0)
	{
		speed=0;//防止意外
	}
	if (angle>380)//底盘最大转向角度+-380度
	{
		angle=380;
	}else if(angle<-380)
	{
		angle=-380;
	}

	// mtx_130.lock();
	// ctrl_130.SpeedCtrl=speed;

	// buf=(unsigned char *)&ctrl_130;
	// for(int i=0 ;i<8;i++)
	// {
	// 	canbus_Ctrl_130[0].Data[i]=*buf;
	// 	buf++;
	// }
	
	// mtx_130.unlock();

	mtx_132.lock();
	ctrl_132.SteerAngleTarget=angle;
	ctrl_132.SteerAngleSpeedCtrl=250;
	buf=(unsigned char *)&ctrl_132;

	for(int i=0 ;i<8;i++)
	{
		canbus_Ctrl_132[0].Data[i]=*buf;
		//printf("data= %d\n",ctrl_132[0].Data[i]);
		buf++;
	}
	mtx_132.unlock();
}
void Actuation_cmd_callback(const tier4_vehicle_msgs::msg::ActuationCommandStamped::SharedPtr msg)
{
	short accel_cmd=(short)(msg->actuation.accel_cmd*100*10);//
	short brake_cmd=(short)(msg->actuation.brake_cmd*100);//
	//unsigned int steer_cmd=msg->actuation.steer_cmd;//
	std::cout<<"time"<<i++<<std::endl;
		std::cout<<"accel_cmd "<<accel_cmd <<std::endl;
		std::cout<<"brake_cmd "<<brake_cmd <<std::endl;
		unsigned char * buf;
	mtx_130.lock();
	ctrl_130.ThrottlePdlTarget=accel_cmd;

	buf=(unsigned char *)&ctrl_130;
	for(int i=0 ;i<8;i++)
	{
		canbus_Ctrl_130[0].Data[i]=*buf;
		buf++;
	}
	
	mtx_130.unlock();


	mtx_131.lock();
	ctrl_131.BrakePdlTarget=brake_cmd;
	buf=(unsigned char *)&ctrl_131;
 	for(int i=0 ;i<8;i++)
	{
		canbus_Ctrl_131[0].Data[i]=*buf;
		//printf("data= %d\n",ctrl_132[0].Data[i]);
		buf++;
	}
	mtx_131.unlock();
}


void hw_mode_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
	int mode=msg->data;
	unsigned char * buf;
	if(state_530.ChassisSpeedFb!=0)
	{
		return;
	}
	//std::cout<<"msg->data =="<<mode<<std::endl;
	if (mode==1)
	{
		ctrl_140.VehicleCtrlModeCtrl=2;
	}else if(mode==2)
	{
		ctrl_140.VehicleCtrlModeCtrl=0;
	}
	mtx_140.lock();
	buf=(unsigned char *)&ctrl_140;
 	for(int i=0 ;i<8;i++)
	{
		canbus_Ctrl_140[0].Data[i]=*buf;
		//printf("data= %d\n",ctrl_132[0].Data[i]);
		buf++;
	}
	mtx_140.unlock();
	
}
void hw_work_callback(const std_msgs::msg::Int32::SharedPtr msg)
{

	int mode=msg->data;
	//std::cout<<mode<<std::endl;
	if (mode==1)
	{
		ctrl_140.WorkEnableCtrl=1;
		ctrl_140.EnableCtrl=1;
		
		ctrl_140.SweepWaterSprayCtrl=1;

		ctrl_131.BrakePdlTarget=0;
	}else if (mode==2)	
	{
		ctrl_140.WorkEnableCtrl=0;
		//ctrl_140.EnableCtrl=0;
		ctrl_140.SweepWaterSprayCtrl=0;

		ctrl_131.BrakePdlTarget=30*10;
	}else

	{
		return;
	}	
		
    
     
	unsigned char * buf;
	mtx_140.lock();
	buf=(unsigned char *)&ctrl_140;
 	for(int i=0 ;i<8;i++)
	{
		canbus_Ctrl_140[0].Data[i]=*buf;
		//printf("data= %d\n",ctrl_132[0].Data[i]);
		buf++;
	}
	mtx_140.unlock();
	mtx_131.lock();
	buf=(unsigned char *)&ctrl_131;
 	for(int i=0 ;i<8;i++)
	{
		canbus_Ctrl_131[0].Data[i]=*buf;
		//printf("data= %d\n",ctrl_132[0].Data[i]);
		buf++;
	}
	mtx_131.unlock();
	
	if(mode==2)	
	{
		//ctrl_140.WorkEnableCtrl=0;
		ctrl_140.EnableCtrl=0;	
		sleep(1);	
	}else

	{
		return;
	}	
		
	mtx_140.lock();
	buf=(unsigned char *)&ctrl_140;
 	for(int i=0 ;i<8;i++)
	{
		canbus_Ctrl_140[0].Data[i]=*buf;
		//printf("data= %d\n",ctrl_132[0].Data[i]);
		buf++;
	}
	mtx_140.unlock();
}

int main(int argc, char** argv)
{

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("canbus_node");

	

	can_init();

    canbus_write_msg_init();

  

   auto sub_hw_work_cmd = node->create_subscription<std_msgs::msg::Int32>("hw_work_state",  1, hw_work_callback);//工作状态
   auto sub_hw_mode_cmd = node->create_subscription<std_msgs::msg::Int32>("hw_mode_state",  1, hw_mode_callback);//驾驶模式
   auto sub_control_cmd = node->create_subscription<autoware_control_msgs::msg::Control>("/control/command/control_cmd",  1, Control_cmd_callback);//线速度 角速度
   //auto sub_gear_cmd = node->create_subscription<autoware_auto_vehicle_msgs::msg::GearCommand>("/control/command/gear_cmd",  1, Gear_cmd_callback);//档位
   //auto sub_gate_mode = node->create_subscription<tier4_control_msgs::msg::GateMode>("/control/current_gate_mode",  1, Gate_mode_callback);//是否控制autoware
   //auto sub_Emergency_cmd = node->create_subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>("/control/command/emergency_cmd",  1, Emergency_cmd_callback);//紧急信息
   //auto sub_Turn_indicators_cmd = node->create_subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>("/control/command/turn_indicators_cmd",  1, Turn_indicators_cmd_callback);//转向信号
   //auto sub_Hazard_lights_cmd = node->create_subscription<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>("/control/command/hazard_lights_cmd",  1, Hazard_lights_cmd_callback);//危险转向灯
   auto sub_Actuation_cmd = node->create_subscription<tier4_vehicle_msgs::msg::ActuationCommandStamped>("/control/command/actuation_cmd",  1, Actuation_cmd_callback);//油门踏板，TYPE B 控制车辆

   battery_charge_pub=node->create_publisher<tier4_vehicle_msgs::msg::BatteryStatus>("/vehicle/status/battery_charge", rclcpp::QoS{1});//电池信息
   control_mode_pub=node->create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>("/vehicle/status/control_mode", rclcpp::QoS{1});//控制模式
   gear_status_pub=node->create_publisher<autoware_vehicle_msgs::msg::GearReport>("/vehicle/status/gear_status", rclcpp::QoS{1});//当前档位
   hazard_lights_status_pub=node->create_publisher<autoware_vehicle_msgs::msg::HazardLightsReport>("/vehicle/status/hazard_lights_status", rclcpp::QoS{1});//危险灯状态
   turn_indicators_status_pub=node->create_publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>("/vehicle/status/turn_indicators_status", rclcpp::QoS{1});//转向状态
   steering_status_pub=node->create_publisher<autoware_vehicle_msgs::msg::SteeringReport>("/vehicle/status/steering_status", rclcpp::QoS{1});//转向状态
   velocity_Status_pub=node->create_publisher<autoware_vehicle_msgs::msg::VelocityReport>("/vehicle/status/velocity_status", rclcpp::QoS{1});//速度状态
   hw_can_msgs_pub=node->create_publisher<mycan_msgs::msg::Hwcan>("/hw_can_msgs", rclcpp::QoS{1});//反馈车辆信息
   marker_pub=node->create_publisher<visualization_msgs::msg::Marker>("/visualization_maker", rclcpp::QoS{1});//make
   ars408_pub=node->create_publisher<ars408_msg::msg::ARS408>("/ars408_msg", rclcpp::QoS{1});//ars408内容

	
	int m_run0=1;
   	pthread_t recv1_threadid,recv2_threadid,send_threadid,topic_threadid,hw_msg_threadid;
    canbus_write_msg_init();
    pthread_create(&recv1_threadid,NULL,recv1_func,&m_run0);	
    pthread_create(&recv2_threadid,NULL,recv2_func,&m_run0);	
    pthread_create(&send_threadid,NULL,send_func,&m_run0);
    pthread_create(&topic_threadid,NULL,topic_func,&m_run0);
   	pthread_create(&hw_msg_threadid,NULL,hw_msg_func,&m_run0);
   	printf(">>this is hello !\r\n");//指示程序已运行
   rclcpp::spin(node);
   rclcpp::shutdown();
}

void can_init()
{

    num=VCI_FindUsbDevice2(pInfo1);

	printf(">>USBCAN DEVICE NUM:");printf("%d", num);printf(" PCS");printf("\n");

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
	
	if(VCI_InitCAN(VCI_USBCAN2,0,1,&config)!=1)
	{
		printf(">>Init CAN2 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);
	}
	
	if(VCI_StartCAN(VCI_USBCAN2,0,1)!=1)
	{
		printf(">>Start CAN2 error\n");
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
    ctrl_130.DriverModeCtrl = 0x01;
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
	ctrl_140.VehicleCtrlModeCtrl=0;
	ctrl_140.EnableCtrl=1;
	ctrl_140.VehiclePosLampCtrl=0;
	ctrl_140.VehicleHeadLampCtrl=1;
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
