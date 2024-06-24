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
#include <websocket_msgs/msg/websocket.hpp>
//#include "builtin_interfaces/msg/Time.idl"

#include "autoware_control_msgs/msg/control.hpp"
#include "autoware_auto_vehicle_msgs/msg/gear_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/control_mode_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/gear_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/hazard_lights_report.hpp"
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
#include "sensor_msgs/msg/nav_sat_fix.hpp"


rclcpp::Publisher<tier4_vehicle_msgs::msg::BatteryStatus>::SharedPtr battery_charge_pub;
rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr control_mode_pub;
rclcpp::Publisher<autoware_vehicle_msgs::msg::GearReport>::SharedPtr gear_status_pub;
rclcpp::Publisher<autoware_vehicle_msgs::msg::HazardLightsReport>::SharedPtr hazard_lights_status_pub;
rclcpp::Publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr turn_indicators_status_pub;
rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_status_pub;
rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_Status_pub;
rclcpp::Publisher<websocket_msgs::msg::Websocket>::SharedPtr websocket_pub;


VCI_BOARD_INFO pInfo;//用来获取设备信息。
int count=0;//数据列表中，用来存储列表序号。
int count1=0;
VCI_BOARD_INFO pInfo1 [50];

double wgLat=41.768628;
double wgLon=123.435442;
int num=0;
const double pi = 3.14159265358979324;
const double a = 6378245.0;
const double ee = 0.00669342162296594323;


std::mutex mtx_130;
std::mutex mtx_131;
std::mutex mtx_132;
std::mutex mtx_140;
std::mutex mtx_ptr;

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
	uint64_t EPSTorqueFb:8;
	uint64_t EPSAngleFb:16;
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
 static bool outOfChina(double lat, double lon)
{
    if (lon < 72.004 || lon > 137.8347)
        return true;
    if (lat < 0.8293 || lat > 55.8271)
        return true;
    return false;
}


static double transformLat(double x, double y)
{
    double ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * sqrt(abs(x));
    ret += (20.0 * sin(6.0 * x * pi) + 20.0 * sin(2.0 * x * pi)) * 2.0 / 3.0;
    ret += (20.0 * sin(y * pi) + 40.0 * sin(y / 3.0 * pi)) * 2.0 / 3.0;
    ret += (160.0 * sin(y / 12.0 * pi) + 320 * sin(y * pi / 30.0)) * 2.0 / 3.0;
    return ret;
}


static double transformLon(double x, double y)
{
    double ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * sqrt(abs(x));
    ret += (20.0 * sin(6.0 * x * pi) + 20.0 * sin(2.0 * x * pi)) * 2.0 / 3.0;
    ret += (20.0 * sin(x * pi) + 40.0 * sin(x / 3.0 * pi)) * 2.0 / 3.0;
    ret += (150.0 * sin(x / 12.0 * pi) + 300.0 * sin(x / 30.0 * pi)) * 2.0 / 3.0;
    return ret;
}


void gps_transform( double wgLat, double wgLon, double& mgLat, double& mgLon)
{
if (outOfChina(wgLat, wgLon))
        {
            mgLat = wgLat;
            mgLon = wgLon;
            return;
        }
        double dLat = transformLat(wgLon - 105.0, wgLat - 35.0);
        double dLon = transformLon(wgLon - 105.0, wgLat - 35.0);
        double radLat = wgLat / 180.0 * pi;
        double magic = sin(radLat);
        magic = 1 - ee * magic * magic;
        double sqrtMagic = sqrt(magic);
        dLat = (dLat * 180.0) / ((a * (1 - ee)) / (magic * sqrtMagic) * pi);
        dLon = (dLon * 180.0) / (a / sqrtMagic * cos(radLat) * pi);
        mgLat = wgLat + dLat;
        mgLon = wgLon + dLon;
};

void can_init(); 
void canbus_write_msg_init();
void get_time(char* buffer )
{
            //获取时间
    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::strftime(buffer, 80, "%Y-%m-%d %H:%M:%S", std::localtime(&now_c));
}

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
		            std::cout << "no find id:" << std::hex<<rec[j].ID<<std::endl;
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
	
	int *run=(int*)param;//线程启动，退出控制。
	//unsigned char buff[8]={0};
	
	
	while((*run)&0x0f)
	{
		if((reclen=VCI_Receive(VCI_USBCAN2,0,1,rec,3000,100))>0)//调用接收函数，如果有数据，进行数据处理显示。
		{

			for(j=0;j<reclen;j++)
			{

					printf("CAN%d RX ID:0x%08X", 1, rec[j].ID);//ID
					if(rec[j].ExternFlag==0) printf(" Standard ");//帧格式：标准帧
					if(rec[j].ExternFlag==1) printf(" Extend   ");//帧格式：扩展帧
					if(rec[j].RemoteFlag==0) printf(" Data   ");//帧类型：数据帧
					if(rec[j].RemoteFlag==1) printf(" Remote ");//帧类型：远程帧
					printf("DLC:0x%02X",rec[j].DataLen);//帧长度
					printf(" data:0x");	//数据
					for(i = 0; i < rec[j].DataLen; i++)
					{
						printf(" %02X", rec[j].Data[i]);
					}
					printf(" TimeStamp:0x%08X",rec[j].TimeStamp);//时间标识。
					printf("\n");



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
	rclcpp::Rate rate(100);
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

 	pthread_exit(0);

}


void *websocket_func(void* param) 
{

	int *run=(int*)param;//线程启动，退出控制。
	rclcpp::Rate rate(1.0/3.0);
	websocket_msgs::msg::Websocket  websocket_msgs;
	std::string Vin="LCFCHBHE1P1010046";
	websocket_msgs.type="report";
	websocket_msgs.vin=Vin;
	websocket_msgs.data.car_no="京N GN0000";
	websocket_msgs.data.vin=Vin;
	websocket_msgs.data.vehicle_status=0;


	double mgLat;
	double mgLon;
	while((*run)&0x0f)
	{



    	websocket_msgs.data.taskld=1;
    	int  battery=state_535.ChassisPowerSocFb;
      	websocket_msgs.data.battery=battery;
       	int cupula_work_status=state_540.WorkEnableFb;
   		websocket_msgs.data.cupula_work_status=cupula_work_status;
   		int driving_mode=state_540.VehicleCtrlModeFb;
   		if (driving_mode==0)
   		{
   			driving_mode=2;
   		}else if(driving_mode==2)
   		{
   			driving_mode=1;
   		}else{
   			driving_mode=3;
   		}
    	websocket_msgs.data.driving_mode=driving_mode;
    	//worktime
       	websocket_msgs.data.expected_remaining_driving_time=0;
       	int garbage_bin_overflow_status=state_540.DustbinOverflowStatusFb;
        websocket_msgs.data.garbage_bin_overflow_status=garbage_bin_overflow_status;

        gps_transform(wgLat,wgLon,mgLat,mgLon);
      	websocket_msgs.data.latitude=mgLat;
      	websocket_msgs.data.location="北京";
      	websocket_msgs.data.longitude=mgLon;
      	websocket_msgs.data.params="string";
      	int remaining_water_percentage=state_548.WaterlevelRatio;
     	websocket_msgs.data.remaining_water_percentage=remaining_water_percentage;
     	websocket_msgs.data.remark="备注";
     	char time[48];
     	get_time(time);
  		websocket_msgs.data.reporting_time=time;
  		double speed=(double)state_530.ChassisSpeedFb/100*3.6;
   		websocket_msgs.data.speed=std::to_string(speed);
   		int sweep_work_status=state_540.WorkEnableFb;
   		websocket_msgs.data.sweep_work_status=sweep_work_status;
   		int watering_work_status=state_540.SweepWaterSprayFb;
    	websocket_msgs.data.watering_work_status=watering_work_status;
    	websocket_pub->publish(websocket_msgs);
		rate.sleep();
	}

 	pthread_exit(0);

}


void Control_cmd_callback(const autoware_control_msgs::msg::Control::SharedPtr msg)
{	
	unsigned char * buf;
	unsigned short speed=msg->longitudinal.velocity*100;//*100 ；分辨率100
	short  angle=msg->lateral.steering_tire_angle*(180/3.1415957)/0.0684;//弧度变角度 分辨率1
	angle=-angle;
	// if (speed<0)
	// {
	// 	speed=0;//防止意外
	// }
	if (angle>380)//底盘最大转向角度+-380度
	{
		angle=380;
	}else if(angle<-380)
	{
		angle=-380;
	}

	mtx_130.lock();
	ctrl_130.SpeedCtrl=speed;

	buf=(unsigned char *)&ctrl_130;
	for(int i=0 ;i<8;i++)
	{
		canbus_Ctrl_130[0].Data[i]=*buf;
		buf++;
	}
	
	mtx_130.unlock();

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
	unsigned int accel_cmd=msg->actuation.accel_cmd;//
	unsigned int brake_cmd=msg->actuation.brake_cmd;//
	//unsigned int steer_cmd=msg->actuation.steer_cmd;//

	unsigned char * buf;
	mtx_130.lock();
	ctrl_130.ThrottlePdlTarget=accel_cmd*10;

	buf=(unsigned char *)&ctrl_130;
	for(int i=0 ;i<8;i++)
	{
		canbus_Ctrl_130[0].Data[i]=*buf;
		buf++;
	}
	
	mtx_130.unlock();


	mtx_131.lock();
	ctrl_131.BrakePdlTarget=brake_cmd*10;
	buf=(unsigned char *)&ctrl_131;
 	for(int i=0 ;i<8;i++)
	{
		canbus_Ctrl_131[0].Data[i]=*buf;
		//printf("data= %d\n",ctrl_132[0].Data[i]);
		buf++;
	}
	mtx_131.unlock();
}


void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
	sensor_msgs::msg::NavSatFix gps=*msg;
	wgLat=gps.latitude;
	wgLon=gps.longitude;
	// std::cout<<wgLat<<std::endl;
	// std::cout<<wgLon<<std::endl;
}
void Autowaremode_callback(const autoware_adapi_v1_msgs::msg::OperationModeState::SharedPtr msg)
{

	int mode=msg->mode;
	std::cout<<mode<<std::endl;
	if (mode==2)
	{
		ctrl_140.WorkEnableCtrl=1;
		ctrl_140.EnableCtrl=1;
		ctrl_140.SweepWaterSprayCtrl=1;

		ctrl_131.BrakePdlTarget=0;
	}else if (mode==1)	
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
	
	if(mode==1)	
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

	printf(">>this is hello !\r\n");//指示程序已运行

	can_init();

    canbus_write_msg_init();

  

   auto sub_autoware_mode_cmd = node->create_subscription<autoware_adapi_v1_msgs::msg::OperationModeState>("/api/operation_mode/state",  1, Autowaremode_callback);//autoware_mode
   auto sub_gps_cmd = node->create_subscription<sensor_msgs::msg::NavSatFix>("/sensing/gnss/ublox/nav_sat_fix",  1, gps_callback);//autoware_mode
   auto sub_control_cmd = node->create_subscription<autoware_control_msgs::msg::Control>("/control/command/control_cmd",  1, Control_cmd_callback);//线速度 角速度
   //auto sub_gear_cmd = node->create_subscription<autoware_auto_vehicle_msgs::msg::GearCommand>("/control/command/gear_cmd",  1, Gear_cmd_callback);//档位
   //auto sub_gate_mode = node->create_subscription<tier4_control_msgs::msg::GateMode>("/control/current_gate_mode",  1, Gate_mode_callback);//是否控制autoware
   //auto sub_Emergency_cmd = node->create_subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>("/control/command/emergency_cmd",  1, Emergency_cmd_callback);//紧急信息
   //auto sub_Turn_indicators_cmd = node->create_subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>("/control/command/turn_indicators_cmd",  1, Turn_indicators_cmd_callback);//转向信号
   //auto sub_Hazard_lights_cmd = node->create_subscription<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>("/control/command/hazard_lights_cmd",  1, Hazard_lights_cmd_callback);//危险转向灯
   //auto sub_Actuation_cmd = node->create_subscription<tier4_vehicle_msgs::msg::ActuationCommandStamped>("/control/command/actuation_cmd",  1, Actuation_cmd_callback);//油门踏板，TYPE B 控制车辆

   battery_charge_pub=node->create_publisher<tier4_vehicle_msgs::msg::BatteryStatus>("/vehicle/status/battery_charge", rclcpp::QoS{1});//电池信息
   control_mode_pub=node->create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>("/vehicle/status/control_mode", rclcpp::QoS{1});//控制模式
   gear_status_pub=node->create_publisher<autoware_vehicle_msgs::msg::GearReport>("/vehicle/status/gear_status", rclcpp::QoS{1});//当前档位
   hazard_lights_status_pub=node->create_publisher<autoware_vehicle_msgs::msg::HazardLightsReport>("/vehicle/status/hazard_lights_status", rclcpp::QoS{1});//危险灯状态
   turn_indicators_status_pub=node->create_publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>("/vehicle/status/turn_indicators_status", rclcpp::QoS{1});//转向状态
   steering_status_pub=node->create_publisher<autoware_vehicle_msgs::msg::SteeringReport>("/vehicle/status/steering_status", rclcpp::QoS{1});//转向状态
   velocity_Status_pub=node->create_publisher<autoware_vehicle_msgs::msg::VelocityReport>("/vehicle/status/velocity_status", rclcpp::QoS{1});//速度状态
   websocket_pub=node->create_publisher<websocket_msgs::msg::Websocket>("/websocket_in", rclcpp::QoS{1});//速度状态

	
	int m_run0=1;
   	pthread_t recv1_threadid,recv2_threadid,send_threadid,topic_threadid;
    canbus_write_msg_init();
	pthread_create(&recv1_threadid,NULL,recv1_func,&m_run0);	
	pthread_create(&recv2_threadid,NULL,recv2_func,&m_run0);	
    pthread_create(&send_threadid,NULL,send_func,&m_run0);
    pthread_create(&topic_threadid,NULL,topic_func,&m_run0);
    pthread_create(&topic_threadid,NULL,websocket_func,&m_run0);

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
	ctrl_140.VehicleCtrlModeCtrl=0;
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