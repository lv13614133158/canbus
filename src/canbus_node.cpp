#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>
#include <thread>

struct can_frame canbus_DriveCtrl_130;
struct can_frame canbus_BrakeCtrl_131;
struct can_frame canbus_SteerCtrl_132;
struct can_frame canbus_VehicleCtrl_133;

int Socket;
struct sockaddr_can addr;
struct ifreq ifr;

int can_init()
{



    const char *ifname = "can1"; // CAN 接口名称，根据实际情况更改

    // 创建 socket
    if ((Socket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Socket creation error");
        return 1;
    }

    // 获取 CAN 接口索引
    strcpy(ifr.ifr_name, ifname);
    ioctl(Socket, SIOCGIFINDEX, &ifr);

    // 绑定到 CAN 接口
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(Socket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Bind error");
        close(Socket);
        return 1;
    }

    //模式控制
    canbus_DriveCtrl_130.can_id = 0x130;  // CAN 帧 ID
    canbus_DriveCtrl_130.can_dlc = 8;     // 数据长度
    canbus_DriveCtrl_130.data[0] = 0x01;
    canbus_DriveCtrl_130.data[1] = 0x01;
    canbus_DriveCtrl_130.data[2] = 0x00;
    canbus_DriveCtrl_130.data[3] = 0x00;
    canbus_DriveCtrl_130.data[4] = 0x00;
    canbus_DriveCtrl_130.data[5] = 0x00;
    canbus_DriveCtrl_130.data[6] = 0x00;
    canbus_DriveCtrl_130.data[7] = 0x00;

    //制动控制
    canbus_DriveCtrl_130.can_id = 0x131;  // CAN 帧 ID
    canbus_DriveCtrl_130.can_dlc = 8;     // 数据长度
    canbus_DriveCtrl_130.data[0] = 0x01;
    canbus_DriveCtrl_130.data[1] = 0x00;
    canbus_DriveCtrl_130.data[2] = 0x00;
    canbus_DriveCtrl_130.data[3] = 0x00;
    canbus_DriveCtrl_130.data[4] = 0x00;
    canbus_DriveCtrl_130.data[5] = 0x00;
    canbus_DriveCtrl_130.data[6] = 0x00;
    canbus_DriveCtrl_130.data[7] = 0x00;

    //转向控制
    canbus_DriveCtrl_130.can_id = 0x132;  // CAN 帧 ID
    canbus_DriveCtrl_130.can_dlc = 8;     // 数据长度
    canbus_DriveCtrl_130.data[0] = 0x00;
    canbus_DriveCtrl_130.data[1] = 0x00;
    canbus_DriveCtrl_130.data[2] = 0x00;
    canbus_DriveCtrl_130.data[3] = 0x00;
    canbus_DriveCtrl_130.data[4] = 0x00;
    canbus_DriveCtrl_130.data[5] = 0x00;
    canbus_DriveCtrl_130.data[6] = 0x00;
    canbus_DriveCtrl_130.data[7] = 0x00;

    //照明控制
    canbus_DriveCtrl_130.can_id = 0x133;  // CAN 帧 ID
    canbus_DriveCtrl_130.can_dlc = 8;     // 数据长度
    canbus_DriveCtrl_130.data[0] = 0x00;
    canbus_DriveCtrl_130.data[1] = 0x00;
    canbus_DriveCtrl_130.data[2] = 0x00;
    canbus_DriveCtrl_130.data[3] = 0x00;
    canbus_DriveCtrl_130.data[4] = 0x00;
    canbus_DriveCtrl_130.data[5] = 0x00;
    canbus_DriveCtrl_130.data[6] = 0x00;
    canbus_DriveCtrl_130.data[7] = 0x00;

    return 0;

}

void can_write_thread()
{
    struct can_frame canbus_write_data;
    int msg_len=sizeof(canbus_write_data);
    memset(&canbus_write_data,0,msg_len);
   std::cout<<"can_write_thread  ok"<<std::endl;
    while(0)
    {

    // 发送 CAN 帧
    if (write(Socket, &canbus_write_data, msg_len) != msg_len) 
    {
        perror("Write error");
    }

    usleep(1);
    }

    //close(Socket);
}

void can_read_thread(){

    std::cout<<"can_write_thread  ok"<<std::endl;
    struct can_frame recv_data;
    long int msg_len =sizeof(recv_data);
    while (true) {
    
        long  int nbytes = read(Socket, &recv_data, msg_len);
        if (nbytes < 0) {
            perror("Read error");
            break;
        } else if (nbytes < msg_len) {
            std::cerr << "Incomplete frame" << std::endl;
            break;
        }

        // 处理接收到的 CAN 帧
        // 可以在这里进行自定义的处理逻辑
        std::cout << "Received CAN frame ID: 0x" << std::hex << recv_data.can_id << std::endl;
        for (int i = 0; i < recv_data.can_dlc; ++i) {
            std::cout << "Data[" << i << "]: 0x" << std::hex << static_cast<int>(recv_data.data[i]) << std::endl;
        }
    }

}
int main(int argc, char** argv)
{



    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("canbus_node");



    //can通讯初始化
    can_init();
    std::cout<<"can_init  ok"<<std::endl;
    std::thread can_write(can_write_thread);
    std::thread can_read(can_read_thread);
  /* auto publisher = node->create_publisher<std_msgs::msg::String>("topic", 10);
    auto subscriber = node->create_subscription<std_msgs::msg::String>(
    "topic",
    10,
    callback);
   */
    std::cout<<"main  ok"<<std::endl;
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;



    /*
    // 接收 CAN 帧
    while (true) {
        struct can_frame recv_frame;
        long  int nbytes = read(s, &recv_frame, sizeof(recv_frame));
        if (nbytes < 0) {
            perror("Read error");
            break;
        } else if (nbytes < sizeof(recv_frame)) {
            std::cerr << "Incomplete frame" << std::endl;
            break;
        }

        // 处理接收到的 CAN 帧
        // 可以在这里进行自定义的处理逻辑
        std::cout << "Received CAN frame ID: 0x" << std::hex << recv_frame.can_id << std::endl;
        for (int i = 0; i < recv_frame.can_dlc; ++i) {
            std::cout << "Data[" << i << "]: 0x" << std::hex << static_cast<int>(recv_frame.data[i]) << std::endl;
        }
    }
    */

  
}