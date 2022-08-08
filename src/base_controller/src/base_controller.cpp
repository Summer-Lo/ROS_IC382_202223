#include <iostream>
#include <string>
#include <sstream>
using namespace std;

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "serial/serial.h"


#include "base_controller/my_msg.h"

#define sBUFFER_SIZE 1024
#define rBUFFER_SIZE 1024
unsigned char s_buffer[sBUFFER_SIZE];
unsigned char r_buffer[rBUFFER_SIZE];

serial::Serial ser;
typedef union
{
    float data;
    unsigned char data8[4];
} data_u;
data_u pitch;
data_u roll;
data_u yaw;

int main(int argc, char** argv)
{
    ros::init(argc,argv, "base_controller");
    ros::NodeHandle nh;

    try
    {
        ser.setPort("/dev/ttyACM0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch(serial::IOException &e)
    {
        ROS_INFO_STREAM("Failed to open port");
        return -1;
    }
    ROS_INFO_STREAM("Succeed to open port");

    int cnt = 0;
    ros::Rate loop_rate(500);
    ros::Publisher imu_pub = nh.advertise<base_controller::my_msg>("imu", 1000);
    while(ros::ok())
    {
        base_controller::my_msg msg;
        if(ser.available())
        {
            size_t bytes_read = ser.read(r_buffer, ser.available());
            int state = 0;
            unsigned char buffer[12] = {0};
            for(int i = 0; i< bytes_read && i < rBUFFER_SIZE; i++)
            {
                if(state == 0 && r_buffer[i] == 0xAA)
                {
                    state++;
                }
                else if(state == 1 && r_buffer[i] == 0xBB)
                {
                    state++;
                }
                else if(state == 2 && state < 14)
                {
                    buffer[state-2]=r_buffer[i];
                    state++;
                    if(state == 14)
                    {
                        for(int k =0; k < 4; k++)
                        {
                            roll.data8[k] = buffer[k];
                            pitch.data8[k] = buffer[4 + k];
                            yaw.data8[k] = buffer[8 + k];
                        }
                        ROS_INFO("%f, %f, %f, %d", roll.data, pitch.data, yaw.data, cnt);
                        state = 0;
                    }
                }
                else state = 0;
            }
        }
        msg.roll = roll.data;
        msg.pitch = pitch.data;
        msg.yaw = yaw.data;
        //msg.yaw = 1;
        imu_pub.publish(msg);
        loop_rate.sleep();
    }
    ser.close();
    return 0;
}
