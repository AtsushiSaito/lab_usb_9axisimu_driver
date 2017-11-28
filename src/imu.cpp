#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <string>
#include "ros/ros.h"

#include "lab_usb_9axisimu_driver/Axis.h"
#include "lab_usb_9axisimu_driver/Mag.h"
#include "lab_usb_9axisimu_driver/Temp.h"

#define SERIAL_PORT "/dev/ttyACM0"
using namespace std;

int main(int argc, char **argv){

    ros::init(argc, argv, "Imu_Pub");
    ros::NodeHandle n;
    ros::Publisher axis_pub = n.advertise<lab_usb_9axisimu_driver::Axis>("Imu_Axis", 100);
    ros::Publisher mag_pub = n.advertise<lab_usb_9axisimu_driver::Mag>("Imu_Mag", 100);
    ros::Publisher temp_pub = n.advertise<lab_usb_9axisimu_driver::Temp>("Imu_Temp", 100);
    lab_usb_9axisimu_driver::Axis axis_msg;
    lab_usb_9axisimu_driver::Mag mag_msg;
    lab_usb_9axisimu_driver::Temp temp_msg;

    unsigned char buf[255];
    struct termios tio;
    int baudRate = B115200;
    int i,fd,buf_size,count=0;
    vector<string> imu_data;
    string buf_imu;

    fd = open(SERIAL_PORT, O_RDWR);
    if(fd < 0){
        cout << "Device not found." << endl;
        return -1;
    }

    tio.c_cflag += CREAD;
    tio.c_cflag += CLOCAL;
    tio.c_cflag += CS8;
    tio.c_cflag += 0;
    tio.c_cflag += 0;
    cfsetispeed( &tio, baudRate );
    cfsetospeed( &tio, baudRate );
    cfmakeraw(&tio);
    tcsetattr( fd, TCSANOW, &tio );
    ioctl(fd, TCSETS, &tio);
    
    while(ros::ok()){
        buf_imu.clear();
        buf_size = read(fd,buf,sizeof(buf));
        if (0 < buf_size){
            for(i = 0; i < buf_size; i++){
                if(buf[i]==','|| buf[i]=='\n'){
                    count++;
                    imu_data.push_back(buf_imu);
                    buf_imu.clear();
                }else{
                    buf_imu+=buf[i];
                }
                if(buf[i]=='\n' && count==11){
                    axis_msg.time = stoi(imu_data[0]);
                    mag_msg.time = stoi(imu_data[0]);
                    temp_msg.time = stoi(imu_data[0]);
                    axis_msg.omega_x = stof(imu_data[1]);
                    axis_msg.omega_y = stof(imu_data[2]);
                    axis_msg.omega_z = stof(imu_data[3]);
                    axis_msg.accel_x = stof(imu_data[4]);
                    axis_msg.accel_y = stof(imu_data[5]);
                    axis_msg.accel_z = stof(imu_data[6]);
                    mag_msg.mag_x = stof(imu_data[7]);
                    mag_msg.mag_y = stof(imu_data[8]);
                    mag_msg.mag_z = stof(imu_data[9]);
                    temp_msg.temp = stof(imu_data[10]);
                    axis_pub.publish(axis_msg);
                    mag_pub.publish(mag_msg);
                    temp_pub.publish(temp_msg);
                    count = 0;
                    //cout << "---------------------------------" << endl;
                    imu_data.clear();
                }else if(count>11){
                    count=0;
                }
            }
        }
    }
    close(fd);
    return 0;
}
