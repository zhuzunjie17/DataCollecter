#include <iostream>
#include <fstream>
#include <drm/radeon_drm.h>
#include "../../thirdparty/librealsense-1.12.1/linux-v4.4-wily/arch/cris/arch-v10/kernel/entry.S"

using namespace std;

int main()
{
	istream gyro;
	istream accel;
	ostream imu;
	
	gyro.open("/home/zhuzunjie/Documents/data/RealsenseData/lab/GYRO.txt");
	accel.open("/home/zhuzunjie/Documents/data/RealsenseData/lab/ACCEL.txt");
	imu.open("/home/zhuzunjie/Documents/data/RealsenseData/lab/imu.txt");
	
	
	
	
	
	gyro.close();
	accel.close();
	imu.close();
	
}