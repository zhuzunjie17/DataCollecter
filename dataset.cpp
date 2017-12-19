#include <librealsense/rs.hpp>
#include "example.hpp"
#include "concurrency.hpp"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <opencv2/opencv.hpp>
#include <GLFW/glfw3.h>
#include <eigen3/Eigen/Core>

#include <iostream>
#include <fstream>
#include <vector>
#include <cstdio>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <string>
#include <stdlib.h>
#include <thread>
#include <atomic>
#include <map>
using namespace std;
using namespace cv;

void make_depth_histogram(Mat rgb_image, Mat depth_image, int width, int height)
{
	static uint32_t histogram[0x10000];
	memset(histogram, 0, sizeof(histogram));
	uchar* pmat = rgb_image.data;
	
	for(int i = 0; i < height; ++i)
		for(int j = 0; j < width; ++j)
		{
// 			if(ushort d = depth_image.at<ushort>(i,j))
				++histogram[depth_image.at<ushort>(i,j)];
		}
	for(int i = 2; i < 0x10000; ++i) histogram[i] += histogram[i-1]; // Build a cumulative histogram for the indices in [1,0xFFFF]
	for(int i = 0; i < height; ++i)
		for(int j = 0; j < width; ++j)
		{
			if(ushort d = depth_image.at<ushort>(i,j))
			{
				uchar f = histogram[d] * 255 / histogram[0xFFFF]; // 0-255 based on histogram location
				*pmat = 0;
				pmat++;
				*pmat = 255 - f;
				pmat++;
				*pmat = f;
				pmat++;
			}
			else
			{
				*pmat = 5;
				pmat++;
				*pmat = 20;
				pmat++;
				*pmat = 0;
				pmat++;
			}
		}
}

int main() try
{
    //init
    static string str;
	str = "/home/zhuzunjie/Documents/data/RealsenseData/testdepthRGB/";
    
    rs::context ctx;
    printf("There are %d connected RealSense devices.\n", ctx.get_device_count());
    if(ctx.get_device_count() == 0) return EXIT_FAILURE;
    
    rs::device *dev = ctx.get_device(0);
    printf("\nUsing device 0, an %s\n", dev->get_name());
    printf("    Serial number: %s\n", dev->get_serial());
    printf("    Firmware version: %s\n", dev->get_firmware_version());
    
    const auto streams = 5;
    std::vector<uint16_t> supported_streams = { (uint16_t)rs::stream::depth, (uint16_t)rs::stream::color, (uint16_t)rs::stream::infrared,(uint16_t)rs::stream::fisheye};
    const size_t max_queue_size = 1; // To minimize latency prefer frame drops
    static single_consumer_queue<rs::frame> frames_queue[streams];
	static single_consumer_queue<rs::frame> query;
    texture_buffer buffers[streams];
    static std::atomic<bool> running(true);
    
    static vector<vector<Mat>> images(6);
    static vector<vector<string>> name(6);
    
    static map<rs::stream,int> components_map =
    {
    { rs::stream::depth,     3  },      // RGB
    { rs::stream::color,     3  },
    { rs::stream::infrared , 1  },      // Monochromatic
    { rs::stream::infrared2, 1  },
    { rs::stream::fisheye,   1  }
    };

    struct resolution
    {
        int width;
        int height;
        rs::format format;
    };
    std::map<rs::stream, resolution> resolutions;
    
    static ofstream fimu,faccel,fgyro,fdepth,fcolor,finf,finf2,ffisheye;
    fimu.open(str + "IMU.txt");
    faccel.open(str + "ACCEL.txt");
    fgyro.open(str + "GYRO.txt");
    fdepth.open(str + "DEPTH.txt");
    fcolor.open(str + "COLOR.txt");
    finf.open(str + "INFRARED.txt");
    finf2.open(str + "INFRARED2.txt");
    ffisheye.open(str + "FISHEYE.txt");
	fimu   << "#timestamp(ms) rx(rad s^-1) ry(rad s^-1) rz(rad s^-1) ax(m s^-2) ay(m s^-2) az(m s^-2)";
    faccel << "#timestamp(ms) ax(m s^-2) ay(m s^-2) az(m s^-2)" << endl;
    fgyro  << "#timestamp(ms) rx(rad s^-1) ry(rad s^-1) rz(rad s^-1)";
    fdepth << "#timestamp(ms) filename" << endl;
    fcolor << "#timestamp(ms) filename" << endl;
    finf   << "#timestamp(ms) filename" << endl;
    finf2  << "#timestamp(ms) filename" << endl;
    ffisheye << "#timestamp(ms) filename" << endl;
    
	
	
	static double gtime=0.0,atime=0.0,accel1,accel2,accel3;
    auto motion_callback = [](rs::motion_data entry)
    {
		if (entry.timestamp_data.source_id == RS_EVENT_IMU_ACCEL)
		{
			faccel << fixed << setprecision(2) << entry.timestamp_data.timestamp << " "
			<< fixed << setprecision(5) << entry.axes[0] << " " << entry.axes[1] << " " << entry.axes[2]
			<< endl;
			double t = entry.timestamp_data.timestamp;
			if(t < gtime || gtime == 0.0 || atime >= gtime )
			{
				atime = t;
				accel1 = entry.axes[0];
				accel2 = entry.axes[1];
				accel3 = entry.axes[2];
			}	
			else 
			{
				fgyro << " " << ((t-gtime)*accel1+(gtime-atime)*entry.axes[0])/(t-atime)
				<< " " << ((t-gtime)*accel2+(gtime-atime)*entry.axes[1])/(t-atime)
				<< " " << ((t-gtime)*accel3+(gtime-atime)*entry.axes[2])/(t-atime);
				
				fimu << " " << ((t-gtime)*accel1+(gtime-atime)*entry.axes[0])/(t-atime)
				<< " " << ((t-gtime)*accel2+(gtime-atime)*entry.axes[1])/(t-atime)
				<< " " << ((t-gtime)*accel3+(gtime-atime)*entry.axes[2])/(t-atime);
				
				
				atime = t;
				accel1 = entry.axes[0];
				accel2 = entry.axes[1];
				accel3 = entry.axes[2];
				
			}
		}
		
		if (entry.timestamp_data.source_id == RS_EVENT_IMU_GYRO)
		{
			fgyro << endl
			<< fixed << setprecision(2) << entry.timestamp_data.timestamp << " "
			<< fixed << setprecision(5) << entry.axes[0] << " " << entry.axes[1] << " " << entry.axes[2];
			
			fimu << endl
			<< fixed << setprecision(2) << entry.timestamp_data.timestamp << " "
			<< fixed << setprecision(5) << entry.axes[0] << " " << entry.axes[1] << " " << entry.axes[2];
			
			
			gtime = entry.timestamp_data.timestamp;
		}
    };

    // ... and the timestamp packets (DS4.1/FishEye Frame, GPIOS...)
    auto timestamp_callback = [](rs::timestamp_data entry)
    {
        
    };

    // 1. Make motion-tracking available
    if (dev->supports(rs::capabilities::motion_events))
    {
	dev->enable_motion_tracking(motion_callback, timestamp_callback);
    }

    // 2. Optional - configure motion module
    //dev->set_options(mm_cfg_list.data(), mm_cfg_list.size(), mm_cfg_params.data());
    
    dev->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 30);
    dev->enable_stream(rs::stream::color, 640, 480, rs::format::bgr8, 30);
    dev->enable_stream(rs::stream::infrared, 640, 480, rs::format::y8, 30);
    dev->enable_stream(rs::stream::infrared2, 640, 480, rs::format::y8, 30);
    dev->enable_stream(rs::stream::fisheye, 640, 480, rs::format::raw8, 60);
    dev->set_option(rs::option::fisheye_strobe, 1);

    resolutions[rs::stream::depth] = { dev->get_stream_width(rs::stream::depth), dev->get_stream_height(rs::stream::depth), rs::format::z16 };
    resolutions[rs::stream::color] = { dev->get_stream_width(rs::stream::color), dev->get_stream_height(rs::stream::color), rs::format::rgb8 };
    resolutions[rs::stream::infrared] = { dev->get_stream_width(rs::stream::infrared), dev->get_stream_height(rs::stream::infrared), rs::format::y8 };
    resolutions[rs::stream::fisheye] = { dev->get_stream_width(rs::stream::fisheye), dev->get_stream_height(rs::stream::fisheye), rs::format::raw8 };

// 	rs::extrinsics color_to_imu = dev->get_motion_extrinsics_from(rs::stream::color);
// 	std::cout<< color_to_imu.rotation<<std::endl;
// 	std::cout<< color_to_imu.translation<<std::endl;
	
    auto frame_callback = [](rs::frame frame){
		if (frame.get_frame_timestamp_domain() != rs::timestamp_domain::microcontroller) return;
		char buffer[10];
		gcvt(frame.get_timestamp(),10,buffer);
		
		if (frame.get_stream_type() == rs::stream::depth)
		{
			fdepth << fixed << setprecision(2) << frame.get_timestamp() << " "
				<< buffer << ".png"
				<< endl;
			Mat image(Size(frame.get_width(),frame.get_height()), CV_16UC1, (void*)(frame.get_data()), Mat::AUTO_STEP);
			images[0].push_back(image.clone());
			name[0].push_back(str + "depth/" + buffer + ".png");
			name[5].push_back(str + "depthRGB/" + buffer + ".png");
			if (running && frames_queue[0].size() <= max_queue_size) frames_queue[0].enqueue(std::move(frame));
		}
		else if (frame.get_stream_type() == rs::stream::color)
		{
			fcolor << fixed << setprecision(2) << frame.get_timestamp() << " "
			<< buffer << ".png"
			<< endl;
			
			Mat image(Size(frame.get_width(),frame.get_height()), CV_8UC3, const_cast<void*>(frame.get_data()));
			images[1].push_back(image.clone());
			name[1].push_back(str + "color/" + buffer + ".png");

			if (running && frames_queue[1].size() <= max_queue_size) frames_queue[1].enqueue(std::move(frame));
		}
		else if (frame.get_stream_type() == rs::stream::infrared)
		{
			finf << fixed << setprecision(2) << frame.get_timestamp() << " "
			<< buffer << ".png"
				<< endl;
			Mat image(Size(frame.get_width(),frame.get_height()), CV_8UC1, (void*)(frame.get_data()), Mat::AUTO_STEP);
			images[2].push_back(image.clone());
			name[2].push_back(str + "infrared/" + buffer + ".png");
			if (running && frames_queue[2].size() <= max_queue_size) frames_queue[2].enqueue(std::move(frame));
		}
		else if (frame.get_stream_type() == rs::stream::infrared2)
		{
			finf2 << fixed << setprecision(2) << frame.get_timestamp() << " "
			<< buffer << ".png"
				<< endl;
			Mat image(Size(frame.get_width(),frame.get_height()), CV_8UC1, (void*)(frame.get_data()), Mat::AUTO_STEP);
			images[3].push_back(image.clone());
			name[3].push_back(str + "infrared2/" + buffer + ".png");
		}
		else if (frame.get_stream_type() == rs::stream::fisheye)
		{
			ffisheye << fixed << setprecision(2) << frame.get_timestamp() << " "
			<< buffer << ".png"
				<< endl;
	// 	    stbi_write_png((str + "lab/fisheye/" + buffer + ".png").c_str(),
	// 			    frame.get_width(),
	// 			    frame.get_height(),
	// 			    components_map[frame.get_stream_type()],
	// 			    frame.get_data(),
	// 			    frame.get_width() * components_map[frame.get_stream_type()]);
			name[4].push_back(str + "fisheye/" + buffer + ".png");
			Mat image(Size(frame.get_width(),frame.get_height()), CV_8UC1, (void*)(frame.get_data()), Mat::AUTO_STEP);
			images[4].push_back(image.clone());
			if (running && frames_queue[4].size() <= max_queue_size) frames_queue[4].enqueue(std::move(frame));
		}
    };

    for (int i = (int)(rs::stream::depth); i <= (int)(rs::stream::fisheye); i++)
	dev->set_frame_callback((rs::stream)i, frame_callback);
    
    //init viewer
    glfwInit();
    auto max_aspect_ratio = 0.0f;
    for (auto i : supported_streams)
    {
        auto aspect_ratio = static_cast<float>(resolutions[static_cast<rs::stream>(i)].height) / static_cast<float>(resolutions[static_cast<rs::stream>(i)].width);
        if (max_aspect_ratio < aspect_ratio)
            max_aspect_ratio = aspect_ratio;
    };
    auto win = glfwCreateWindow(1100, int(1100*max_aspect_ratio), "data collection", nullptr, nullptr);
    glfwMakeContextCurrent(win);
    
    // 3. Start generating motion-tracking data
    dev->start(rs::source::all_sources);
	namedWindow("Display Image", WINDOW_AUTOSIZE );
    while (!glfwWindowShouldClose(win))
    {
        glfwPollEvents();
        rs::frame frame;

        int w, h;
        glfwGetFramebufferSize(win, &w, &h);
        glViewport(0, 0, w, h);
        glClear(GL_COLOR_BUFFER_BIT);

        glfwGetWindowSize(win, &w, &h);
        glLoadIdentity();
        glOrtho(0, w, h, 0, -1, +1);

        for (auto i = 0; i < streams; i++)
        {
            if(!dev->supports(rs::capabilities(i))) continue;
			if(i==3)	continue;
			auto res = resolutions[(rs::stream)i];
			if (frames_queue[i].try_dequeue(&frame))
				buffers[i].upload(frame);

			if(i==4)
			{
				auto x = (3 % 2) * (w / 2);
				auto y = (3 / 2) * (h / 2);
				buffers[i].show(x, y, w / 2, h / 2, res.width, res.height);
			}
			else
			{
				auto x = (i % 2) * (w / 2);
				auto y = (i / 2) * (h / 2);
				buffers[i].show(x, y, w / 2, h / 2, res.width, res.height);
			}
            
        }

        glfwSwapBuffers(win);
    }
    glfwDestroyWindow(win);
    glfwTerminate();
    
// 	rs::extrinsics color_to_imu = dev->get_motion_extrinsics_from(rs::stream::color);
// 	std::cout<< color_to_imu.rotation<<std::endl;
// 	std::cout<< color_to_imu.translation<<std::endl;
    
/*	//get extrinsics	
	rs::extrinsics fisheye_to_imu = dev->get_motion_extrinsics_from(rs::stream::fisheye);


	rs::extrinsics color_to_fisheye = dev->get_extrinsics(rs::stream::color,rs::stream::fisheye);
// 	rs::extrinsics color_to_imu = dev->get_motion_extrinsics_from(rs::stream::color);
	
	Eigen::Matrix4f Mfisheye_to_imu;
	Eigen::Matrix4f Mcolor_to_fisheye;
	Eigen::Matrix4f Mcolor_to_imu;
	
	for(int ii=0;ii<3;ii++)
	{
		for(int jj=0;jj<3;jj++)
		{
			int n=ii*3+jj;
			Mfisheye_to_imu(ii,jj) = fisheye_to_imu.rotation[n];
			
			Mcolor_to_fisheye(ii,jj) = color_to_fisheye.rotation[n];
		}
		Mfisheye_to_imu(ii,3) = fisheye_to_imu.translation[ii];
		Mcolor_to_fisheye(ii,3) = color_to_fisheye.translation[ii];
		Mfisheye_to_imu(3,ii) = 0.0;
		Mcolor_to_fisheye(3,ii) = 0.0;
	}
	Mfisheye_to_imu(3,3) = 1.0;
	Mcolor_to_fisheye(3,3) = 1.0;
	
	Mcolor_to_imu = Mcolor_to_fisheye * Mfisheye_to_imu;
	cout<<Mfisheye_to_imu<<endl;
	cout<<Mcolor_to_fisheye<<endl;
	cout<<Mcolor_to_imu<<endl;
		*/
    // 4. stop data acquisition
    running = false;
    for (int i=0;i<5;i++) frames_queue[i].clear();
    dev->stop(rs::source::all_sources);
    // 5. reset previous settings formotion data handlers
    dev->disable_motion_tracking();

    //save images.
    for(int j=0; j<2;j++)
		for(int i=0;i<images[j].size();i++)
			imwrite(name[j][i],images[j][i]);
	int width = images[0][0].cols;
	int height = images[0][0].rows;
	for(int i=0; i<name[5].size();i++)
	{
		cv::Mat depth = cv::Mat::zeros(height,width,CV_8UC3);
		make_depth_histogram(depth,images[0][i],width,height);
		imwrite(name[5][i],depth);
	}
    faccel.close();
    fcolor.close();
    fdepth.close();
    ffisheye.close();
    fgyro.close();
	fimu.close();
	
    finf.close();
    finf2.close();
    //TODO:combine accelerometer and gyroscope
    
    cout<<"dataset collection complete"<<endl;

    return EXIT_SUCCESS;
}
catch (const rs::error & e)
{
    printf("rs::error was thrown when calling %s(%s):\n", e.get_failed_function().c_str(), e.get_failed_args().c_str());
    printf("    %s\n", e.what());
    return EXIT_FAILURE;
}