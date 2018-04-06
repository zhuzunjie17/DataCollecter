// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#include <librealsense/rs.hpp>
#include <iostream>
#include <iomanip>
#include <vector>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
int main() try
{
    rs::log_to_console(rs::log_severity::warn);
    //rs::log_to_file(rs::log_severity::debug, "librealsense.log");

    // Obtain a list of devices currently present on the system
    rs::context ctx;
    int device_count = ctx.get_device_count();   
    if (!device_count) printf("No device detected. Is it plugged in?\n");

    for(int i = 0; i < device_count; ++i)
    {
        // Show the device name and information
        rs::device * dev = ctx.get_device(i);
        std::cout << "Device " << i << " - " << dev->get_name() << ":\n";
        std::cout << " Serial number: " << dev->get_serial() << "\n";
        std::cout << " Firmware version: " << dev->get_firmware_version() << "\n";
        try { std::cout << " USB Port ID: " << dev->get_usb_port_id() << "\n"; } catch (...) {}
        if (dev->supports(rs::capabilities::adapter_board)) std::cout << " Adapter Board Firmware version: " << dev->get_info(rs::camera_info::adapter_board_firmware_version) << "\n";
        if (dev->supports(rs::capabilities::motion_events)) std::cout << " Motion Module Firmware version: " << dev->get_info(rs::camera_info::motion_module_firmware_version) << "\n";



        // Show which options are supported by this device
        std::cout << " Camera info: \n";
        for (int j = RS_CAMERA_INFO_DEVICE_NAME; j < RS_CAMERA_INFO_COUNT; ++j)
        {
            rs::camera_info param = (rs::camera_info)j;
            if (dev->supports(param))
                std::cout << "    " << std::left << std::setw(20) << rs_camera_info_to_string(rs_camera_info(param)) << ": \t" << dev->get_info(param) << std::endl;
        }



        // Show which options are supported by this device
        std::cout << std::setw(55) << " Supported options:" << std::setw(10) << "min"  << std::setw(10) << " max" << std::setw(6) << " step" << std::setw(10) << " default" << std::endl;
        for(int j = 0; j < RS_OPTION_COUNT; ++j)
        {
            rs::option opt = (rs::option)j;
            if(dev->supports_option(opt))
            {
                double min, max, step, def;
                dev->get_option_range(opt, min, max, step, def);
                std::cout   << "    " << std::left << std::setw(50)  << opt << " : " << std::setw(5) << min << "... " << std::setw(12) << max << std::setw(6) << step << std::setw(10) << def << "\n";
            }
        }

        // Show which streams are supported by this device
        for(int j = 0; j < RS_STREAM_COUNT; ++j)
        {
            // Determine number of available streaming modes (zero means stream is unavailable) 
            rs::stream strm = (rs::stream)j;
            int mode_count = dev->get_stream_mode_count(strm);
            if(mode_count == 0) continue;

            // Show each available mode for this stream
            std::cout << " Stream " << strm << " - " << mode_count << " modes:\n";
            for(int k = 0; k < mode_count; ++k)
            {
                // Show width, height, format, and framerate, the settings required to enable the stream in this mode
                int width, height, framerate;
                rs::format format;
                dev->get_stream_mode(strm, k, width, height, format, framerate);
                std::cout << "  " << width << "\tx " << height << "\t@ " << framerate << "Hz\t" << format;

                // Enable the stream in this mode so that we can retrieve its intrinsics
                dev->enable_stream(strm, width, height, format, framerate);
                rs::intrinsics intrin = dev->get_stream_intrinsics(strm);

                // Show horizontal and vertical field of view, in degrees
                std::cout << "\t" << std::setprecision(3) << intrin.hfov() << " x " << intrin.vfov() << " degrees, distortion = " << intrin.model() << std::endl;
            }

            // Some stream mode combinations are invalid, so disable this stream before moving on to the next one
            dev->disable_stream(strm);
        }

        //Show camera intrinces and extrinces
        auto motion_callback = [](rs::motion_data entry)
        {
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
        dev->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 30);
        dev->enable_stream(rs::stream::color, 640, 480, rs::format::bgr8, 30);
        dev->enable_stream(rs::stream::infrared, 640, 480, rs::format::y8, 30);
        dev->enable_stream(rs::stream::infrared2, 640, 480, rs::format::y8, 30);
        dev->enable_stream(rs::stream::fisheye, 640, 480, rs::format::raw8, 60);
        dev->set_option(rs::option::fisheye_strobe, 1);

        std::cout << std::endl;
        std::cout << std::setw(20) << "device intrinces:" << std::setw(25) << "model"
                  << std::setw(7) << "hfov" << std::setw(10) << "vfov" << std::setw(7) << "width" << std::setw(7) << "height"
                  << std::setw(7) << "fx"  << std::setw(7) << "fy" << std::setw(7) << "ppx" << std::setw(7) << "ppy" << "coeffs"
                  << std::endl;
        rs::intrinsics color = dev->get_stream_intrinsics(rs::stream::color);
        rs::intrinsics depth = dev->get_stream_intrinsics(rs::stream::depth);
        rs::intrinsics fisheye = dev->get_stream_intrinsics(rs::stream::fisheye);
        rs::intrinsics infrared = dev->get_stream_intrinsics(rs::stream::infrared);
        rs::intrinsics infrared2 = dev->get_stream_intrinsics(rs::stream::infrared);
        rs::motion_intrinsics imu = dev->get_motion_intrinsics();


        std::cout << std::setw(20) << "color:" << std::setw(25) << color.model()
                  << std::setw(7)  << color.hfov() << std::setw(10) << color.vfov() << std::setw(7) << color.width << std::setw(7) << color.height
                  << std::setw(7) << color.fx  << std::setw(7) << color.fy << std::setw(7) << color.ppx << std::setw(7) << color.ppy
                  << color.coeffs[0] <<" "<<color.coeffs[1] <<" "<<color.coeffs[2] <<" "<<color.coeffs[3] <<" "<<color.coeffs[4]
                  << std::endl;
        std::cout << std::setw(20) << "depth:" << std::setw(25) << depth.model()
                  << std::setw(7)  << depth.hfov() << std::setw(10) << depth.vfov() << std::setw(7) << depth.width << std::setw(7) << depth.height
                  << std::setw(7) << depth.fx  << std::setw(7) << depth.fy << std::setw(7) << depth.ppx << std::setw(7) << depth.ppy
                  << depth.coeffs[0] <<" "<<depth.coeffs[1] <<" "<<depth.coeffs[2] <<" "<<depth.coeffs[3] <<" "<<depth.coeffs[4]
                  << std::endl;
        std::cout << std::setw(20) << "fisheye:" << std::setw(25) << fisheye.model()
                  << std::setw(7)  << fisheye.hfov() << std::setw(10) << fisheye.vfov() << std::setw(7) << fisheye.width << std::setw(7) << fisheye.height
                  << std::setw(7) << fisheye.fx  << std::setw(7) << fisheye.fy << std::setw(7) << fisheye.ppx << std::setw(7) << fisheye.ppy
                  << fisheye.coeffs[0] <<" "<<fisheye.coeffs[1] <<" "<<fisheye.coeffs[2] <<" "<<fisheye.coeffs[3] <<" "<<fisheye.coeffs[4]
                  << std::endl;
        std::cout << std::setw(20) << "infrared:" << std::setw(25) << infrared.model()
                  << std::setw(7)  << infrared.hfov() << std::setw(10) << infrared.vfov() << std::setw(7) << infrared.width << std::setw(7) << infrared.height
                  << std::setw(7) << infrared.fx  << std::setw(7) << infrared.fy << std::setw(7) << infrared.ppx << std::setw(7) << infrared.ppy
                  << infrared.coeffs[0] <<" "<<infrared.coeffs[1] <<" "<<infrared.coeffs[2] <<" "<<infrared.coeffs[3] <<" "<<infrared.coeffs[4]
                  << std::endl;
        std::cout << std::setw(20) << "infrared2:" << std::setw(25) << infrared2.model()
                  << std::setw(7)  << infrared2.hfov() << std::setw(10) << infrared2.vfov() << std::setw(7) << infrared2.width << std::setw(7) << infrared2.height
                  << std::setw(7) << infrared2.fx  << std::setw(7) << infrared2.fy << std::setw(7) << infrared2.ppx << std::setw(7) << infrared2.ppy
                  << infrared2.coeffs[0] <<" "<<infrared2.coeffs[1] <<" "<<infrared2.coeffs[2] <<" "<<infrared2.coeffs[3] <<" "<<infrared2.coeffs[4]
                  << std::endl;
        std::cout << "IMU:" << std::endl
                            << " acc.bias_variances " << imu.acc.bias_variances[0] << " " << imu.acc.bias_variances[1] << " " << imu.acc.bias_variances[2] << std::endl
                            << " acc.noise_variances " << imu.acc.noise_variances[0] << " " << imu.acc.noise_variances[1] << " " << imu.acc.noise_variances[2] << std::endl
                            << " gyro.bias_variances " << imu.gyro.bias_variances[0] << " " << imu.gyro.bias_variances[1] << " " << imu.gyro.bias_variances[2] << std::endl
                            << " gyro.noise_variances "<< imu.gyro.noise_variances[0] << " "<< imu.gyro.noise_variances[1] << " "<< imu.gyro.noise_variances[2] << std::endl;


        rs::extrinsics color2depth = dev->get_extrinsics(rs::stream::color, rs::stream::depth);
        rs::extrinsics depth2color = dev->get_extrinsics(rs::stream::depth, rs::stream::color);
        rs::extrinsics color2fisheye = dev->get_extrinsics(rs::stream::color, rs::stream::fisheye);
        rs::extrinsics fisheye2color = dev->get_extrinsics(rs::stream::fisheye, rs::stream::color);
        rs::extrinsics fisheye2imu = dev->get_motion_extrinsics_from(rs::stream::fisheye);

        std::cout << std::setw(20) << "device extrinces:" << std::endl;


        Eigen::Matrix4f c2d;
        Eigen::Matrix4f d2c;
        Eigen::Matrix4f c2f;
        Eigen::Matrix4f f2c;
        Eigen::Matrix4f f2i;
        Eigen::Matrix4f c2i;
        for(int ii=0;ii<3;ii++)
        {
            for(int jj=0;jj<3;jj++)
            {
                int n=ii*3+jj;
                c2d(ii,jj) = color2depth.rotation[n];
                d2c(ii,jj) = depth2color.rotation[n];
                c2f(ii,jj) = color2fisheye.rotation[n];
                f2c(ii,jj) = fisheye2color.rotation[n];
                f2i(ii,jj) = fisheye2imu.rotation[n];
            }
            c2d(ii,3) = color2depth.translation[ii];
            d2c(ii,3) = depth2color.translation[ii];
            c2f(ii,3) = color2fisheye.translation[ii];
            f2c(ii,3) = fisheye2color.translation[ii];
            f2i(ii,3) = fisheye2imu.translation[ii];

            c2d(3,ii) = 0.0;
            d2c(3,ii) = 0.0;
            c2f(3,ii) = 0.0;
            f2c(3,ii) = 0.0;
            f2i(3,ii) = 0.0;

        }
        c2d(3,3) = 1.0;
        d2c(3,3) = 1.0;
        c2f(3,3) = 1.0;
        f2c(3,3) = 1.0;
        f2i(3,3) = 1.0;

        c2i = c2f * f2i;
        std::cout << std::setprecision (6); // 精度
        std::cout.setf(std::ios::fixed,std::ios::floatfield); // 定点格式
        std::cout << "color2depth:" << std::endl<< c2d << std::endl;
        std::cout << "color2fisheye:" << std::endl<< c2f << std::endl;
        std::cout << "fisheye2imu:" << std::endl<< f2i << std::endl;
        std::cout << "color2imu:" << std::endl<< c2i << std::endl;
        Eigen::Matrix3f rot= c2i.block(0,0,3,3);
        Eigen::Quaternionf qr(rot);
        rot = qr.inverse().normalized().toRotationMatrix();
        std::cout << "q_color2imu: " << std::endl << qr.vec() << std::endl<< qr.w()<<std::endl;

    }
    return EXIT_SUCCESS;
}
catch(const rs::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
