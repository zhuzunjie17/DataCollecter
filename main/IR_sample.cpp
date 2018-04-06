// include the librealsense C++ header file
#include <librealsense/rs.hpp>

// include OpenCV header file
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main()
{
	rs::context ctx;
	rs::device * dev = ctx.get_device(0);
	
	// Configure Infrared stream to run at VGA resolution at 30 frames per second
	dev->enable_stream(rs::stream::infrared, 640, 480, rs::format::y8, 30);
	
	// We must also configure depth stream in order to IR stream run properly
	dev->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 30);
	
	// Start streaming
	dev->start();
	
	// Camera warmup - Dropped frames to allow stabilization
	for(int i = 0; i < 40; i++)
		dev->wait_for_frames();
	
	// Creating OpenCV matrix from IR image
	Mat ir(Size(640, 480), CV_8UC1, (void*)dev->get_frame_data(rs::stream::infrared), Mat::AUTO_STEP);
	
	// Apply Histogram Equalization
	equalizeHist( ir, ir );
	applyColorMap(ir, ir, COLORMAP_JET);
	
	// Display the image in GUI
	namedWindow("Display Image", WINDOW_AUTOSIZE );
	imshow("Display Image", ir);
	
	waitKey(0);
	
	return 0;
}