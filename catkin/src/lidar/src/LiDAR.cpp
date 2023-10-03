#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <sstream>
#include <vector>
#include <pcl/point_types.h>
#include "HPS3DUser_IF.h"


// Define the frame buffer vector
// Must be global due to being used in an interrupt
typedef struct{
	std::vector<HPS3D_PerPointCloudData_t> points;
	uint16_t width;
	uint16_t height;
} PointCloudData_t;

std::vector<PointCloudData_t> frameBuffer;

// Global variable which stores the current measurement data
// Must be global due to being used in an interrupt
static HPS3D_MeasureData_t g_measureData;


// LiDAR event ISR
// I dont know anything about the parameters except that data is a pointer to a HPS3D_MeasureData_t struct
// All this does is add the new data to the frame buffer and throws a warning if anything else happens
static void EventCallBackFunc(int handle, int eventType, uint8_t *data,int dataLen, void *userPara);

int main(int argc, char** argv){
	// Initialize ros node
	ros::init(argc, argv, "LiDAR");
	ros::NodeHandle n;

	// Initialize test publisher
	ros::Publisher int_pub = n.advertise<std_msgs::Int32>("Chatter", 1000);

	// Tell user were starting
	ROS_INFO("Starting...");

	// Start initializing the LiDAR
	// Throw errors and return if initialization failed
	int handle = -1;
	HPS3D_StatusTypeDef ret = HPS3D_RET_OK;

	ret = HPS3D_MeasureDataInit(&g_measureData);
	if(ret != HPS3D_RET_OK){
		ROS_FATAL("MeasureDataInit failed, Err: %d", ret);
		return 0;
	}

	ROS_INFO("MeasureDataInit success!");

	ret = HPS3D_USBConnectDevice((char*)"/dev/ttyACM0", &handle);
	if(ret != HPS3D_RET_OK){
		ROS_FATAL("USB Connect error: %d", ret);
		return 0;
	}


	ret = HPS3D_RegisterEventCallback(EventCallBackFunc, NULL);
	if(ret != HPS3D_RET_OK){
		ROS_FATAL("Register Callback Error: %d", ret);
		return 0;
	}

	HPS3D_DeviceSettings_t settings;
	ret = HPS3D_ExportSettings(handle, &settings);
	if(ret != HPS3D_RET_OK){
		ROS_FATAL("Export Settings Error: %d", ret);
		return 0;
	}

	ROS_INFO("Connected!");
	ROS_INFO("Max resolution: %dX\t%dY", settings.max_resolution_X, settings.max_resolution_Y);

	// Start continuous capture
	HPS3D_StartCapture(handle);

	// Run main loop at 20 Hz
	ros::Rate loop_rate(40);

	int count = 0;
	while(ros::ok()){

		std_msgs::Int32 msg;

		msg.data = count;
		int_pub.publish(msg);

		ros::spinOnce();

		count++;
		loop_rate.sleep();
	}

	// Cleanup ROS and LiDAR
	ROS_INFO("Shutting down...");
	HPS3D_StopCapture(handle);
	HPS3D_CloseDevice(handle);

	return 0;
	
}



static void EventCallBackFunc(int handle, int eventType, uint8_t *data,int dataLen, void *userPara){
	// Initialize point cloud structures
	PointCloudData_t point_data;
	HPS3D_PerPointCloudData_t* dataptr; // This pointer is used to create the point_data.points vector from the raw LiDAR data

	switch((HPS3D_EventType_t)eventType){

		// This appears to be the only event callback we normally get. Use this tog et data
		case HPS3D_FULL_DEPTH_EVEN:	
			// Read the data into g_measureData
			HPS3D_ConvertToMeasureData(data, &g_measureData, (HPS3D_EventType_t)eventType);

			// Debug logging
			ROS_INFO("Average distance: %d", g_measureData.full_depth_data.distance_average);
			ROS_INFO("Width: %d\tHeight: %d\tPoints: %u", g_measureData.full_depth_data.point_cloud_data.width, g_measureData.full_depth_data.point_cloud_data.height, g_measureData.full_depth_data.point_cloud_data.points);

			// Create the point_data struct
			dataptr = g_measureData.full_depth_data.point_cloud_data.point_data;
			point_data.points = std::vector<HPS3D_PerPointCloudData_t>(dataptr, dataptr + g_measureData.full_depth_data.point_cloud_data.points);
			point_data.width = g_measureData.full_depth_data.point_cloud_data.width;
			point_data.height = g_measureData.full_depth_data.point_cloud_data.height;
			
			// Add the data to the frame buffer
			frameBuffer.push_back(point_data);
			
			break;

		// These also contain data but they were never used in my tests so just leave them as a warning for now
		case HPS3D_SIMPLE_ROI_EVEN:	
		case HPS3D_FULL_ROI_EVEN:	
		case HPS3D_SIMPLE_DEPTH_EVEN:
			ROS_WARN("Wrong data");

		case HPS3D_SYS_EXCEPTION_EVEN: /*系统异常通知事件*/
			ROS_WARN("SYS ERR :%s", data);
			break;

		case HPS3D_DISCONNECT_EVEN: /*连接异常断开通知事件*/
			ROS_ERROR("Device disconnected!");
			HPS3D_StopCapture(handle);
			break;


	}

}
