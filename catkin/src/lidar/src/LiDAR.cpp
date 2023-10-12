#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <sstream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include "HPS3DUser_IF.h"
#include "ObjectDetection/ObjectDetection.h"
#include "Visualize/Visualize.h"
#include "DownsampleAndFilter/DownsampleAndFilter.h"

#define _GLIBCXX_USE_CXX11_ABI 0

// Define the frame buffer vector
// Must be global due to being used in an interrupt
typedef struct
{
    std::vector<HPS3D_PerPointCloudData_t> points;
    uint16_t width;
    uint16_t height;
} PointCloudData_t;

std::vector<pcl::PointCloud<pcl::PointXYZ>> frameBuffer;

// Global variable which stores the current measurement data
// Must be global due to being used in an interrupt
static HPS3D_MeasureData_t g_measureData;

// LiDAR event ISR
// I don't know anything about the parameters except that data is a pointer to a HPS3D_MeasureData_t struct
// All this does is add the new data to the frame buffer and throws a warning if anything else happens
static void EventCallBackFunc(int handle, int eventType, uint8_t *data, int dataLen, void *userPara);

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "LiDAR");
    ros::NodeHandle n;

    // Initialize test publisher
    ros::Publisher int_pub = n.advertise<std_msgs::Int32>("Chatter", 1000);

    // Tell the user we're starting
    ROS_INFO("Starting...");

    // Start initializing the LiDAR
    // Throw errors and return if initialization failed
    int handle = -1;
    HPS3D_StatusTypeDef ret = HPS3D_RET_OK;

    ret = HPS3D_MeasureDataInit(&g_measureData);
    if (ret != HPS3D_RET_OK)
    {
        ROS_FATAL("MeasureDataInit failed, Err: %d", ret);
        return 0;
    }

    ROS_INFO("MeasureDataInit success!");

    bool lidar_connected = false;

    const char* acm_paths[] = {"/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyACM3"};

    for (const char* acm_path : acm_paths)
	{
    	char acm_path_copy[32]; // Allocate a character array for the copy
    	strncpy(acm_path_copy, acm_path, sizeof(acm_path_copy)); // Copy the string
    	acm_path_copy[sizeof(acm_path_copy) - 1] = '\0'; // Ensure null-terminated

    	ret = HPS3D_USBConnectDevice(acm_path_copy, &handle);
    	if (ret == HPS3D_RET_OK)
    	{
        	lidar_connected = true;
        	break; // Connected successfully, break out of the loop
    	}
	}

    if (!lidar_connected)
    {
        ROS_FATAL("Failed to connect to LiDAR on all ACM devices");
        return 0;
    }

    ret = HPS3D_RegisterEventCallback(EventCallBackFunc, NULL);
    if (ret != HPS3D_RET_OK)
    {
        ROS_FATAL("Register Callback Error: %d", ret);
        return 0;
    }

    HPS3D_DeviceSettings_t settings;
    ret = HPS3D_ExportSettings(handle, &settings);
    if (ret != HPS3D_RET_OK)
    {
        ROS_FATAL("Export Settings Error: %d", ret);
        return 0;
    }

    ROS_INFO("Connected!");
    ROS_INFO("Max resolution: %dX\t%dY", settings.max_resolution_X, settings.max_resolution_Y);

    // Start continuous capture
    HPS3D_StartCapture(handle);

    // Create an instance of ObjectDetection
    ObjectDetection object_detection;

    // Create an instance of Visualize
    Visualize visualize;

    // Create an instance of DownsampleAndFilter
    DownsampleAndFilter downsample_filter;

    // Run the main loop
    ros::Rate loop_rate(40);
    int count = 0;
    while (ros::ok())
    {
        // Process LiDAR data and detect objects
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // Implement code to convert frameBuffer data to input_cloud

        // Downsample the LiDAR data
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointXYZ);

        downsample_filter.downsample(input_cloud, downsampled_cloud);

        // Perform object detection
        object_detection.detectObjects(downsampled_cloud);

        // Set the input point cloud and detected objects for visualization
        visualize.setInputCloud(downsampled_cloud);
        visualize.setDetectedObjects(object_detection.getDetectedObjects());

        // Visualize the point cloud and detected objects
        visualize.visualizePointCloud();

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
static void EventCallBackFunc(int handle, int eventType, uint8_t *data, int dataLen, void *userPara)
{
	// Initialize point cloud structures
	pcl::PointCloud<pcl::PointXYZ> point_data;
	HPS3D_PerPointCloudData_t *dataptr; // This pointer is used to create the point_data.points vector from the raw LiDAR data

	switch ((HPS3D_EventType_t)eventType)
	{

	// This appears to be the only event callback we normally get. Use this tog et data
	case HPS3D_FULL_DEPTH_EVEN:
		// Read the data into g_measureData
		HPS3D_ConvertToMeasureData(data, &g_measureData, (HPS3D_EventType_t)eventType);

		// Debug logging
		ROS_INFO("Average distance: %d", g_measureData.full_depth_data.distance_average);
		ROS_INFO("Width: %d\tHeight: %d\tPoints: %u", g_measureData.full_depth_data.point_cloud_data.width, g_measureData.full_depth_data.point_cloud_data.height, g_measureData.full_depth_data.point_cloud_data.points);

		// Create the point_data struct
		dataptr = g_measureData.full_depth_data.point_cloud_data.point_data;
		for (int i = 0; i < g_measureData.full_depth_data.point_cloud_data.points; i++)
		{
			point_data.push_back(pcl::PointXYZ(dataptr[i].x, dataptr[i].y, dataptr[i].z));
		}
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
