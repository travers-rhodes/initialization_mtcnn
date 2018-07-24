#include <iostream>
#include <math.h>       /*  isnan, sqrt */
#include "face_detector.hpp"
#include "helpers.hpp"
#include "ros/ros.h"
#include "initialization_mtcnn/InitializationSrvMsg.h"
#include "initialization_mtcnn/PointCloudMTCNNSrvMsg.h"
#include "initialization_mtcnn/MTCNNSrvMsg.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/shared_ptr.hpp>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <vector>
#include <array>

void pixelTo3DPoint(const sensor_msgs::PointCloud2 pCloud, const int u, const int v, geometry_msgs::Point &p)
{

  // get width and height of 2D point cloud data
  int width = pCloud.width;
  int height = pCloud.height;

  // Convert from u (column / width), v (row/height) to position in array
  // where X,Y,Z data starts
  int arrayPosition = v*pCloud.row_step + u*pCloud.point_step;

  // compute position in array where x,y,z data start
  int arrayPosX = arrayPosition + pCloud.fields[0].offset; // X has an offset of 0
  int arrayPosY = arrayPosition + pCloud.fields[1].offset; // Y has an offset of 4
  int arrayPosZ = arrayPosition + pCloud.fields[2].offset; // Z has an offset of 8

  float X = 0.0;
  float Y = 0.0;
  float Z = 0.0;

  memcpy(&X, &pCloud.data[arrayPosX], sizeof(float));
  memcpy(&Y, &pCloud.data[arrayPosY], sizeof(float));
  memcpy(&Z, &pCloud.data[arrayPosZ], sizeof(float));

  p.x = X;
  p.y = Y;
  p.z = Z;

}

mtcnn::FaceDetector fd("/home/candeias/code/mtcnn-cpp/model/", 0.6f, 0.7f, 0.7f, true, true, 0);

bool mtcnnRgbService(initialization_mtcnn::MTCNNSrvMsg::Request &req,
                        initialization_mtcnn::MTCNNSrvMsg::Response &res)
{
    sensor_msgs::Image rgb_image;
    rgb_image = req.input_rgb;
    
   	cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(rgb_image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return false;
    }

std::vector<mtcnn::Face> faces = fd.detect(cv_ptr->image, 40.f, 0.709f);
if(faces.size()< 1)
{
    ROS_ERROR("No face detected");
    return false;		
}


//fill message with pts coordinates
std_msgs::Float64MultiArray mtcnn_ptscoords;

mtcnn_ptscoords.layout.dim.push_back(std_msgs::MultiArrayDimension());    
mtcnn_ptscoords.layout.dim[0].label = "ptsCoords";
mtcnn_ptscoords.layout.dim[0].size = 10;
mtcnn_ptscoords.layout.dim[0].stride = 10;
mtcnn_ptscoords.layout.data_offset = 0;

std::vector<double> vec(10,0);
for (int i=0; i<10; i++)
{
    vec[i] = faces[0].ptsCoords[i];
}
mtcnn_ptscoords.data = vec;
res.mtcnn_ptscoords = mtcnn_ptscoords;

//fill message with box
std_msgs::Float64MultiArray mtcnn_ptsbbox;

mtcnn_ptsbbox.layout.dim.push_back(std_msgs::MultiArrayDimension());    
mtcnn_ptsbbox.layout.dim[0].label = "ptsBox";
mtcnn_ptsbbox.layout.dim[0].size = 4;
mtcnn_ptsbbox.layout.dim[0].stride = 4;
mtcnn_ptsbbox.layout.data_offset = 0;

std::vector<double> vec2(4,0);
vec2[0] = faces[0].bbox.x1;
vec2[1] = faces[0].bbox.y1;
vec2[2] = faces[0].bbox.x2;
vec2[3] = faces[0].bbox.y2;

mtcnn_ptsbbox.data = vec2;
res.mtcnn_ptsbbox = mtcnn_ptsbbox;

/*
//just for vizualization
std::vector<cv::Point> pts;
for (int p = 0; p < mtcnn::NUM_PTS; ++p) {
    pts.push_back(cv::Point(faces[0].ptsCoords[2 * p], faces[0].ptsCoords[2 * p + 1]));
}
std::cout << "score: "  << faces[0].score << std::endl;
drawAndShowFace(cv_ptr->image, faces[0].bbox.getRect(), pts);
*/
    ROS_INFO("RGB SERVICE CALLED");
    return true;

}

bool passThroughMtcnnServiceHandle(initialization_mtcnn::PointCloudMTCNNSrvMsg::Request &req,
            initialization_mtcnn::PointCloudMTCNNSrvMsg::Response &res)
{

    sensor_msgs::PointCloud2 pc;
    pc = req.input_pc;

    std::cout << "received message with pc" << std::endl; 
    
   	sensor_msgs::Image image;
	pcl::toROSMsg(pc,image);
	cv_bridge::CvImagePtr cv_ptr;
    	try
    	{
      		cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    	}
    	catch (cv_bridge::Exception& e)
    	{
      		ROS_ERROR("cv_bridge exception: %s", e.what());
      		return false;
    	}
	
	std::vector<mtcnn::Face> faces = fd.detect(cv_ptr->image, 40.f, 0.709f);
	if(faces.size()< 1)
	{
		ROS_ERROR("No face detected");
		return false;		
	}


    //fill message with pts coordinates
    std_msgs::Float64MultiArray mtcnn_ptscoords;

    mtcnn_ptscoords.layout.dim.push_back(std_msgs::MultiArrayDimension());    
    mtcnn_ptscoords.layout.dim[0].label = "ptsCoords";
    mtcnn_ptscoords.layout.dim[0].size = 10;
    mtcnn_ptscoords.layout.dim[0].stride = 10;
	mtcnn_ptscoords.layout.data_offset = 0;

    std::vector<double> vec(10,0);
    for (int i=0; i<10; i++)
    {
        vec[i] = faces[0].ptsCoords[i];
    }
    mtcnn_ptscoords.data = vec;
    res.mtcnn_ptscoords = mtcnn_ptscoords;

    //fill message with box
    std_msgs::Float64MultiArray mtcnn_ptsbbox;

    mtcnn_ptsbbox.layout.dim.push_back(std_msgs::MultiArrayDimension());    
    mtcnn_ptsbbox.layout.dim[0].label = "ptsBox";
    mtcnn_ptsbbox.layout.dim[0].size = 4;
    mtcnn_ptsbbox.layout.dim[0].stride = 4;
	mtcnn_ptsbbox.layout.data_offset = 0;

    std::vector<double> vec2(4,0);
    vec2[0] = faces[0].bbox.x1;
    vec2[1] = faces[0].bbox.y1;
    vec2[2] = faces[0].bbox.x2;
    vec2[3] = faces[0].bbox.y2;

    mtcnn_ptsbbox.data = vec2;
    res.mtcnn_ptsbbox = mtcnn_ptsbbox;

	/*
	//just for vizualization
	std::vector<cv::Point> pts;
	for (int p = 0; p < mtcnn::NUM_PTS; ++p) {
		pts.push_back(cv::Point(faces[0].ptsCoords[2 * p], faces[0].ptsCoords[2 * p + 1]));
	}
	std::cout << "score: "  << faces[0].score << std::endl;
	drawAndShowFace(cv_ptr->image, faces[0].bbox.getRect(), pts);
	*/

    return true;
}

bool initializationServiceHandle(initialization_mtcnn::InitializationSrvMsg::Request &req,
		  initialization_mtcnn::InitializationSrvMsg::Response &res)
{
	// get point cloud and convert it to pcl format
	boost::shared_ptr<sensor_msgs::PointCloud2 const> msg_sharedptr; 
	sensor_msgs::PointCloud2 pc;
	

	msg_sharedptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/depth_registered/points");
        if (msg_sharedptr == NULL)
	{
		ROS_INFO("No point Cloud");
		return false;
	}else
	{
		pc = *msg_sharedptr;
	}
		
	sensor_msgs::Image image;
	pcl::toROSMsg(pc,image);
	cv_bridge::CvImagePtr cv_ptr;
    	try
    	{
      		cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    	}
    	catch (cv_bridge::Exception& e)
    	{
      		ROS_ERROR("cv_bridge exception: %s", e.what());
      		return false;
    	}
	
	std::vector<mtcnn::Face> faces = fd.detect(cv_ptr->image, 40.f, 0.709f);
	if(faces.size()< 1)
	{
		ROS_ERROR("No face detected");
		return false;		
	}
	
	/*
	//just for vizualization
	std::vector<cv::Point> pts;
	for (int p = 0; p < mtcnn::NUM_PTS; ++p) {
		pts.push_back(cv::Point(faces[0].ptsCoords[2 * p], faces[0].ptsCoords[2 * p + 1]));
	}
	std::cout << "score: "  << faces[0].score << std::endl;
	drawAndShowFace(cv_ptr->image, faces[0].bbox.getRect(), pts);
	*/

	geometry_msgs::Point point_tmp;
	int u;
	int v;
	float p_x = 0;
	float p_y = 0;
	float p_z= 0;
	for(int p = 0; p < mtcnn::NUM_PTS; ++p)
	{
		u = faces[0].ptsCoords[2 * p];
		v = faces[0].ptsCoords[2 * p + 1];	
		pixelTo3DPoint(pc, u , v , point_tmp);
		if( std::isnan(point_tmp.x) || std::isnan(point_tmp.y) || std::isnan(point_tmp.z)  )
		{
			continue;
		}
		p_x = p_x + point_tmp.x;
		p_y = p_y + point_tmp.y;
		p_z = p_z + point_tmp.z;
	}

	if(p_x == 0)
	{ 
		ROS_ERROR("No points");
		return false;
	}

	geometry_msgs::Point point;
	point.x = p_x / 5.0;
	point.y = p_y / 5.0;
	point.z = p_z / 5.0;
	res.initial_translation = point;
	return true;	

}

int main(int argc, char **argv)
{
	
	ros::init(argc,argv,"initialization_mtcnn_server");
	ros::NodeHandle n;

	ros::ServiceServer service_initialization = n.advertiseService("initialization_mtcnn",
		       			initializationServiceHandle);
	ROS_INFO("Initialization Service Ready");
	
    ros::ServiceServer service_mtcnn = n.advertiseService("mtcnn_pc2",
            passThroughMtcnnServiceHandle);
    ROS_INFO("MTCNN Point Cloud 2 service ready");

    ros::ServiceServer service_rgb_mtcnn = n.advertiseService("mtcnn_rgb", mtcnnRgbService );
    ROS_INFO("MTCNN RGB service ready");

	ros::spin();
	return 0;
}
