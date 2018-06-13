
#include <ros/ros.h>
#include "pointcloud_registration_server/registration_service.h"
#include "pointcloud_registration_server/pointcloud_registration.h"

// -------------------------------------------
// *** Service Call *** 
//   Not a part of the below class, so that the service itself doesn't have to be templated 
//   This means that subsequent runs can be made with the same instantiation, without restarting the service node 
//   For now, going to include point type specifications in the service object as the same #DEFINES listed above 
bool registerPointclouds(pointcloud_registration_server::registration_service::Request& req, pointcloud_registration_server::registration_service::Response& res)
{	
	// --------------------- Starting service ---------------------
	//   Initialize clouds, set the start time
	ros::Time callback_received_time = ros::Time::now();
	ROS_DEBUG_STREAM("[PCRegistration] Received service callback.");

	// This implementation is HORRIBLE and I hate it... but it's the best way I could think of to wrap this templated class in a ROS service
	// Very open to comments on how to avoid this nasty nested switch statement business

	switch(req.point_type)
	{
		case pointcloud_registration_server::registration_service::Request::POINT_TYPE_XYZ:
		{
			switch(req.feature_type)
			{
				case pointcloud_registration_server::registration_service::Request::FEATURE_TYPE_NONE:
				{
					//PCRegistration<pcl::PointXYZ, pcl::PointXYZ> registration(req);
					break;
				}
				case pointcloud_registration_server::registration_service::Request::FEATURE_TYPE_XYZN:
				{
					PCRegistration<pcl::PointXYZ, pcl::PointNormal> registration(req);
					break;
				}
				case pointcloud_registration_server::registration_service::Request::FEATURE_TYPE_SIFT:
				{

					break;
				}
				default:
				{
					ROS_ERROR_STREAM("[PCRegistration] Feature type not specified properly - exiting service as a failure.");
					return false;
				}
			}
			break;
		}
		case pointcloud_registration_server::registration_service::Request::POINT_TYPE_XYZI:
		{

			break;
		}
		case pointcloud_registration_server::registration_service::Request::POINT_TYPE_XYZN:
		{

			break;
		}
		default:
		{
			ROS_ERROR_STREAM("[PCRegistration] Point type not specified properly - exiting service as a failure.");
			return false;
		}
	}
	return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pc_registration");

	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    	ros::console::notifyLoggerLevelsChanged();

	ROS_DEBUG_STREAM("[PCRegistration] Started up node.");

	ros::NodeHandle nh;
	ros::ServiceServer server = nh.advertiseService("pointcloud_registration", &registerPointclouds);

	ros::spin();
}