
#include <ros/ros.h>
#include "pointcloud_registration_server/registration_service.h"
#include "pointcloud_registration_server/pointcloud_registration.h"

// -------------------------------------------
// *** Service Call *** 
//   Not a part of the PCRegistration class, so that the service itself doesn't have to be templated 
//   This means that subsequent runs using different types can be made without restarting the service node 
//   Obviously this comes at the slight cost of re-declaring and instantiating the class for every service call
//   I can't imagine that cost will ever be meaningful relative to the cost of actual registration though 
bool registerPointclouds(pointcloud_registration_server::registration_service::Request& req, pointcloud_registration_server::registration_service::Response& res)
{	
	// --------------------- Starting service ---------------------
	//   Initialize clouds, set the start time
	ros::Time callback_received_time = ros::Time::now();
	ROS_DEBUG_STREAM("[PCRegistration] Received service callback.");

	// This implementation is HORRIBLE and I hate it... but it's the best way I could think of to wrap this twice-templated class in a ROS service
	//   Very open to comments on how to avoid this nasty nested switch statement business
	//   Cannot use explicit template specialization because not all functions can be defined on all point types (since they have different fields)
	//   Functions within the class have to be implicitly declared for specific cases of point and feature types to prevent compilation errors  
	switch(req.point_type)
	{
		case pointcloud_registration_server::registration_service::Request::POINT_TYPE_XYZ:
		{
			switch(req.feature_type)
			{
				case pointcloud_registration_server::registration_service::Request::FEATURE_TYPE_XYZ:
				{
					PCRegistration<pcl::PointXYZ, pcl::PointXYZ> registration(req, res);
					break;
				}
				case pointcloud_registration_server::registration_service::Request::FEATURE_TYPE_XYZN:
				{
					PCRegistration<pcl::PointXYZ, pcl::PointNormal> registration(req, res);
					break;
				}
				case pointcloud_registration_server::registration_service::Request::FEATURE_TYPE_SIFT:
				{

					break;
				}
				case pointcloud_registration_server::registration_service::Request::FEATURE_TYPE_ITWF:
				{
					PCRegistration<pcl::PointXYZ, pcl::ITWFSignature84> registration(req, res);
					break;
				}
				default:
				{
					ROS_ERROR_STREAM("[PCRegistration] Feature and Point type chosen not implemented. Requested types: " << req.point_type << " " << req.feature_type << ". Exiting service as failure.");
					return false;
				}
			}
			break;
		}
		case pointcloud_registration_server::registration_service::Request::POINT_TYPE_XYZI:
		{

			switch(req.feature_type)
			{
				case pointcloud_registration_server::registration_service::Request::FEATURE_TYPE_XYZI:
				{
					PCRegistration<pcl::PointXYZI, pcl::PointXYZI> registration(req, res);
					break;
				}
				case pointcloud_registration_server::registration_service::Request::FEATURE_TYPE_XYZNI:
				{
					PCRegistration<pcl::PointXYZI, pcl::PointXYZINormal> registration(req, res);
					break;
				}
				case pointcloud_registration_server::registration_service::Request::FEATURE_TYPE_SIFT:
				{

					break;
				}
				case pointcloud_registration_server::registration_service::Request::FEATURE_TYPE_ITWF:
				{
					PCRegistration<pcl::PointXYZ, pcl::ITWFSignature84> registration(req, res);
					break;
				}
				default:
				{
					ROS_ERROR_STREAM("[PCRegistration] Feature and Point type chosen not implemented. Requested types: " << req.point_type << " " << req.feature_type << ". Exiting service as failure.");
					return false;
				}
			}
			break;
		}
		case pointcloud_registration_server::registration_service::Request::POINT_TYPE_XYZN:
		{

			break;
		}
		case pointcloud_registration_server::registration_service::Request::POINT_TYPE_WALL_DAMAGE:
		{
			switch(req.feature_type)
			{
				case pointcloud_registration_server::registration_service::Request::FEATURE_TYPE_XYZI:
				{
					PCRegistration<pcl::PointXYZI, pcl::PointXYZI> registration(req, res);
					break;
				}
				case pointcloud_registration_server::registration_service::Request::FEATURE_TYPE_XYZNI:
				{
					PCRegistration<pcl::PointXYZI, pcl::PointXYZINormal> registration(req, res);
					break;
				}
				case pointcloud_registration_server::registration_service::Request::FEATURE_TYPE_SIFT:
				{

					break;
				}
				case pointcloud_registration_server::registration_service::Request::FEATURE_TYPE_ITWF:
				{
					PCRegistration<pcl::PointWallDamage, pcl::ITWFSignature84> registration(req, res);
					break;
				}
			}
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