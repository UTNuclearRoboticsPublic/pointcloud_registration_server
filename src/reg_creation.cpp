
#include "pointcloud_registration_server/reg_creation.h"

namespace RegCreation
{
	// *** Overall Framework ***
	//   This is the function called by external users to populate a service.
	//   Calls other subfunctions within this library.
	bool registrationFromYAML(pointcloud_registration_server::registration_service *srv, std::string yaml_file_name);

	// *** High Level Process Definition ***
	//   Finds input choices (as strings) for Point, Feature, Interest Point, Correspondence, and Transform Types
	//   Maps those strings to int codes for registration types (see  /srv/registration_service.srv )
	bool basicTypeSpecification(pointcloud_registration_server::registration_service *srv, std::string yaml_file_name, std::string name, ros::NodeHandle nh);

	// *** Transform-specific Parameter Loading ***
	//   Loads parameters for Transformation Methods (ie ICP, ICPNL, NDT...)
	//   These parameters are different for different kinds of search methods, although some are common across methods.
	//   The process-specific parameters are saved in a vector of floats within the service. 
	bool transformSpecificParameters(pointcloud_registration_server::registration_service *srv, std::string yaml_file_name, std::string name, ros::NodeHandle nh);

};

bool RegCreation::registrationFromYAML(pointcloud_registration_server::registration_service *srv, std::string yaml_file_name)
{
//  *** Uncomment the following lines to enable debugging output ***
//	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
//    ros::console::notifyLoggerLevelsChanged();  

	ros::NodeHandle nh;
	ROS_DEBUG_STREAM("[RegCreation] Beginning to attempt to initialize parameters from server for registration " + yaml_file_name + ".");

	// -------------------------------------------------------------------
	// ------------------------ Process Namespace ------------------------
	//   This is necessary in order to source all later parameters  
	std::string name;
	bool temp_bool;
	if( !nh.getParam(yaml_file_name + "/registration_name", name) )
	{
		ROS_ERROR_STREAM("[RegCreation] Failed to get registration process name from yaml file! Exiting.");
		return false;
	}

	// --------------------------------------------------------------------
	// ------------ High Level Registration Type Specification ------------
	//   Finds string parameters for Point, Feature, Interest Point, Correspondence, and Transform Types
	//   Maps those strings to int codes for registration types (see  /srv/registration_service.srv  in this package)
	basicTypeSpecification(srv, yaml_file_name, name, nh);


	if( !nh.param<bool>(yaml_file_name + "/should_publish", temp_bool, true) )
		ROS_WARN_STREAM("[RegCreation] Failed to get should_publish - defaulting to true.");
	srv->request.should_publish = temp_bool;
	if( !nh.param<float>(yaml_file_name + "/" + name + "/epsilon", srv->request.epsilon, .001) )
		ROS_WARN_STREAM("[RegCreation] Failed to get epsilon - defaulting to 0.001");
	if( !nh.param<int>(yaml_file_name + "/" + name +  "/max_iterations", srv->request.max_iterations, 30) )
		ROS_WARN_STREAM("[RegCreation] Failed to get max_iterations - defaulting to 30.");
	// --------------------------------- Looping ----------------------------------
	if( !nh.param<bool>(yaml_file_name + "/loop_until_threshold", temp_bool, false))
		ROS_WARN_STREAM("[RegCreation] Failed to get loop_until_threshold - defaulting to false.");
	srv->request.repeatedly_register = temp_bool;
	if( !nh.param<float>(yaml_file_name + "/translation_threshold", srv->request.translation_threshold, 0.002))
		ROS_WARN_STREAM("[RegCreation] Failed to get translation_threshold - defaulting to 0.002.");
	if( !nh.param<float>(yaml_file_name + "/rotation_threshold", srv->request.rotation_threshold, 0.05))
		ROS_WARN_STREAM("[RegCreation] Failed to get rotation_threshold - defaulting to 0.05.");

	// ------------------------ Stuff for Preprocessing ------------------------
	pointcloud_processing_server::pointcloud_process preprocess;
	PointcloudTaskCreation::processFromYAML(&preprocess, "preprocess", "registration_example");
	for(int i=0; i<preprocess.request.tasks.size(); i++)
		srv->request.preprocessing_tasks.push_back(preprocess.request.tasks[i]);

	// ------------------------ Stuff for Postprocessing ------------------------
	pointcloud_processing_server::pointcloud_process postprocess;
	PointcloudTaskCreation::processFromYAML(&postprocess, "postprocess", "registration_example");
	for(int i=0; i<postprocess.request.tasks.size(); i++)
		srv->request.postprocessing_tasks.push_back(postprocess.request.tasks[i]);

	float temp_float;
	std::vector<float> temp_float_vector;
	switch(srv->request.registration_type)
	{
		case pointcloud_registration_server::registration_service::Request::TRANSFORM_METHOD_ICP:
			if( !nh.param<float>(yaml_file_name + "/" + name + "/ksearch", temp_float, 30) )
				ROS_WARN_STREAM("[RegCreation] Failed to get ksearch - defaulting to true.");
			srv->request.parameters.push_back(temp_float);
			if( !nh.param<float>(yaml_file_name + "/" + name + "/max_dist", temp_float, 0.1) )
				ROS_WARN_STREAM("[RegCreation] Failed to get max_dist - defaulting to true.");
			srv->request.parameters.push_back(temp_float);
			if( !nh.getParam(yaml_file_name + "/" + name + "/alpha", temp_float_vector) )
				ROS_WARN_STREAM("[RegCreation] Failed to get alpha - defaulting to true.");
			for(int i=0; i<4; i++)
				srv->request.parameters.push_back(temp_float_vector[i]);
			break;
		case pointcloud_registration_server::registration_service::Request::TRANSFORM_METHOD_NDT:
			if( !nh.param<float>(yaml_file_name + "/" + name + "/step_size", temp_float, 0.1) )
				ROS_WARN_STREAM("[RegCreation] Failed to get step_size - defaulting to true.");
			srv->request.parameters.push_back(temp_float);
			if( !nh.param<float>(yaml_file_name + "/" + name + "/resolution", temp_float, 1.0) )
				ROS_WARN_STREAM("[RegCreation] Failed to get resolution - defaulting to true.");
			srv->request.parameters.push_back(temp_float);
			break;
	}

	return true;
}


// *** High Level Process Definition ***
//   Sets up the basic indices for Point, Feature, Interest Point, Correspondence, and Transform types (see registration_service.srv)
//   Right now, inputs are specified as strings, which leads to this awkward if/else structure
//   It would be possible to directly specify these as integer codes, but they would have to be hard-coded in the YAML file
//   I think this is dangerous, as it could break functionality if the code mapping changes later on... so I prefer to do it this way
bool RegCreation::basicTypeSpecification(pointcloud_registration_server::registration_service *srv, std::string yaml_file_name, std::string name, ros::NodeHandle nh)
{
	// ------ Point Type ------
	std::string temp_str;
	if( !nh.param<std::string>(yaml_file_name + "/" + name + "/point_type", temp_str, "XYZ") )
		ROS_WARN_STREAM("[RegCreation] Failed to get Input Point Type - defaulting to XYZ");
	if( temp_str.compare("XYZ") )
		srv->request.point_type = pointcloud_registration_server::registration_service::Request::POINT_TYPE_XYZ;
	else if( temp_str.compare("XYZI") )
		srv->request.point_type = pointcloud_registration_server::registration_service::Request::POINT_TYPE_XYZI;
	else if( temp_str.compare("XYZN") )
		srv->request.point_type = pointcloud_registration_server::registration_service::Request::POINT_TYPE_XYZN;
	else if( temp_str.compare("XYZRGB") )
		srv->request.point_type = pointcloud_registration_server::registration_service::Request::POINT_TYPE_XYZRGB;
	else if( temp_str.compare("XYZRGBI") )
		srv->request.point_type = pointcloud_registration_server::registration_service::Request::POINT_TYPE_XYZRGBI;
	else if( temp_str.compare("XYZRGBN") )
		srv->request.point_type = pointcloud_registration_server::registration_service::Request::POINT_TYPE_XYZRGBN;
	else 
	{
		ROS_ERROR_STREAM("[RegCreation] Point Type input is not a valid option: " << temp_str << ". Exiting RegCreation as a failure.");
		return false;
	}

	// ------ Feature Type ------
	if( !nh.param<std::string>(yaml_file_name + "/" + name + "/feature_type", temp_str, "XYZ") )
		ROS_WARN_STREAM("[RegCreation] Failed to get Feature Type - defaulting to XYZ");
	if( temp_str.compare("XYZ") )
		srv->request.feature_type = pointcloud_registration_server::registration_service::Request::FEATURE_TYPE_XYZ;
	else if( temp_str.compare("XYZN") )
		srv->request.feature_type = pointcloud_registration_server::registration_service::Request::FEATURE_TYPE_XYZN;
	else 
	{
		ROS_ERROR_STREAM("[RegCreation] Feature Type input is not a valid option: " << temp_str << ". Exiting RegCreation as a failure.");
		return false;
	}

	// ------ Interest Point Type ------
	if( !nh.param<std::string>(yaml_file_name + "/" + name + "/interest_point_type", temp_str, "NONE") )
		ROS_WARN_STREAM("[RegCreation] Failed to get Feature Type - defaulting to NONE");
	if( temp_str.compare("NONE") )
		srv->request.interest_point_type = pointcloud_registration_server::registration_service::Request::INTEREST_TYPE_NONE;
	else if( temp_str.compare("SIFT") )
		srv->request.interest_point_type = pointcloud_registration_server::registration_service::Request::INTEREST_TYPE_SIFT;
	else 
	{
		ROS_ERROR_STREAM("[RegCreation] Interest Point Type input is not a valid option: " << temp_str << ". Exiting RegCreation as a failure.");
		return false;
	}

	// ------ Correspondence Search Method ------
	if( !nh.param<std::string>(yaml_file_name + "/" + name + "/correspondence_type", temp_str, "NONE") )
		ROS_WARN_STREAM("[RegCreation] Failed to get Feature Type - defaulting to NONE");
	if( temp_str.compare("XYZ") )
		srv->request.correspondence_search_type = pointcloud_registration_server::registration_service::Request::CORRESP_TYPE_NONE;
	else 
	{
		ROS_ERROR_STREAM("[RegCreation] Correspondence Search Method input is not a valid option: " << temp_str << ". Exiting RegCreation as a failure.");
		return false;
	}

	// ------ Transform Search Method ------
	if( !nh.param<std::string>(yaml_file_name + "/" + name + "/feature_type", temp_str, "ICP") )
		ROS_WARN_STREAM("[RegCreation] Failed to get Feature Type - defaulting to ICP");
	if( temp_str.compare("ICP") )
		srv->request.transformation_search_type = pointcloud_registration_server::registration_service::Request::TRANSFORM_METHOD_ICP;
	if( temp_str.compare("ICPNL") )
		srv->request.transformation_search_type = pointcloud_registration_server::registration_service::Request::TRANSFORM_METHOD_ICPNL;
	if( temp_str.compare("NDT") )
		srv->request.transformation_search_type = pointcloud_registration_server::registration_service::Request::TRANSFORM_METHOD_ICP;
	else 
	{
		ROS_ERROR_STREAM("[RegCreation] Transform Search Method input is not a valid option: " << temp_str << ". Exiting RegCreation as a failure.");
		return false;
	}
}



// *** Transform-specific Parameter Loading ***
//   Loads parameters for Transformation Methods (ie ICP, ICPNL, NDT...)
//   These parameters are different for different kinds of search methods, although some are common across methods.
//   The process-specific parameters are saved in a vector of floats within the service. 
bool RegCreation::transformSpecificParameters(pointcloud_registration_server::registration_service *srv, std::string yaml_file_name, std::string name, ros::NodeHandle nh)
{


}
