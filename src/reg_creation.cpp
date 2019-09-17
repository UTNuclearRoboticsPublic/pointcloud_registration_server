
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
	bool basicTypeSpecification(pointcloud_registration_server::registration_service *srv, std::string yaml_file_name, ros::NodeHandle nh);

	// *** Interest Point-specific Parameter Loading ***
	//   Loads parameters for Feature Estimation Methods (ie Normals, SIFT, etc...)
	//   These parameters are different for different kinds of features, although some are common across methods.
	//   The process-specific parameters are saved in a vector of floats within the service. 
	bool interestPointSpecificParameters(pointcloud_registration_server::registration_service *srv, std::string yaml_file_name, ros::NodeHandle nh);

	// *** Feature-specific Parameter Loading ***
	//   Loads parameters for Feature Estimation Methods (ie Normals, SIFT, etc...)
	//   These parameters are different for different kinds of features, although some are common across methods.
	//   The process-specific parameters are saved in a vector of floats within the service. 
	bool featureSpecificParameters(pointcloud_registration_server::registration_service *srv, std::string yaml_file_name, ros::NodeHandle nh);

	// *** Correspondence-specific Parameter Loading ***
	//   Loads parameters for Feature Estimation Methods (ie Normals, SIFT, etc...)
	//   These parameters are different for different kinds of features, although some are common across methods.
	//   The process-specific parameters are saved in a vector of floats within the service. 
	bool correspondenceSpecificParameters(pointcloud_registration_server::registration_service *srv, std::string yaml_file_name, ros::NodeHandle nh);

	// *** Transform-specific Parameter Loading ***
	//   Loads parameters for Transformation Methods (ie ICP, ICPNL, NDT...)
	//   These parameters are different for different kinds of search methods, although some are common across methods.
	//   The process-specific parameters are saved in a vector of floats within the service. 
	bool transformSpecificParameters(pointcloud_registration_server::registration_service *srv, std::string yaml_file_name, ros::NodeHandle nh);
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
	bool temp_bool;

	// --------------------------------------------------------------------
	// ------------ High Level Registration Type Specification ------------
	//   Finds string parameters for Point, Feature, Interest Point, Correspondence, and Transform Types
	//   Maps those strings to int codes for registration types (see  /srv/registration_service.srv  in this package)
	basicTypeSpecification(srv, yaml_file_name, nh);

	if( !nh.param<bool>(yaml_file_name + "/transform_raw_clouds", temp_bool, false) )
		ROS_WARN_STREAM("[RegCreation] Failed to get decision on whether to transform raw clouds - defaulting to FALSE");
	srv->request.transform_raw_clouds = temp_bool;
	if( !nh.param<bool>(yaml_file_name + "/transform_feature_clouds", temp_bool, false) )
		ROS_WARN_STREAM("[RegCreation] Failed to get decision on whether to transform feature clouds - defaulting to FALSE");
	srv->request.transform_raw_clouds = temp_bool;
	if( !nh.param<bool>(yaml_file_name + "/transform_postprocessed_clouds", temp_bool, false) )
		ROS_WARN_STREAM("[RegCreation] Failed to get decision on whether to transform postprocessed clouds - defaulting to FALSE");
	srv->request.transform_raw_clouds = temp_bool;

	// ------------------------ Stuff for Preprocessing ------------------------
	pointcloud_processing_server::pointcloud_process preprocess;
	PointcloudTaskCreation::processFromYAML(&preprocess, "preprocess", yaml_file_name);
	for(int i=0; i<preprocess.request.tasks.size(); i++)
		srv->request.preprocessing_tasks.push_back(preprocess.request.tasks[i]);

	// ------------------------ Stuff for Postprocessing ------------------------
	pointcloud_processing_server::pointcloud_process postprocess;
	PointcloudTaskCreation::processFromYAML(&postprocess, "postprocess", yaml_file_name);
	for(int i=0; i<postprocess.request.tasks.size(); i++)
		srv->request.postprocessing_tasks.push_back(postprocess.request.tasks[i]);

	// ----------------------- Interest Point-specific Parameters -----------------------
	if( !interestPointSpecificParameters(srv, yaml_file_name, nh) ) return false;
	// ----------------------- Feature-specific Parameters -----------------------
	if( !featureSpecificParameters(srv, yaml_file_name, nh) ) return false;
	// ----------------------- Correspondence-specific Parameters -----------------------
	if( !correspondenceSpecificParameters(srv, yaml_file_name, nh) ) return false;
	// ----------------------- Transform-specific Parameters -----------------------
	if( !transformSpecificParameters(srv, yaml_file_name, nh) ) return false;

	return true;
}


// *** High Level Process Definition ***
//   Sets up the basic indices for Point, Feature, Interest Point, Correspondence, and Transform types (see registration_service.srv)
//   Right now, inputs are specified as strings, which leads to this awkward if/else structure
//   It would be possible to directly specify these as integer codes, but they would have to be hard-coded in the YAML file
//   I think this is dangerous, as it could break functionality if the code mapping changes later on... so I prefer to do it this way
bool RegCreation::basicTypeSpecification(pointcloud_registration_server::registration_service *srv, std::string yaml_file_name, ros::NodeHandle nh)
{
	// ------ Point Type ------
	std::string temp_str;
	if( !nh.param<std::string>(yaml_file_name + "/point_type", temp_str, "XYZ") )
		ROS_WARN_STREAM("[RegCreation] Failed to get Input Point Type - defaulting to XYZ");
	if( temp_str.compare("XYZ") == 0 )
		srv->request.point_type = pointcloud_registration_server::registration_service::Request::POINT_TYPE_XYZ;
	else if( temp_str.compare("XYZI") == 0 )
		srv->request.point_type = pointcloud_registration_server::registration_service::Request::POINT_TYPE_XYZI;
	else if( temp_str.compare("XYZN") == 0 )
		srv->request.point_type = pointcloud_registration_server::registration_service::Request::POINT_TYPE_XYZN;
	else if( temp_str.compare("XYZNI") == 0 )
		srv->request.point_type = pointcloud_registration_server::registration_service::Request::POINT_TYPE_XYZNI;
	else if( temp_str.compare("XYZRGB") == 0 )
		srv->request.point_type = pointcloud_registration_server::registration_service::Request::POINT_TYPE_XYZRGB;
	else if( temp_str.compare("XYZRGBI") == 0 )
		srv->request.point_type = pointcloud_registration_server::registration_service::Request::POINT_TYPE_XYZRGBI;
	else if( temp_str.compare("XYZRGBN") == 0 )
		srv->request.point_type = pointcloud_registration_server::registration_service::Request::POINT_TYPE_XYZRGBN;
	else if( temp_str.compare("XYZRGBNI") == 0 )
		srv->request.point_type = pointcloud_registration_server::registration_service::Request::POINT_TYPE_XYZRGBNI;
	else if( temp_str.compare("WALLDAMAGE") == 0 )
		srv->request.point_type = pointcloud_registration_server::registration_service::Request::POINT_TYPE_WALL_DAMAGE;
	else 
	{
		ROS_ERROR_STREAM("[RegCreation] Point Type input is not a valid option: " << temp_str << ". Exiting RegCreation as a failure.");
		return false;
	}

	// ------ Feature Type ------
	if( !nh.param<std::string>(yaml_file_name + "/feature_parameters/feature_type", temp_str, "XYZ") )
		ROS_WARN_STREAM("[RegCreation] Failed to get Feature Type - defaulting to XYZ");
	if( temp_str.compare("XYZ") == 0 )
		srv->request.feature_type = pointcloud_registration_server::registration_service::Request::FEATURE_TYPE_XYZ;
	else if( temp_str.compare("XYZI") == 0 )
		srv->request.feature_type = pointcloud_registration_server::registration_service::Request::FEATURE_TYPE_XYZI;
	else if( temp_str.compare("XYZN") == 0 )
		srv->request.feature_type = pointcloud_registration_server::registration_service::Request::FEATURE_TYPE_XYZN;
	else if( temp_str.compare("XYZNI") == 0 )
		srv->request.feature_type = pointcloud_registration_server::registration_service::Request::FEATURE_TYPE_XYZNI;
	else if( temp_str.compare("XYZRGB") == 0 )
		srv->request.feature_type = pointcloud_registration_server::registration_service::Request::FEATURE_TYPE_XYZRGB;
	else if( temp_str.compare("XYZRGBI") == 0 )
		srv->request.feature_type = pointcloud_registration_server::registration_service::Request::FEATURE_TYPE_XYZRGBI;
	else if( temp_str.compare("XYZRGBN") == 0 )
		srv->request.feature_type = pointcloud_registration_server::registration_service::Request::FEATURE_TYPE_XYZRGBN;
	else if( temp_str.compare("XYZRGBNI") == 0 )
		srv->request.feature_type = pointcloud_registration_server::registration_service::Request::FEATURE_TYPE_XYZRGBNI;
	else if( temp_str.compare("SIFT") == 0 )
		srv->request.feature_type = pointcloud_registration_server::registration_service::Request::FEATURE_TYPE_SIFT;
	else if( temp_str.compare("ITWF") == 0)
		srv->request.feature_type = pointcloud_registration_server::registration_service::Request::FEATURE_TYPE_ITWF;
	else if( temp_str.compare("DIFF_NORM") == 0)
		srv->request.feature_type = pointcloud_registration_server::registration_service::Request::FEATURE_TYPE_DIFF_NORM;
	else 
	{
		ROS_ERROR_STREAM("[RegCreation] Feature Type input is not a valid option: " << temp_str << ". Exiting RegCreation as a failure.");
		return false;
	}

	// ------ Interest Point Type ------
	if( !nh.param<std::string>(yaml_file_name + "/interest_point_parameters/interest_point_type", temp_str, "NONE") )
		ROS_WARN_STREAM("[RegCreation] Failed to get Feature Type - defaulting to NONE");
	if( temp_str.compare("NONE") == 0 )
		srv->request.interest_point_type = pointcloud_registration_server::registration_service::Request::INTEREST_TYPE_NONE;
	else if( temp_str.compare("SIFT") == 0 )
		srv->request.interest_point_type = pointcloud_registration_server::registration_service::Request::INTEREST_TYPE_SIFT;
	else if( temp_str.compare("WALLMAXIMA") == 0 )
		srv->request.interest_point_type = pointcloud_registration_server::registration_service::Request::INTEREST_TYPE_WALL_MAXIMA;
	else 
	{
		ROS_ERROR_STREAM("[RegCreation] Interest Point Type input is not a valid option: " << temp_str << ". Exiting RegCreation as a failure.");
		return false;
	}

	// ------ Correspondence Search Method ------
	if( !nh.param<std::string>(yaml_file_name + "/correspondence_parameters/correspondence_type", temp_str, "NONE") )
		ROS_WARN_STREAM("[RegCreation] Failed to get Feature Type - defaulting to NONE");
	if( temp_str.compare("NONE") == 0 )
		srv->request.correspondence_search_type = pointcloud_registration_server::registration_service::Request::CORRESP_TYPE_NONE;
	if( temp_str.compare("Normal") == 0 )
		srv->request.correspondence_search_type = pointcloud_registration_server::registration_service::Request::CORRESP_TYPE_NORMAL;
	if( temp_str.compare("DIST") == 0 )
		srv->request.correspondence_search_type = pointcloud_registration_server::registration_service::Request::CORRESP_TYPE_DIST;
	if( temp_str.compare("DIST_HORZ") == 0 )
		srv->request.correspondence_search_type = pointcloud_registration_server::registration_service::Request::CORRESP_TYPE_DIST_HORZ;
	else 
	{
		ROS_ERROR_STREAM("[RegCreation] Correspondence Search Method input is not a valid option: " << temp_str << ". Exiting RegCreation as a failure.");
		return false;
	}

	// ------ Transform Search Method ------
	if( !nh.param<std::string>(yaml_file_name + "/transform_parameters/transformation_type", temp_str, "ICP") )
		ROS_WARN_STREAM("[RegCreation] Failed to get Feature Type - defaulting to ICP");
	if( temp_str.compare("ICP") == 0 )
		srv->request.transformation_search_type = pointcloud_registration_server::registration_service::Request::TRANSFORM_METHOD_ICP;
	else if( temp_str.compare("ICPNL") == 0 )
		srv->request.transformation_search_type = pointcloud_registration_server::registration_service::Request::TRANSFORM_METHOD_ICPNL;
	else if( temp_str.compare("NDT") == 0 )
		srv->request.transformation_search_type = pointcloud_registration_server::registration_service::Request::TRANSFORM_METHOD_NDT;
	else if( temp_str.compare("RANSAC") == 0 )
		srv->request.transformation_search_type = pointcloud_registration_server::registration_service::Request::TRANSFORM_METHOD_RANSAC;
	else 
	{
		ROS_ERROR_STREAM("[RegCreation] Transform Search Method input is not a valid option: " << temp_str << ". Exiting RegCreation as a failure.");
		return false;
	}
}


// *** Interest Point-specific Parameter Loading ***
//   Loads parameters for Feature Estimation Methods (ie Normals, SIFT, etc...)
//   These parameters are different for different kinds of features, although some are common across methods.
//   The process-specific parameters are saved in a vector of floats within the service. 
bool RegCreation::interestPointSpecificParameters(pointcloud_registration_server::registration_service *srv, std::string yaml_file_name, ros::NodeHandle nh)
{
	switch(srv->request.interest_point_type)
	{
		case pointcloud_registration_server::registration_service::Request::INTEREST_TYPE_NONE:
		{
			break;
		}
		case pointcloud_registration_server::registration_service::Request::INTEREST_TYPE_SIFT:
		{
			break;
		}
		case pointcloud_registration_server::registration_service::Request::INTEREST_TYPE_WALL_MAXIMA:
		{
			break;
		}
	}
}


// *** Feature-specific Parameter Loading ***
//   Loads parameters for Feature Estimation Methods (ie Normals, SIFT, etc...)
//   These parameters are different for different kinds of features, although some are common across methods.
//   The process-specific parameters are saved in a vector of floats within the service. 
bool RegCreation::featureSpecificParameters(pointcloud_registration_server::registration_service *srv, std::string yaml_file_name, ros::NodeHandle nh)
{
	// Clear the parameter list
	srv->request.feature_parameters.clear();

	float temp_float;

	switch(srv->request.feature_type)
	{
		case pointcloud_registration_server::registration_service::Request::FEATURE_TYPE_XYZ:
		{
			break;
		}
		case pointcloud_registration_server::registration_service::Request::FEATURE_TYPE_XYZN:
		{
			if( !nh.param<float>(yaml_file_name + "/feature_parameters/ksearch", temp_float, 30) )
				ROS_WARN_STREAM("[RegCreation] Failed to get ksearch - defaulting to 30.");
			srv->request.transform_parameters.push_back(temp_float);
			break;
		}
		case pointcloud_registration_server::registration_service::Request::FEATURE_TYPE_XYZNI:
		{
			if( !nh.param<float>(yaml_file_name + "/feature_parameters/ksearch", temp_float, 30) )
				ROS_WARN_STREAM("[RegCreation] Failed to get ksearch - defaulting to 30.");
			srv->request.transform_parameters.push_back(temp_float);
			break;
		}
		case pointcloud_registration_server::registration_service::Request::FEATURE_TYPE_XYZRGBN:
		{
			if( !nh.param<float>(yaml_file_name + "/feature_parameters/ksearch", temp_float, 30) )
				ROS_WARN_STREAM("[RegCreation] Failed to get ksearch - defaulting to 30.");
			srv->request.transform_parameters.push_back(temp_float);
			break;
		}
		case pointcloud_registration_server::registration_service::Request::FEATURE_TYPE_XYZRGBNI:
		{
			if( !nh.param<float>(yaml_file_name + "/feature_parameters/ksearch", temp_float, 30) )
				ROS_WARN_STREAM("[RegCreation] Failed to get ksearch - defaulting to 30.");
			srv->request.transform_parameters.push_back(temp_float);
			break;
		}
		case pointcloud_registration_server::registration_service::Request::FEATURE_TYPE_ITWF:
		{
			if( !nh.param<float>(yaml_file_name + "/feature_parameters/ksearch", temp_float, 30) )
				ROS_WARN_STREAM("[RegCreation] Failed to get ksearch - defaulting to 30.");
			srv->request.transform_parameters.push_back(temp_float);
			std::vector<int> dimensions;
			if( !nh.getParam(yaml_file_name + "/feature_parameters/dimension_use", dimensions) )
			{
				ROS_WARN_STREAM("[RegCreation] Failed to get dimension_use specificaiton for ITWF - defaulting to use all dimensions.");
				for(int i=0; i<7; i++)
					dimensions.push_back(1);
			}
			else if (dimensions.size() != 7)
			{
				ROS_WARN_STREAM("[RegCreation] Found dimension_use specificaiton for ITWF, but it didn't have 7 indices - defaulting to use all dimensions.");
				for(int i=0; i<7; i++)
					dimensions.push_back(1);
			}
			for(int i=0; i<dimensions.size(); i++)
				srv->request.transform_parameters.push_back(dimensions[i]);
			std::vector<int> wall_coeffs;
			if( !nh.getParam(yaml_file_name + "/feature_parameters/wall_coeffs", wall_coeffs) )
			{
				ROS_WARN_STREAM("[RegCreation] Failed to get wall_coeffs specificaiton for ITWF - exiting as failure.");
				return -1;
			}
			else if (wall_coeffs.size() != 7)
			{
				ROS_WARN_STREAM("[RegCreation] Found wall_coeffs specificaiton for ITWF, but it didn't have 7 indices - exiting as failure.");
				return -1;
			}
			for(int i=0; i<wall_coeffs.size(); i++)
				srv->request.transform_parameters.push_back(wall_coeffs[i]);
			break;
		}
		case pointcloud_registration_server::registration_service::Request::FEATURE_TYPE_FPFH:
		{
			if( !nh.param<float>(yaml_file_name + "/feature_parameters/search_radius", temp_float, 0.1) )
				ROS_WARN_STREAM("[RegCreation] Failed to get search radius - defaulting to 30.");
			srv->request.transform_parameters.push_back(temp_float);
			break;
		}
		default:
		{
			ROS_ERROR_STREAM("[RegCreation] Illegal feature_type specified with value " << srv->request.feature_type << ". Exiting as failure...");
			return false;
		}
	}

	return true;

}



// *** Correspondence-specific Parameter Loading ***
//   Loads parameters for Feature Estimation Methods (ie Normals, SIFT, etc...)
//   These parameters are different for different kinds of features, although some are common across methods.
//   The process-specific parameters are saved in a vector of floats within the service. 
bool RegCreation::correspondenceSpecificParameters(pointcloud_registration_server::registration_service *srv, std::string yaml_file_name, ros::NodeHandle nh)
{
	// Clear the parameter list
	srv->request.correspondence_parameters.clear();

	float temp_float;

	switch(srv->request.correspondence_search_type)
	{
		case pointcloud_registration_server::registration_service::Request::CORRESP_TYPE_NONE:
		{
			break;
		}
		case pointcloud_registration_server::registration_service::Request::CORRESP_TYPE_NORMAL:
		{
			if( !nh.param<float>(yaml_file_name + "/feature_parameters/ransac_rejection", temp_float, 0.15) )
				ROS_WARN_STREAM("[RegCreation] Failed to get decision regarding RANSAC correspondence rejection - defaulting to not apply rejection.");
			srv->request.transform_parameters.push_back(temp_float);
			if( !nh.param<float>(yaml_file_name + "/feature_parameters/inlier_threshold", temp_float, 0.02) )
				ROS_WARN_STREAM("[RegCreation] Failed to get inlier threshold distance for RANSAC correspondence rejection - defaulting to 0.02.");
			srv->request.transform_parameters.push_back(temp_float);
			if( !nh.param<float>(yaml_file_name + "/feature_parameters/ransac_iterations", temp_float, 10000) )
				ROS_WARN_STREAM("[RegCreation] Failed to get maximum number of iterations for RANSAC correspondence rejection - defaulting to 10000.");
			srv->request.transform_parameters.push_back(temp_float);
			break;
		}
		case pointcloud_registration_server::registration_service::Request::CORRESP_TYPE_DIST:
		{
			if( !nh.param<float>(yaml_file_name + "/feature_parameters/ransac_rejection", temp_float, 0.15) )
				ROS_WARN_STREAM("[RegCreation] Failed to get decision regarding RANSAC correspondence rejection - defaulting to not apply rejection.");
			srv->request.transform_parameters.push_back(temp_float);
			if( !nh.param<float>(yaml_file_name + "/feature_parameters/inlier_threshold", temp_float, 0.02) )
				ROS_WARN_STREAM("[RegCreation] Failed to get inlier threshold distance for RANSAC correspondence rejection - defaulting to 0.02.");
			srv->request.transform_parameters.push_back(temp_float);
			if( !nh.param<float>(yaml_file_name + "/feature_parameters/ransac_iterations", temp_float, 10000) )
				ROS_WARN_STREAM("[RegCreation] Failed to get maximum number of iterations for RANSAC correspondence rejection - defaulting to 10000.");
			srv->request.transform_parameters.push_back(temp_float);
			if( !nh.param<float>(yaml_file_name + "/feature_parameters/band_height", temp_float, 0.15) )
				ROS_WARN_STREAM("[RegCreation] Failed to get ksearch - defaulting to 0.15");
			srv->request.transform_parameters.push_back(temp_float);
			break;
		}
		case pointcloud_registration_server::registration_service::Request::CORRESP_TYPE_DIST_HORZ:
		{			
			if( !nh.param<float>(yaml_file_name + "/feature_parameters/ransac_rejection", temp_float, 0.15) )
				ROS_WARN_STREAM("[RegCreation] Failed to get decision regarding RANSAC correspondence rejection - defaulting to not apply rejection.");
			srv->request.transform_parameters.push_back(temp_float);
			if( !nh.param<float>(yaml_file_name + "/feature_parameters/inlier_threshold", temp_float, 0.02) )
				ROS_WARN_STREAM("[RegCreation] Failed to get inlier threshold distance for RANSAC correspondence rejection - defaulting to 0.02.");
			srv->request.transform_parameters.push_back(temp_float);
			if( !nh.param<float>(yaml_file_name + "/feature_parameters/ransac_iterations", temp_float, 10000) )
				ROS_WARN_STREAM("[RegCreation] Failed to get maximum number of iterations for RANSAC correspondence rejection - defaulting to 10000.");
			srv->request.transform_parameters.push_back(temp_float);
			if( !nh.param<float>(yaml_file_name + "/feature_parameters/band_height", temp_float, 0.15) )
				ROS_WARN_STREAM("[RegCreation] Failed to get ksearch - defaulting to 1");
			srv->request.transform_parameters.push_back(temp_float);
			if( !nh.param<float>(yaml_file_name + "/feature_parameters/band_length", temp_float, 1.0) )
				ROS_WARN_STREAM("[RegCreation] Failed to get ksearch - defaulting to 30.");
			srv->request.transform_parameters.push_back(temp_float);
			break;
		}
	}
}




// *** Transform-specific Parameter Loading ***
//   Loads parameters for Transformation Methods (ie ICP, ICPNL, NDT...)
//   These parameters are different for different kinds of search methods, although some are common across methods.
//   The process-specific parameters are saved in a vector of floats within the service. 
bool RegCreation::transformSpecificParameters(pointcloud_registration_server::registration_service *srv, std::string yaml_file_name, ros::NodeHandle nh)
{
	// Clear the parameter list
	srv->request.transform_parameters.clear();

	// Temp objects to assist with transfer of data from parameter server
	float temp_float;
	std::vector<float> temp_float_vector;

	switch(srv->request.transformation_search_type)
	{
		case pointcloud_registration_server::registration_service::Request::TRANSFORM_METHOD_ICP:
		{
			if( !nh.param<float>(yaml_file_name + "/transformation_parameters/epsilon", temp_float, .01) )
				ROS_WARN_STREAM("[RegCreation] Failed to get epsilon - defaulting to " << temp_float << ".");
			srv->request.transform_parameters.push_back(temp_float);
			if( !nh.param<float>(yaml_file_name + "/transform_parameters/max_iterations", temp_float, 30) )
				ROS_WARN_STREAM("[RegCreation] Failed to get max_iterations - defaulting to " << temp_float << ".");
			srv->request.transform_parameters.push_back(temp_float);
			if( !nh.param<float>(yaml_file_name + "/transform_parameters/max_dist", temp_float, 0.1) )
				ROS_WARN_STREAM("[RegCreation] Failed to get max_dist - defaulting to " << temp_float << ".");
			srv->request.transform_parameters.push_back(temp_float);
			if( !nh.param<float>(yaml_file_name + "/transform_parameters/euclidian_epsilon", temp_float, 0.1) )
				ROS_WARN_STREAM("[RegCreation] Failed to get max_dist - defaulting to " << temp_float << ".");
			srv->request.transform_parameters.push_back(temp_float);
			if( !nh.getParam(yaml_file_name + "/transform_parameters/alpha", temp_float_vector) )
			{
				ROS_ERROR_STREAM("[RegCreation] Failed to get alpha - exiting as failed.");
				return false;
			}
			for(int i=0; i<4; i++)
				srv->request.transform_parameters.push_back(temp_float_vector[i]);
			break;
		}
		case pointcloud_registration_server::registration_service::Request::TRANSFORM_METHOD_NDT:
		{
			if( !nh.param<float>(yaml_file_name + "/transform_parameters/epsilon", temp_float, .001) )
				ROS_WARN_STREAM("[RegCreation] Failed to get epsilon - defaulting to " << temp_float << ".");
			srv->request.transform_parameters.push_back(temp_float);
			if( !nh.param<float>(yaml_file_name + "/transform_parameters/max_iterations", temp_float, 30) )
				ROS_WARN_STREAM("[RegCreation] Failed to get max_iterations - defaulting to " << temp_float << ".");
			srv->request.transform_parameters.push_back(temp_float);
			if( !nh.param<float>(yaml_file_name + "/transform_parameters/step_size", temp_float, 0.1) )
				ROS_WARN_STREAM("[RegCreation] Failed to get step_size - defaulting to " << temp_float << ".");
			srv->request.transform_parameters.push_back(temp_float);
			if( !nh.param<float>(yaml_file_name + "/transform_parameters/resolution", temp_float, 1.0) )
				ROS_WARN_STREAM("[RegCreation] Failed to get resolution - defaulting to " << temp_float << ".");
			srv->request.transform_parameters.push_back(temp_float);
			break;
		}
		case pointcloud_registration_server::registration_service::Request::TRANSFORM_METHOD_SVD:
		{

			break;
		}
		case pointcloud_registration_server::registration_service::Request::TRANSFORM_METHOD_AVG:
		{

			break;
		}
	}

}
