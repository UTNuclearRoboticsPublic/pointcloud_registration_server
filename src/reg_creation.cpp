
#include "pointcloud_registration_server/reg_creation.h"

namespace RegCreation
{
	bool registrationFromYAML(pointcloud_registration_server::registration_service *srv, std::string yaml_file_name)
	{
//		if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
//    ros::console::notifyLoggerLevelsChanged();  

		ros::NodeHandle nh;
		ROS_DEBUG_STREAM("[RegCreation] Beginning to attempt to initialize parameters from server for registration " + yaml_file_name + ".");

		// ------------ Basic Stuff Required for All Registration Processes ------------
		std::string name;
		bool temp_bool;
		if( !nh.getParam(yaml_file_name + "/registration_name", name) )
		{
			ROS_ERROR_STREAM("[RegCreation] Failed to get registration process name from yaml file! Exiting.");
			return false;
		}
		if( !nh.getParam(yaml_file_name + "/" + name + "/registration_type", srv->request.registration_type) )
		{
			ROS_ERROR_STREAM("[RegCreation] Failed to get registration type." );
			return false;
		} 
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
};