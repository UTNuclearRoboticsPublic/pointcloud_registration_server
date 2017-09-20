
#include "pointcloud_registration_server/pointcloud_registration.h"

PCRegistration::PCRegistration()
{
	ros::ServiceServer server = nh_.advertiseService("register_pointclouds", &PCRegistration::registerPointclouds, this);

	ros::spin();
}
	
bool PCRegistration::registerPointclouds(pointcloud_registration_server::registration_service::Request& req, pointcloud_registration_server::registration_service::Response& res)
{	
	// --------------------- Starting service ---------------------
	//   Initialize clouds, set the start time
	ros::Time callback_received_time = ros::Time::now();
	ROS_DEBUG_STREAM("[PCRegistration] Received service callback.");
	PCP source_cloud = PCP(new PC);
	PCP target_cloud = PCP(new PC);
	PCP output_cloud = PCP(new PC);
	pcl::fromROSMsg(req.cloud_list[0], *output_cloud);

	// --------------------- Preprocessing ---------------------
	std::vector<PCP> preprocessed_clouds;
	for(int i=0; i<req.cloud_list.size(); i++)
	{
		preprocessing(req, res, i);
		pcl::fromROSMsg(res.preprocessing_results[i].task_pointcloud, *preprocessed_clouds[i]);
	}

	// --------------------- Actually Register --------------------- 
	// Loop over all clouds provided in service input 
	for (int i=1; i<req.cloud_list.size(); i++)
	{
		// Start time
		ros::Time registration_start_time = ros::Time::now();
		
		// Output transform for this registration
		Eigen::Matrix4f final_transform;

		switch(req.registration_type)
		{
			case 1:
			{
				ROS_DEBUG_STREAM("[PCRegistration] Running " << i <<"th ICP registration process.");
				float alpha[4] = {req.parameters[2], req.parameters[3], req.parameters[4], req.parameters[5]};
				registerICP(source_cloud, target_cloud, output_cloud, final_transform, req.epsilon, req.max_iterations, req.parameters[0], req.parameters[1], alpha);
				break;
			}
			case 2:
			{
				ROS_DEBUG_STREAM("[PCRegistration] Running " << i <<"th NDT registration process.");
				registerNDT(source_cloud, target_cloud, output_cloud, final_transform, req.epsilon, req.max_iterations, req.parameters[0], req.parameters[1]);
				break;
			}
			default:
			{
				ROS_ERROR_STREAM("[PCRegistration] Registration type selected for " << i << "th registration, " << req.registration_type << ", is not (yet?) supported.");
				break;
			}
		}
		ROS_DEBUG_STREAM("[PCRegistration] Successfully registered a pair of clouds. Transformation is: ");

		// --------------------- Transformation ---------------------
		//   -------- Reformatting the output transform (into a ROS message) --------
		tf::Transform tf_transform_final;
		tf::Vector3 tf_vector_final;
		tf_vector_final.setValue(final_transform(0,3),final_transform(1,3),final_transform(2,3));
		tf_transform_final.setOrigin(tf_vector_final);

		tf::Quaternion tf_quat_final;
		tf::Matrix3x3 tf3d;
	  	tf3d.setValue(	static_cast<double>(final_transform(0,0)), static_cast<double>(final_transform(0,1)), static_cast<double>(final_transform(0,2)), 
        				static_cast<double>(final_transform(1,0)), static_cast<double>(final_transform(1,1)), static_cast<double>(final_transform(1,2)), 
    					static_cast<double>(final_transform(2,0)), static_cast<double>(final_transform(2,1)), static_cast<double>(final_transform(2,2)));
		tf3d.getRotation(tf_quat_final);
		tf_transform_final.setRotation(tf_quat_final);

		ROS_DEBUG_STREAM("   Translation:  x: " << tf_transform_final.getOrigin().x() << "  y: " << tf_transform_final.getOrigin().y() << "  z: " << tf_transform_final.getOrigin().z());
		ROS_DEBUG_STREAM("   Rotation:     x: " << tf_transform_final.getRotation().getAxis().x() << "  y: " << tf_transform_final.getRotation().getAxis().y() << "  z: " << tf_transform_final.getRotation().getAxis().z() << "  w: " << tf_transform_final.getRotation().w());

		geometry_msgs::Pose output_pose;
		output_pose.position.x = tf_transform_final.getOrigin().x();
		output_pose.position.y = tf_transform_final.getOrigin().y();
		output_pose.position.z = tf_transform_final.getOrigin().z();
		output_pose.orientation.x = tf_transform_final.getRotation().getAxis().x();
		output_pose.orientation.x = tf_transform_final.getRotation().getAxis().y();
		output_pose.orientation.x = tf_transform_final.getRotation().getAxis().z();
		output_pose.orientation.x = tf_transform_final.getRotation().w();

		//   -------- Actually transform and concatenate clouds --------
		PCP transformed_source = PCP(new PC());
		pcl::transformPointCloud(*source_cloud, *transformed_source, final_transform);
		*output_cloud += *transformed_source;

		res.transforms.push_back(output_pose);

		ros::Duration registration_duration = ros::Time::now() - registration_start_time;
		res.registration_time.push_back(registration_duration.toSec());
	}
	ROS_DEBUG_STREAM("[PCRegistration] Finished all registration processing.");

	// --------------------- Postprocessing --------------------- 
	postprocessing(req, res, output_cloud);

	// ------------------ Outputs / Publishing ------------------ 
	pcl::toROSMsg(*output_cloud, res.output_cloud);
	if(req.should_publish)
	{
		ros::Publisher output_publisher = nh_.advertise<sensor_msgs::PointCloud2>("pc_registration/output", 1);
		output_publisher.publish(res.output_cloud);
	}

	ros::Duration total_time = callback_received_time - ros::Time::now();
	res.total_time = total_time.toSec();

	return true;
}

void PCRegistration::registerNDT(const PCP source_cloud, const PCP target_cloud, PCP output_cloud, Eigen::Matrix4f &final_transform, float epsilon, int max_iterations, float step_size, float resolution) //, Eigen::Something pose_estimate)
{
	// TODO
	//   Implement odometry estimate stuff using actual robot pose
	//   Expose more of the parameter options 
	ROS_DEBUG_STREAM("[PCRegistration] Starting NDT Process!  Epsilon: " << epsilon << "  Max_It: " << max_iterations << "  Step Size: " << step_size << "  Res: " << resolution);

	ros::Time time_start = ros::Time::now();

	// Initializing Normal Distributions Transform (NDT).
	pcl::NormalDistributionsTransform<PCLPoint, PCLPoint> ndt;

	// Setting scale dependent NDT parameters
	// Setting minimum transformation difference for termination condition.
	ndt.setTransformationEpsilon (epsilon);
	// Setting maximum step size for More-Thuente line search.
	ndt.setStepSize (step_size);
	//Setting Resolution of NDT grid structure (VoxelGridCovariance).
	ndt.setResolution (resolution);

	// Setting max number of registration iterations.
	ndt.setMaximumIterations (max_iterations);

	// Setting point cloud to be aligned.
	ndt.setInputSource (source_cloud);
	// Setting point cloud to be aligned to.
	ndt.setInputTarget (target_cloud);

	// Set initial alignment estimate found using robot odometry.
	Eigen::AngleAxisf init_rotation (0.0, Eigen::Vector3f::UnitZ ());
	Eigen::Translation3f init_translation (0.0, 0.0, 0);
	Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();

	// Calculating required rigid transform to align the input cloud to the target cloud.
	ndt.align (*output_cloud, init_guess);

	final_transform = ndt.getFinalTransformation();

	ros::Duration registration_duration = ros::Time::now() - time_start;
	ROS_DEBUG_STREAM("[PCRegistration] Finished NDT Registration! Entire registration process took " << registration_duration << " seconds.");
}

void PCRegistration::registerICP(const PCP source_cloud, const PCP target_cloud, PCP output_cloud, Eigen::Matrix4f &final_transform, float epsilon, int max_iterations, int ksearch, float max_dist, float alpha[4])
{	
	ROS_DEBUG_STREAM("[PCRegistration] Starting ICP Process!  Epsilon: " << epsilon << "  Max_It:" << max_iterations << "  K_Search: " << ksearch << "  Max_Dist: " << max_dist << "  Alpha: " << alpha[0] << alpha[1] << alpha[2] << alpha[3]);

	ros::Time time_start = ros::Time::now();

	ROS_DEBUG_STREAM("[PCRegistration] Received call to registration process.");
	PCNP points_with_normals_source (new PCN);
	PCNP points_with_normals_target (new PCN);
	ROS_DEBUG_STREAM("[PCRegistration] Created clouds from input.");

	std::vector<int> index_source, index_target;
	pcl::removeNaNFromPointCloud(*source_cloud, *source_cloud, index_source);
	pcl::removeNaNFromPointCloud(*target_cloud, *target_cloud, index_target);

	pcl::NormalEstimation<PCLPoint, PCLPointNormal> norm_est;
	pcl::search::KdTree<PCLPoint>::Ptr tree (new pcl::search::KdTree<PCLPoint> ());
	norm_est.setSearchMethod (tree);
	norm_est.setKSearch (ksearch);
	ROS_DEBUG_STREAM("[PCRegistration] Initialized norm_est object.");

	norm_est.setInputCloud (source_cloud);
	norm_est.compute (*points_with_normals_source);
	pcl::copyPointCloud (*source_cloud, *points_with_normals_source);

	norm_est.setInputCloud (target_cloud);
	norm_est.compute (*points_with_normals_target);
	pcl::copyPointCloud (*target_cloud, *points_with_normals_target);
	ROS_DEBUG_STREAM("[PCRegistration] Computed cloud normals.");

	// Instantiate our custom point representation (defined above) ...
	MyPointRepresentation point_representation;
	// ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
	point_representation.setRescaleValues (alpha);
	ROS_DEBUG_STREAM("[PCRegistration] Initialized point_representation.");

	// Align
	pcl::IterativeClosestPointNonLinear<PCLPointNormal, PCLPointNormal> reg;
	reg.setTransformationEpsilon (epsilon);
	// Set the maximum distance between two correspondences (src<->tgt) to 10cm
	// Note: adjust this based on the size of your datasets
	reg.setMaxCorrespondenceDistance (max_dist);  
	// Set the point representation
	reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

	std::vector<int> index_source_normal, index_target_normal;
	pcl::removeNaNFromPointCloud(*points_with_normals_source, *points_with_normals_source, index_source_normal);
	pcl::removeNaNFromPointCloud(*points_with_normals_target, *points_with_normals_target, index_target_normal);	

	pcl::removeNaNNormalsFromPointCloud(*points_with_normals_source, *points_with_normals_source, index_source_normal);
	pcl::removeNaNNormalsFromPointCloud(*points_with_normals_target, *points_with_normals_target, index_target_normal);	
	
	reg.setInputSource (points_with_normals_source);
	reg.setInputTarget (points_with_normals_target);

	Eigen::Matrix4f source_to_target = Eigen::Matrix4f::Identity (), prev, target_to_source;
	PCNP reg_result = PCNP(new PCN);// = points_with_normals_source;
	reg.setMaximumIterations (max_iterations);
	ROS_DEBUG_STREAM("[PCRegistration] Initialized reg object.");

	// Estimate
	reg.setInputSource (points_with_normals_source);
	ros::Time before = ros::Time::now();
	reg.align (*reg_result);
	ros::Time after = ros::Time::now();
	ros::Duration time_elapsed = after - before;
	ROS_DEBUG_STREAM("[PCRegistration] Performed registration - it took " << time_elapsed << " seconds.");
	
	//accumulate transformation between each Iteration
	final_transform = reg.getFinalTransformation () * source_to_target;
	ROS_DEBUG_STREAM("[PCRegistration] Found final transformation.");	

	ros::Duration registration_duration = ros::Time::now() - time_start;
	ROS_DEBUG_STREAM("[PCRegistration] Finished ICP Registration! Entire registration process took " << registration_duration << " seconds.");
}

bool PCRegistration::preprocessing(pointcloud_registration_server::registration_service::Request& req, pointcloud_registration_server::registration_service::Response& res, int cloud_index)
{
	ros::ServiceClient preprocessor = nh_.serviceClient<pointcloud_processing_server::pointcloud_process>("pointcloud_process");
	int service_call_attempts = 0;

	ros::Time time_start_preprocessing = ros::Time::now();
	// --------- Source Cloud Preprocessing ---------
	pointcloud_processing_server::pointcloud_process preprocess;
	for(int i=0; i<req.source_tasks.size(); i++)
	{
		preprocess.request.tasks.push_back(req.source_tasks[i]);
	}
	preprocess.request.pointcloud = req.cloud_list[cloud_index];
	preprocess.request.pointcloud.header.stamp = ros::Time::now();
	preprocess.request.min_cloud_size = 100;
	ROS_DEBUG_STREAM("[PCRegistration] Preprocessing a cloud. Size: " << req.cloud_list[cloud_index].width*req.cloud_list[cloud_index].height << "; i=" << cloud_index << "; process size: " << preprocess.request.tasks.size() );
	int max_attempts = 5;
	while(ros::ok() && !preprocessor.call(preprocess) && service_call_attempts<max_attempts)
	{
		service_call_attempts++;
		ROS_ERROR_STREAM("[PCRegistration] Attempt to call preprocessing on source cloud failed - sleeping 2 seconds and then trying again...");
		ros::Duration(2.0).sleep();
	};

	// --------- Service Population ---------
	ros::Duration preprocessing_time = ros::Time::now() - time_start_preprocessing;
	res.preprocessing_time.push_back(preprocessing_time.toSec());
	
	// If failed
	if(service_call_attempts > max_attempts)
	{
		ROS_ERROR_STREAM("[PCRegistration] Failed to preprocess " << cloud_index << "th cloud " << max_attempts << " times. Returning unprocessed...");
		pointcloud_processing_server::pointcloud_task_result failed_task_result;
		failed_task_result.task_pointcloud = req.cloud_list[cloud_index];
		res.preprocessing_results.push_back(failed_task_result);
		return false;
	}
	// Else - right now, only saving the last task result from each preprocess! Otherwise need to make an extra message (vector of vector of task_results...)
	int preprocess_size = preprocess.request.tasks.size();
	res.preprocessing_results.push_back(preprocess.response.task_results[preprocess_size-1]);
	return true;
}

bool PCRegistration::postprocessing(pointcloud_registration_server::registration_service::Request& req, pointcloud_registration_server::registration_service::Response& res, PCP output_cloud)
{
	ros::ServiceClient postprocessor = nh_.serviceClient<pointcloud_processing_server::pointcloud_process>("pointcloud_process");
	int service_call_attempts = 0;

	ros::Time time_start_postprocessing = ros::Time::now();
	// --------- Source Cloud Preprocessing ---------
	pointcloud_processing_server::pointcloud_process postprocess;
	for(int i=0; i<req.source_tasks.size(); i++)
	{
		postprocess.request.tasks.push_back(req.source_tasks[i]);
	}
	pcl::toROSMsg(*output_cloud, postprocess.request.pointcloud);
	postprocess.request.pointcloud.header.stamp = ros::Time::now();
	postprocess.request.min_cloud_size = 100;
	ROS_DEBUG_STREAM("[PCRegistration] Preprocessing a cloud. Size: " << output_cloud->size() << "; process size: " << postprocess.request.tasks.size() );
	int max_attempts = 5;
	while(ros::ok() && !postprocessor.call(postprocess) && service_call_attempts<max_attempts)
	{
		service_call_attempts++;
		ROS_ERROR_STREAM("[PCRegistration] Attempt to call postprocessing on source cloud failed - sleeping 2 seconds and then trying again...");
		ros::Duration(2.0).sleep();
	};

	// --------- Service Population ---------
	ros::Duration postprocessing_time = ros::Time::now() - time_start_postprocessing;
	res.postprocessing_time.push_back(postprocessing_time.toSec());
	
	// If failed
	if(service_call_attempts > max_attempts)
	{
		ROS_ERROR_STREAM("[PCRegistration] Failed to postprocess output cloud " << max_attempts << " times. Returning unprocessed...");
		return false;
	}
	// Else - right now, only saving the last task result from each postprocess! Otherwise need to make an extra message (vector of vector of task_results...)
	int postprocess_size = postprocess.request.tasks.size();
	res.postprocessing_results.push_back(postprocess.response.task_results[postprocess_size-1]);
	return true;
}

int main (int argc, char **argv)
{ 
	ros::init(argc, argv, "pc_registration");

//	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
 //   	ros::console::notifyLoggerLevelsChanged();

	ROS_DEBUG_STREAM("[PCRegistration] Started up node.");

	PCRegistration registration_server;

	return 0;
}
