
#include "pointcloud_registration_server/pointcloud_registration.h"

PCRegistration::PCRegistration()
{
	ros::ServiceServer server = nh_.advertiseService("register_pointclouds", &PCRegistration::registerPointclouds, this);

	ros::spin();
}
	
bool PCRegistration::registerPointclouds(pointcloud_registration_server::registration_service::Request& req, pointcloud_registration_server::registration_service::Response& res)
{	
	ros::Time callback_received_time = ros::Time::now();

	ROS_DEBUG_STREAM("[PCRegistration] Received service callback.");
	PCP source_cloud = PCP(new PC);
	PCP target_cloud = PCP(new PC);
	PCP output_cloud_ = PCP(new PC);
	pcl::fromROSMsg(req.cloud_list[0], *output_cloud_);

	for (int i=1; i<req.cloud_list.size(); i++)
	{
		ros::Time registration_start_time = ros::Time::now();

		preprocessing(req, res, i, output_cloud_);

		int preprocessing_length = res.source_preprocessing_results.size();

		pcl::fromROSMsg(res.source_preprocessing_results[preprocessing_length-1].task_pointcloud, *source_cloud);
		pcl::fromROSMsg(res.target_preprocessing_results[preprocessing_length-1].task_pointcloud, *target_cloud);

		Eigen::Matrix4f final_transform;

		if(req.registration_type == 1)
		{
			ROS_DEBUG_STREAM("[PCRegistration] Running " << i <<"th ICP registration process.");
			float alpha[4] = {req.parameters[2], req.parameters[3], req.parameters[4], req.parameters[5]};

			registerICP(source_cloud, target_cloud, output_cloud_, final_transform, req.epsilon, req.max_iterations, req.parameters[0], req.parameters[1], alpha);
		}
		else if(req.registration_type == 2)
		{
			ROS_DEBUG_STREAM("[PCRegistration] Running " << i <<"th NDT registration process.");
			registerNDT(source_cloud, target_cloud, output_cloud_, final_transform, req.epsilon, req.max_iterations, req.parameters[0], req.parameters[1]);
		}
		else
		{
			ROS_ERROR_STREAM("[PCRegistration] Registration type selected for " << i << "th registration, " << req.registration_type << ", is not (yet?) supported.");
		}
		
		ROS_DEBUG_STREAM("[PCRegistration] Successfully registered a pair of clouds. Transformation is: ");

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

		res.transforms.push_back(output_pose);

		ros::Duration registration_duration = ros::Time::now() - registration_start_time;
		res.registration_time.push_back(registration_duration.toSec());
	}
	ROS_DEBUG_STREAM("[PCRegistration] Finished all registration processing.");

	// MAKE THIS LOGIC MORE COMPLICATED - allow to make whole map unvoxelized, unclipped, etc... 
	// May not actually need whole-map updating online - could just save many small (but registered) clouds and access a few nearby ones at a time, then stitch at end
	pcl::toROSMsg(*output_cloud_, res.output_cloud);
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

	*output_cloud += *target_cloud;

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

	//ROS_DEBUG_STREAM("first cloud: " << points_with_normals_source.height << " " << points_with_normals_source.width << " " << points_with_normals_source.height*points_with_normals_source.width);
	//ROS_DEBUG_STREAM("second cloud: " << points_with_normals_target.height << " " << points_with_normals_target.width << " " << points_with_normals_target.height*points_with_normals_target.width);	

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
	source_to_target = reg.getFinalTransformation () * source_to_target;

	//if the difference between this transformation and the previous one
	//is smaller than the threshold, refine the process by reducing
	//the maximal correspondence distance
	// This only works if I reincorporate the loop from the tutorial - not sure if this is necessary / warranted
	//  if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
	//  reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);

	prev = reg.getLastIncrementalTransformation ();

	// Get the transformation from target to source
	target_to_source = source_to_target.inverse();
	ROS_DEBUG_STREAM("[PCRegistration] Found final transformation.");

	// Transform target back in source frame
	pcl::transformPointCloud (*target_cloud, *output_cloud, target_to_source);
	//add the source to the transformed target
	*output_cloud += *source_cloud;
	final_transform = target_to_source;

	ros::Duration registration_duration = ros::Time::now() - time_start;
	ROS_DEBUG_STREAM("[PCRegistration] Finished ICP Registration! Entire registration process took " << registration_duration << " seconds.");
}

bool PCRegistration::preprocessing(pointcloud_registration_server::registration_service::Request& req, pointcloud_registration_server::registration_service::Response& res, int cloud_index, PCP output_cloud)
{
	ros::ServiceClient preprocessor = nh_.serviceClient<pointcloud_processing_server::pointcloud_process>("pointcloud_process");
	int service_call_attempts = 0;

	ros::Time time_start_preprocessing = ros::Time::now();
	// --------- Source Cloud Preprocessing ---------
	pointcloud_processing_server::pointcloud_process source_process;
	for(int i=0; i<req.source_tasks.size(); i++)
	{
		source_process.request.tasks.push_back(req.source_tasks[i]);
	}
	source_process.request.pointcloud = req.cloud_list[cloud_index];
	source_process.request.pointcloud.header.stamp = ros::Time::now();
	source_process.request.min_cloud_size = 100;
	ROS_DEBUG_STREAM("[PCRegistration] Preprocessing source cloud. Size: " << req.cloud_list[cloud_index].width*req.cloud_list[cloud_index].height << "; i=" << cloud_index << "; process size: " << source_process.request.tasks.size() );
	while(ros::ok() && !preprocessor.call(source_process) && service_call_attempts<5)
	{
		service_call_attempts++;
		ROS_ERROR_STREAM("[PCRegistration] Attempt to call preprocessing on source cloud failed - sleeping 2 seconds and then trying again...");
		ros::Duration(2.0).sleep();
	};
	ros::Duration source_preprocessing_time = ros::Time::now() - time_start_preprocessing;
	
	// --------- Target Cloud Preprocessing ---------
	pointcloud_processing_server::pointcloud_process target_process;
	for(int i=0; i<req.source_tasks.size(); i++)
		target_process.request.tasks.push_back(req.source_tasks[i]);
	pcl::toROSMsg(*output_cloud, target_process.request.pointcloud);
	target_process.request.pointcloud.header.stamp = ros::Time::now();
	target_process.request.min_cloud_size = 100;
	ROS_DEBUG_STREAM("[PCRegistration] Preprocessing target cloud. Size: " << output_cloud->size() << "; process size: " << source_process.request.tasks.size() );
	while(ros::ok() && !preprocessor.call(target_process) && service_call_attempts<5)
	{
		service_call_attempts++;
		ROS_ERROR_STREAM("[PCRegistration] Attempt to call preprocessing on target cloud failed - sleeping 2 seconds and then trying again...");
		ros::Duration(2.0).sleep();
	};
	ros::Duration target_preprocessing_time = ros::Time::now() - time_start_preprocessing - source_preprocessing_time;
	
	// --------- Service Population ---------
	res.source_preprocessing_results = source_process.response.task_results;
	res.target_preprocessing_results = target_process.response.task_results;
	res.source_preprocessing_time.push_back(source_preprocessing_time.toSec());
	res.target_preprocessing_time.push_back(target_preprocessing_time.toSec());

}

bool PCRegistration::postprocessing()
{

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
