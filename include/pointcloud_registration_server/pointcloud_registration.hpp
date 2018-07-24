

// ------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------
// --------------------------------------------- Class Implementations ----------------------------------------------
// ------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------

template <typename PointType, typename FeatureType>
PCRegistration<PointType, FeatureType>::PCRegistration(pointcloud_registration_server::registration_service::Request& req,
													   pointcloud_registration_server::registration_service::Response& res )
{
	// parameter assignment - defaults for now...
  	registration_failed_ = false;
  	// ----------- Features Paramters -----------
  	features_ksearch_ = 30;
  	// ----------- Correspondence Paramters -----------
  	// ----------- Transform Paramters -----------
  	transform_epsilon_ = 0.01;	//0.0005 for NDT...
  	transform_step_size_ = 0.1; 
  	transform_resolution_ = 0.5;
  	transform_max_iterations_ = 35;
  	transform_max_dist_ = 0.1;
  	transform_alpha_ = 1; // probably won't work like this... a vector? 

	setUpClouds(req);

	for(int i=0; i<req.cloud_list.size(); i++)
	{
		// Pre-/Postprocessing (Filters, etc.)
		preprocessing_results_	.push_back ( preprocessing(input_clouds_[i], preprocessed_clouds_[i], req.preprocessing_tasks    ));
		postprocessing_results_	.push_back ( postprocessing(input_clouds_[i], postprocessed_clouds_[i], req.postprocessing_tasks ));
		// Registration Processes 
		interestPointGeneration		( preprocessed_clouds_[i], interest_point_clouds_[i], req.interest_point_type );
		featureEstimation			( interest_point_clouds_[i], feature_clouds_[i] );
		correspondenceEstimation	( feature_clouds_[i], correspondence_clouds_[i], req.correspondence_search_type );
	}
	ROS_INFO_STREAM("finished first round of processes... trying to actually register now");
	for(int i=1; i<req.cloud_list.size(); i++)
	{
		ROS_INFO_STREAM("[PCRegistration] Attempting " << i << "th round of transforms.");
		ROS_DEBUG_STREAM("[PCRegistration]   Input cloud sizes are " << input_clouds_[i-1]->points[30].x << " " << input_clouds_[i-1]->points.size() << " and " << input_clouds_[i]->points.size());
		ROS_DEBUG_STREAM("[PCRegistration]   Preprocessed cloud sizes are " << preprocessed_clouds_[i-1]->points[30].x << " " << preprocessed_clouds_[i-1]->points.size() << " and " << preprocessed_clouds_[i]->points.size());
		ROS_DEBUG_STREAM("[PCRegistration]   Interest Point cloud sizes are " << interest_point_clouds_[i-1]->points[30].x << " " << interest_point_clouds_[i-1]->points.size() << " and " << interest_point_clouds_[i]->points.size());
		ROS_DEBUG_STREAM("[PCRegistration]   Feature cloud sizes are " << feature_clouds_[i-1]->points[30].x << " " << feature_clouds_[i-1]->points.size() << " and " << feature_clouds_[i]->points.size());
		ROS_DEBUG_STREAM("[PCRegistration]   Correspondence cloud sizes are " << correspondence_clouds_[i-1]->points[30].x << " "  << correspondence_clouds_[i-1]->points.size() << " and " << correspondence_clouds_[i]->points.size());
		Eigen::Matrix4f current_transform = Eigen::Matrix4f::Identity();
		*transformed_clouds_[i] = *correspondence_clouds_[i];
		while( ros::ok() && ( num_iterations_[i] == 0 || (req.repeatedly_register && num_iterations_[i] < req.max_transform_loops && ( translation_offset_[i] > req.translation_threshold || rotation_offset_[i] > req.rotation_threshold ) ) ) )
		{
			// Actually Register
			transformEstimation(correspondence_clouds_[i-1], transformed_clouds_[i], transformed_clouds_[i], current_transform, req.transformation_search_type);
			// Check Last Step Size
			rotation_offset_[i] = sqrt(pow( acos((static_cast<double>(current_transform(0,0))+static_cast<double>(current_transform(1,1))+static_cast<double>(current_transform(2,2))-1)/2) ,2));
			translation_offset_[i] = sqrt(pow(static_cast<double>(current_transform(0,3)),2) + pow(static_cast<double>(current_transform(1,3)),2) + pow(static_cast<double>(current_transform(2,3)),2));
			ROS_DEBUG_STREAM("[PCRegistration]   Completed " << num_iterations_[i] << "th transformation for " << i << "th source cloud. Most recent transform translation: " << translation_offset_[i] << " and rotation: " << rotation_offset_[i]);
			// Overall Transform
			transform_results_[i] = transform_results_[i]*current_transform;
			ROS_DEBUG_STREAM("[PCRegistration]   Current Overall Transformation is: ");
			ROS_DEBUG_STREAM("   Translation:  x: " << static_cast<double>(transform_results_[i](0,3)) << "  y: " << static_cast<double>(transform_results_[i](1,3)) << "  z: " << static_cast<double>(transform_results_[i](2,3)));
			ROS_DEBUG_STREAM("   Rotation:     x: " << static_cast<double>(transform_results_[i](0,0)) << "  y: " << static_cast<double>(transform_results_[i](1,1)) << "  z: " << static_cast<double>(transform_results_[i](2,2)));
			num_iterations_[i]++;
		}
	}

	populateServiceOutputs(res, req.transform_raw_clouds);
}

// --------------------------------------------------------------------------------------

template <typename PointType, typename FeatureType>
void PCRegistration<PointType, FeatureType>::setUpClouds(pointcloud_registration_server::registration_service::Request& req)
{
	pointcloud_processor_ = nh_.serviceClient<pointcloud_processing_server::pointcloud_process>("pointcloud_service");
	for(int i=0; i<req.cloud_list.size(); i++)
	{
		// Declare new clouds
		PC input, preprocessed, interest_points, postprocessed;
		FC features, correspondences, transformed;
		pcl::fromROSMsg(req.cloud_list[i], input);
		// Populate lists of clouds
		input_clouds_			.push_back ( input.makeShared() 			);
		preprocessed_clouds_	.push_back ( preprocessed.makeShared() 		);
		interest_point_clouds_	.push_back ( interest_points.makeShared() 	);
		feature_clouds_			.push_back ( features.makeShared() 			);
		correspondence_clouds_	.push_back ( correspondences.makeShared() 	);
		postprocessed_clouds_	.push_back ( postprocessed.makeShared() 	);
		transformed_clouds_ 	.push_back ( transformed.makeShared() 		);
		transform_results_ 		.push_back ( Eigen::Matrix4f::Identity() 	);
		num_iterations_ 		.push_back ( 0 );
		translation_offset_ 	.push_back ( 0.0 );
		rotation_offset_ 		.push_back ( 0.0 );
		// Initialize input cloud from service request
		ROS_DEBUG_STREAM(i << "th input cloud has size " << input_clouds_[i]->points.size());
	}
}

// ---------------------------------------------------------

// --------------------------------------------------------------------------------------
// Interest Point Generation 
//   Generate a cloud of points which are considered especially salient (different from surroundings, 'important' for registration)
//   
template <typename PointType, typename FeatureType>
void PCRegistration<PointType, FeatureType>::interestPointGeneration(const PCP input_cloud, const PCP interest_point_cloud, int algorithm_type)
{
	switch(algorithm_type)
	{
		// If NO interest points are desired
		case pointcloud_registration_server::registration_service::Request::INTEREST_TYPE_NONE:
		{
			*interest_point_cloud = *input_cloud;
			break;
		}
		// If SIFT interest points are selected
		case pointcloud_registration_server::registration_service::Request::INTEREST_TYPE_SIFT:
		{
			//interestPointGenerationSIFT(input_cloud, interest_point_cloud);
			break;
		}
		// No interest point type is specified
		default:
		{
			ROS_WARN_STREAM("[PCRegistration] No interest point type specified... defaulting to not using interest points.");
			*interest_point_cloud = *input_cloud;
			break;
		}
	}
}
// --------------------------------------------------------------------------------------



// ---------------------------------------------------------
// No Features Estimation (PointXYZ -> PointXYZ)
//   The following are functions for cases where no features are found - input type is the same as feature type 
template <>
void PCRegistration<pcl::PointXYZ, pcl::PointXYZ>::featureEstimation(const PCP input_cloud, const FCP feature_cloud)
{
	*feature_cloud = *input_cloud;
}
template <>
void PCRegistration<pcl::PointXYZI, pcl::PointXYZI>::featureEstimation(const PCP input_cloud, const FCP feature_cloud)
{
	*feature_cloud = *input_cloud;
}
template <>
void PCRegistration<pcl::PointNormal, pcl::PointNormal>::featureEstimation(const PCP input_cloud, const FCP feature_cloud)
{
	*feature_cloud = *input_cloud;
}
template <>
void PCRegistration<pcl::PointXYZINormal, pcl::PointXYZINormal>::featureEstimation(const PCP input_cloud, const FCP feature_cloud)
{
	*feature_cloud = *input_cloud;
}
template <>
void PCRegistration<pcl::PointXYZRGB, pcl::PointXYZRGB>::featureEstimation(const PCP input_cloud, const FCP feature_cloud)
{
	*feature_cloud = *input_cloud;
}
template <>
void PCRegistration<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>::featureEstimation(const PCP input_cloud, const FCP feature_cloud)
{
	*feature_cloud = *input_cloud;
}

// ---------------------------------------------------------
// ---------------------------------------------------------
// Normal Estimation
//   Evaluate surface normals at each point in an input cloud
// Templated for Individual Types: 
//   Each of the following individual type pair setups calls the normal estimation function below
template <>
void PCRegistration<pcl::PointXYZ, pcl::PointNormal>::featureEstimation(const PCP input_cloud, const FCP feature_cloud)
{
	featureEstimationNormals(input_cloud, feature_cloud);
}
template <>
void PCRegistration<pcl::PointXYZI, pcl::PointXYZINormal>::featureEstimation(const PCP input_cloud, const FCP feature_cloud)
{
	featureEstimationNormals(input_cloud, feature_cloud);
}
template <>
void PCRegistration<pcl::PointXYZRGB, pcl::PointXYZRGBNormal>::featureEstimation(const PCP input_cloud, const FCP feature_cloud)
{
	featureEstimationNormals(input_cloud, feature_cloud);
}
// Actual Normal Generation Function
template <typename PointType, typename FeatureType>
void PCRegistration<PointType, FeatureType>::featureEstimationNormals(const PCP input_cloud, const FCP feature_cloud)
{	
	ROS_DEBUG_STREAM("[PCRegistration] Normal Estimation - finding surface normals. First, removing all NaNs from cloud. Initial size is " << input_cloud->points.size());

	// Remove NaNs from cloud (otherwise norm calc may fail)
	std::vector<int> index_source;
	pcl::removeNaNFromPointCloud(*input_cloud, *input_cloud, index_source);
	ROS_DEBUG_STREAM("[PCRegistration]   Following NaN removal, new cloud size is " << input_cloud->points.size());

	pcl::NormalEstimation<PointType, FeatureType> norm_est;
	typedef typename pcl::search::KdTree<PointType>::Ptr KDTree;
	KDTree tree (new pcl::search::KdTree<PointType> ());
	norm_est.setSearchMethod (tree);
	norm_est.setKSearch (features_ksearch_);
	norm_est.setInputCloud (input_cloud);

	// Perform Computation
	norm_est.compute (*feature_cloud);
	pcl::copyPointCloud(*input_cloud, *feature_cloud);
	ROS_DEBUG_STREAM("[PCRegistration]   Final PointNormal cloud size is " << feature_cloud->points.size());
}
// ---------------------------------------------------------


// --------------------------------------------------------------------------------------




// --------------------------------------------------------------------------------------
template <typename PointType, typename FeatureType>
void PCRegistration<PointType, FeatureType>::correspondenceEstimation(const FCP feature_cloud, const FCP correspondence_cloud, int correspondence_type)
{
	switch(correspondence_type)
	{
		case pointcloud_registration_server::registration_service::Request::CORRESP_TYPE_NONE:
		{
			*correspondence_cloud = *feature_cloud;
			break;
		}
		default:
		{
			if(typeid(PointType).name()==typeid(FeatureType).name())
			{
				ROS_WARN_STREAM("[PCRegistration] No correspondence estimation algorithm specified... continuing without correspondences.");
				*feature_cloud = *correspondence_cloud;
			}
			else
			{
				ROS_ERROR_STREAM("[PCRegistration] No correspondence estimation algorithm specified... Exiting!");
				return;
			}
			break;
		}
	}
}

// --------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------
template <typename PointType, typename FeatureType>
void PCRegistration<PointType, FeatureType>::transformEstimation(const FCP target_cloud, const FCP source_cloud, const FCP transformed_source, Eigen::Matrix4f &transform, int transform_type)
{
	Eigen::Matrix4f initial_transform_guess = Eigen::Matrix4f::Identity();
	transformEstimation(source_cloud, target_cloud, transformed_source, transform, initial_transform_guess, transform_type);
}
template <typename PointType, typename FeatureType>
void PCRegistration<PointType, FeatureType>::transformEstimation(const FCP target_cloud, const FCP source_cloud, const FCP transformed_source, Eigen::Matrix4f &transform, Eigen::Matrix4f initial_transform_guess, int transform_type)
{
	switch(transform_type)
	{
		case pointcloud_registration_server::registration_service::Request::TRANSFORM_METHOD_NDT:
		{
			transformEstimationNDT(source_cloud, target_cloud, transformed_source, transform, initial_transform_guess);
			break;
		}
		case pointcloud_registration_server::registration_service::Request::TRANSFORM_METHOD_ICP:
		{
			transformEstimationICP(source_cloud, target_cloud, transformed_source, transform, initial_transform_guess);
			break;
		}
		default:
		{

			break;
		}
	}
}

template <typename PointType, typename FeatureType>
void PCRegistration<PointType, FeatureType>::transformEstimationNDT(const FCP target_cloud, const FCP source_cloud, const FCP transformed_source, Eigen::Matrix4f &transform, Eigen::Matrix4f init_guess)
{	
	ROS_DEBUG_STREAM("[PCRegistration] Starting NDT Process!  Epsilon: " << transform_epsilon_ << "  Max_It: " << transform_max_iterations_ << "  Step Size: " << transform_step_size_ << "  Res: " << transform_resolution_);

	// Initializing Normal Distributions Transform (NDT).
	pcl::NormalDistributionsTransform<FeatureType, FeatureType> ndt;

	// Setting scale dependent NDT parameters
	// Setting minimum transformation difference for termination condition.
	ndt.setTransformationEpsilon (transform_epsilon_);
	// Setting maximum step size for More-Thuente line search.
	ndt.setStepSize (transform_step_size_);
	//Setting Resolution of NDT grid structure (VoxelGridCovariance).
	ndt.setResolution (transform_resolution_);
	// Setting max number of registration iterations.
	ndt.setMaximumIterations (transform_max_iterations_);

	// Setting point cloud to be aligned.
	ndt.setInputSource (source_cloud);
	// Setting point cloud to be aligned to.
	ndt.setInputTarget (target_cloud);

	// Calculating required rigid transform to align the input cloud to the target cloud.
	ndt.align (*transformed_source, init_guess);
	transform = ndt.getFinalTransformation();
}


template <typename PointType, typename FeatureType>
void PCRegistration<PointType, FeatureType>::transformEstimationICP(const FCP target_cloud, const FCP source_cloud, const FCP transformed_source, Eigen::Matrix4f &transform, Eigen::Matrix4f init_guess)
{	
	ROS_DEBUG_STREAM("[PCRegistration] Starting a call to register two clouds via ICP. Epsilon: " << transform_epsilon_ << " Max_dist: " << transform_max_dist_ << " Max_iterations: " << transform_max_iterations_ );
	// Align
	pcl::IterativeClosestPointNonLinear<FeatureType, FeatureType> reg;
	reg.setTransformationEpsilon (transform_epsilon_);
	// Set the maximum distance between two correspondences (src<->tgt) to 10cm
	// Note: adjust this based on the size of your datasets
	reg.setMaxCorrespondenceDistance (transform_max_dist_);  

	// Remove NaNs, else estimate may crash
	std::vector<int> index_source_normal, index_target_normal;
	pcl::removeNaNFromPointCloud(*source_cloud, *source_cloud, index_source_normal);
	pcl::removeNaNFromPointCloud(*target_cloud, *target_cloud, index_target_normal);	
	ROS_DEBUG_STREAM("[PCRegistration]   Removed NaNs from clouds. New sizes are " << source_cloud->points.size() << " and " << target_cloud->points.size());

	reg.setInputSource (source_cloud);
	reg.setInputTarget (target_cloud);

	// Estimate
	ros::Time before = ros::Time::now();
	reg.align (*transformed_source);
	transform = reg.getFinalTransformation();
}
// --------------------------------------------------------------------------------------


template <typename PointType, typename FeatureType>
pointcloud_processing_server::pointcloud_task_result PCRegistration<PointType, FeatureType>::preprocessing(	
				const PCP input_cloud, 
				const PCP preprocessed_cloud, 
				std::vector<pointcloud_processing_server::pointcloud_task> preprocessing_tasks)
{
	pointcloud_processing_server::pointcloud_process preprocess;
	for(int i=0; i<preprocessing_tasks.size(); i++)
		preprocess.request.tasks.push_back(preprocessing_tasks[i]);
	toROSMsg(*input_cloud, preprocess.request.pointcloud);
	preprocess.request.min_cloud_size = 100;
	ROS_DEBUG_STREAM("[PCRegistration] Preprocessing a cloud. Size: " << input_cloud->width * input_cloud->height << "; process size: " << preprocess.request.tasks.size() );
	int max_attempts = 5;
	int service_call_attempts = 0;
	while(ros::ok() && !pointcloud_processor_.call(preprocess) && service_call_attempts<max_attempts)
	{
		service_call_attempts++;
		ROS_ERROR_STREAM("[PCRegistration] Attempt to call preprocessing on source cloud failed - sleeping 2 seconds and then trying again...");
		ros::Duration(2.0).sleep();
	};
	// If failed
	if(service_call_attempts > max_attempts || preprocess.request.tasks.size() < 1)
	{
		ROS_ERROR_STREAM("[PCRegistration] Failed to preprocess cloud " << max_attempts << " times. Returning unprocessed...");
		pointcloud_processing_server::pointcloud_task_result failed_task_result;
		pcl::toROSMsg(*input_cloud, failed_task_result.task_pointcloud);
		*preprocessed_cloud = *input_cloud;
		return failed_task_result;
	}
	// Else - right now, only saving the last task result from each preprocess! Otherwise need to make an extra message (vector of vector of task_results...)
	ROS_DEBUG_STREAM("[PCRegistration]   Pointcloud_Processing_Server call successful. Finishing up preprocess stuff...");
	int preprocess_size = preprocess.request.tasks.size();
	pcl::fromROSMsg(preprocess.response.task_results[preprocess_size-1].task_pointcloud, *preprocessed_cloud);
	return preprocess.response.task_results[preprocess_size-1];
}


template <typename PointType, typename FeatureType>
pointcloud_processing_server::pointcloud_task_result PCRegistration<PointType, FeatureType>::postprocessing(	
				const PCP input_cloud, 
				const PCP postprocessed_cloud, 
				std::vector<pointcloud_processing_server::pointcloud_task> postprocessing_tasks)
{
	pointcloud_processing_server::pointcloud_process postprocess;
	for(int i=0; i<postprocessing_tasks.size(); i++)
		postprocess.request.tasks.push_back(postprocessing_tasks[i]);
	toROSMsg(*input_cloud, postprocess.request.pointcloud);
	postprocess.request.min_cloud_size = 100;
	ROS_DEBUG_STREAM("[PCRegistration] Postprocessing a cloud. Size: " << input_cloud->width * input_cloud->height << "; process size: " << postprocess.request.tasks.size() );
	int max_attempts = 5;
	int service_call_attempts = 0;
	while(ros::ok() && !pointcloud_processor_.call(postprocess) && service_call_attempts<max_attempts)
	{
		service_call_attempts++;
		ROS_ERROR_STREAM("[PCRegistration] Attempt to call postprocessing on source cloud failed - sleeping 2 seconds and then trying again...");
		ros::Duration(2.0).sleep();
	};
	ROS_DEBUG_STREAM("[PCRegistration]   Pointcloud_Processing_Server call successful. Finishing up postprocess stuff...");

	// If failed
	if(service_call_attempts > max_attempts || postprocess.request.tasks.size() < 1)
	{
		ROS_ERROR_STREAM("[PCRegistration] Failed to postprocess cloud " << max_attempts << " times. Returning unprocessed...");
		pointcloud_processing_server::pointcloud_task_result failed_task_result;
		pcl::toROSMsg(*input_cloud, failed_task_result.task_pointcloud);
		*postprocessed_cloud = *input_cloud;
		return failed_task_result;
	}
	// Else - right now, only saving the last task result from each postprocess! Otherwise need to make an extra message (vector of vector of task_results...)
	ROS_DEBUG_STREAM("[PCRegistration]   Pointcloud_Processing_Server call successful. Finishing up postprocess stuff...");
	int postprocess_size = postprocess.request.tasks.size();
	pcl::fromROSMsg(postprocess.response.task_results[postprocess_size-1].task_pointcloud, *postprocessed_cloud);
	return postprocess.response.task_results[postprocess_size-1];
}



// --------------------------------------------------------------------------------------
// *** Outputs ***
// Populate service outputs from clouds and transforms generated inside class. 
template <typename PointType, typename FeatureType>
bool PCRegistration<PointType, FeatureType>::populateServiceOutputs(pointcloud_registration_server::registration_service::Response& res, bool transform_raw_clouds)
{
	res.preprocessing_results = preprocessing_results_;
	res.postprocessing_results = postprocessing_results_;

	sensor_msgs::PointCloud2 full_cloud;
	sensor_msgs::PointCloud2 previous_full_cloud;

	ROS_INFO_STREAM("got to output population; list size " << transformed_clouds_.size());
	for(int i=0; i<transformed_clouds_.size(); i++)
	{
		// *** Transforms ***
		// Translation
		geometry_msgs::Transform transform_msg;
		transform_msg.translation.x = static_cast<double>(transform_results_[i](0,3));
		transform_msg.translation.y = static_cast<double>(transform_results_[i](1,3));
		transform_msg.translation.z = static_cast<double>(transform_results_[i](2,3));
		// Rotation
		tf::Matrix3x3 rotation_matrix;
		rotation_matrix.setValue(	static_cast<double>(transform_results_[i](0,0)), static_cast<double>(transform_results_[i](0,1)), static_cast<double>(transform_results_[i](0,2)), 
									static_cast<double>(transform_results_[i](1,0)), static_cast<double>(transform_results_[i](1,1)), static_cast<double>(transform_results_[i](1,2)),
									static_cast<double>(transform_results_[i](2,0)), static_cast<double>(transform_results_[i](2,1)), static_cast<double>(transform_results_[i](2,2)));
		tf::Quaternion tf_quaternion;
		tf::Transform transform_tf;
		rotation_matrix.getRotation(tf_quaternion);
		transform_msg.rotation.x = tf_quaternion.getAxis().getX();
		transform_msg.rotation.y = tf_quaternion.getAxis().getY();
		transform_msg.rotation.z = tf_quaternion.getAxis().getZ();
		transform_msg.rotation.w = tf_quaternion.getAngle();
		// Push Back Result
		res.transforms.push_back(transform_msg);
		// *** Transformed Raw Clouds ***
		Eigen::Affine3f affine_transform;
		affine_transform.matrix() = transform_results_[i];
		if(transform_raw_clouds)
		{
			PCP transformed_raw_cloud(new PC());
			pcl::transformPointCloud(*input_clouds_[i], *transformed_raw_cloud, affine_transform);
			transformed_raw_clouds_.push_back(transformed_raw_cloud);
			sensor_msgs::PointCloud2 transformed_cloud_msg;
			pcl::toROSMsg(*transformed_raw_clouds_[i], transformed_cloud_msg);
			res.individual_clouds.push_back(transformed_cloud_msg);
		}
		// *** Transformed Postprocessed Clouds ***
		PCP transformed_postprocessed_cloud(new PC());
		pcl::transformPointCloud(*input_clouds_[i], *transformed_postprocessed_cloud, affine_transform);
		transformed_postprocessed_clouds_.push_back(transformed_postprocessed_cloud);
		sensor_msgs::PointCloud2 transformed_postprocessed_msg;
		pcl::toROSMsg(*transformed_postprocessed_clouds_[i], transformed_postprocessed_msg);
		res.individual_clouds_postprocessed.push_back(transformed_postprocessed_msg);
		// *** Build Full Cloud ***
		pcl::concatenatePointCloud(previous_full_cloud, res.individual_clouds_postprocessed[i], res.output_cloud);
		previous_full_cloud = res.output_cloud;
		ROS_INFO_STREAM("pop " << i << " " << res.individual_clouds_postprocessed.size());
	}

	return false;
}