

// ------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------
// --------------------------------------------- Class Implementations ----------------------------------------------
// ------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------

template <typename PointType, typename FeatureType>
PCRegistration<PointType, FeatureType>::PCRegistration(pointcloud_registration_server::registration_service::Request& req,
													   pointcloud_registration_server::registration_service::Response& res )
{
	// Fix service objects 
	*req_ = req;
	*res_ = res;

	setUpClouds();

	// Preprocessing, Interest Point Estimation, Feature Estimation
	for(int i=0; i<req_->cloud_list.size(); i++)
	{
		// Pre-/Postprocessing (Filters, etc.)
		preprocessing_results_	.push_back ( preprocessing(input_clouds_[i], preprocessed_clouds_[i], req_->preprocessing_tasks    ));
		postprocessing_results_	.push_back ( postprocessing(input_clouds_[i], postprocessed_clouds_[i], req_->postprocessing_tasks ));
		// Registration Processes 
		interestPointGeneration		( preprocessed_clouds_[i], interest_point_clouds_[i], req_->interest_point_type );
		featureEstimation			( preprocessed_clouds_[i], interest_point_clouds_[i], feature_clouds_[i] );
	}
	// Correspondences and Transform Estimation 
	ROS_INFO_STREAM("finished first round of processes... trying to actually register now");
	for(int i=1; i<req_->cloud_list.size(); i++)
	{
		ROS_INFO_STREAM("[PCRegistration] Attempting " << i << "th round of transforms.");
		ROS_DEBUG_STREAM("[PCRegistration]   Input cloud sizes are " << input_clouds_[i-1]->points.size() << " and " << input_clouds_[i]->points.size());
		ROS_DEBUG_STREAM("[PCRegistration]   Preprocessed cloud sizes are " << preprocessed_clouds_[i-1]->points.size() << " and " << preprocessed_clouds_[i]->points.size());
		ROS_DEBUG_STREAM("[PCRegistration]   Interest Point cloud sizes are " << interest_point_clouds_[i-1]->points.size() << " and " << interest_point_clouds_[i]->points.size());
		ROS_DEBUG_STREAM("[PCRegistration]   Feature cloud sizes are " << feature_clouds_[i-1]->points.size() << " and " << feature_clouds_[i]->points.size());
		ROS_DEBUG_STREAM("[PCRegistration] Estimating correspondences...");
		// Correspondences
		correspondenceEstimation	( feature_clouds_[i-1], feature_clouds_[i], correspondences_[i], req_->correspondence_search_type );
		ROS_DEBUG_STREAM("[PCRegistration] Found " << correspondences_[i]->size() << " correspondences.");
		// Transform Estimation
		transformEstimation( feature_clouds_[i-1], feature_clouds_[i], transform_results_[i], correspondences_[i], req_->transformation_search_type );
		ROS_DEBUG_STREAM("[PCRegistration]   Registered two clouds. Overall Transformation is: ");
		ROS_DEBUG_STREAM("   Translation:  x: " << static_cast<double>(transform_results_[i](0,3)) << "  y: " << static_cast<double>(transform_results_[i](1,3)) << "  z: " << static_cast<double>(transform_results_[i](2,3)));
		ROS_DEBUG_STREAM("   Rotation:     x: " << static_cast<double>(transform_results_[i](0,0)) << "  y: " << static_cast<double>(transform_results_[i](1,1)) << "  z: " << static_cast<double>(transform_results_[i](2,2)));
	}

	populateServiceOutputs();
}

// --------------------------------------------------------------------------------------

template <typename PointType, typename FeatureType>
void PCRegistration<PointType, FeatureType>::setUpClouds()
{
	pointcloud_processor_ = nh_.serviceClient<pointcloud_processing_server::pointcloud_process>("pointcloud_service");
	for(int i=0; i<req_->cloud_list.size(); i++)
	{
		// Declare new clouds
		PC input, preprocessed, interest_points, postprocessed;
		PC transformed_raw_cloud, transformed_postprocessed_cloud;
		FC features, transformed_feature_cloud;
		pcl::CorrespondencesPtr correspondences;
		pcl::fromROSMsg(req_->cloud_list[i], input);
		// Populate lists of clouds
		input_clouds_						.push_back ( input.makeShared() 						  );
		preprocessed_clouds_				.push_back ( preprocessed.makeShared() 					  );
		interest_point_clouds_				.push_back ( interest_points.makeShared() 				  );
		feature_clouds_						.push_back ( features.makeShared() 						  );
		correspondences_					.push_back ( correspondences 				  			  );
		postprocessed_clouds_				.push_back ( postprocessed.makeShared() 				  );
		transformed_feature_clouds_ 		.push_back ( transformed_feature_cloud.makeShared() 	  );
		transformed_raw_clouds_ 			.push_back ( transformed_raw_cloud.makeShared() 		  );
		transformed_postprocessed_clouds_ 	.push_back ( transformed_postprocessed_cloud.makeShared() );
		transform_results_ 					.push_back ( Eigen::Matrix4f::Identity() 				  );
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
			interestPointGenerationSIFTExplicit(input_cloud, interest_point_cloud);
			break;
		}
		// If ITWF interest points are selected
		case pointcloud_registration_server::registration_service::Request::INTEREST_TYPE_WALL_MAXIMA:
		{
			interestPointGenerationWallMaxima(input_cloud, interest_point_cloud);
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
template <>
bool PCRegistration<pcl::PointWallDamage, pcl::ITWFSignature84>::checkIfMaximum(const PCP input_cloud, const PCP interest_point_cloud, std::vector<int> nearest_indices, int source_index)
{
	int neighbor_index = 0;
	// Maxima Types
	bool is_depth_maximum = true;
	bool is_intensity_maximum = true;
	bool is_intensity_minimum = true;
	bool is_r_maximum = true;
	bool is_g_maximum = true;
	bool is_b_maximum = true;

	ROS_INFO_STREAM_THROTTLE(0.5, "made a search, starting to iterate through points, neighbor list size is " << nearest_indices.size());

	// Maxima check loop (iterates over neighbors within search window)
	while ( neighbor_index < nearest_indices.size() && (is_depth_maximum || is_intensity_maximum || is_intensity_minimum || is_r_maximum || is_g_maximum || is_b_maximum))
	{
		// Skip self-matches
		if (source_index != nearest_indices[neighbor_index])
		{
		  // For brevity and readability of code:
		  pcl::PointWallDamage source_point = input_cloud->points[source_index];
		  pcl::PointWallDamage target_point = input_cloud->points[nearest_indices[neighbor_index]];

		  // Check Depth Maximum
		  if ( is_depth_maximum )
		    if ( source_point.depth_offset >= target_point.depth_offset )// source_point.depth_offset*source_point.depth_offset <= target_point.depth_offset*source_point.depth_offset )
		      is_depth_maximum = false;
		  // Check Intensity Maximum
		  if ( is_intensity_maximum )
		    if ( source_point.intensity <= target_point.intensity )
		      is_intensity_maximum = false;
		  // Check Intensity Minimum
		  if ( is_intensity_minimum )
		    if ( source_point.intensity >= target_point.intensity )
		      is_intensity_minimum = false;
		  // Check Red Maximum
		  if ( is_r_maximum )
		    if ( source_point.r <= target_point.r )
		      is_r_maximum = false;
		  // Check Green Maximum
		  if ( is_g_maximum )
		    if ( source_point.g <= target_point.g )
		      is_g_maximum = false;
		  // Check Blue Maximum
		  if ( is_b_maximum )
		    if ( source_point.b <= target_point.b )
		      is_b_maximum = false;
		}
		// Increment index to consider next neighbor
		neighbor_index++;

		ROS_INFO_STREAM_THROTTLE(0.5, "working on it... indices: " << source_index << " and " << neighbor_index << " and interest size: " << interest_point_cloud->points.size() << "; current window has " << nearest_indices.size() << " neighbors.");
	}
}

template <> 
void PCRegistration<pcl::PointXYZI, pcl::PointXYZI>::interestPointGenerationSIFTExplicit(const PCP input_cloud, const PCP interest_point_cloud)
{
	interestPointGenerationSIFT(input_cloud, interest_point_cloud);
}
template <> 
void PCRegistration<pcl::PointXYZI, pcl::PointXYZINormal>::interestPointGenerationSIFTExplicit(const PCP input_cloud, const PCP interest_point_cloud)
{
	interestPointGenerationSIFT(input_cloud, interest_point_cloud);
}
template <> 
void PCRegistration<pcl::PointXYZRGB, pcl::PointXYZRGB>::interestPointGenerationSIFTExplicit(const PCP input_cloud, const PCP interest_point_cloud)
{
	interestPointGenerationSIFT(input_cloud, interest_point_cloud);
}
template <> 
void PCRegistration<pcl::PointXYZRGB, pcl::PointXYZRGBNormal>::interestPointGenerationSIFTExplicit(const PCP input_cloud, const PCP interest_point_cloud)
{
	interestPointGenerationSIFT(input_cloud, interest_point_cloud);
}

template <> 
void PCRegistration<pcl::PointWallDamage, pcl::ITWFSignature84>::interestPointGenerationWallMaximaExplicit(const PCP input_cloud, const PCP interest_point_cloud)
{
	interestPointGenerationWallMaxima(input_cloud, interest_point_cloud);
}

// ---------------------------------------------------------
template <typename PointType, typename FeatureType>
void PCRegistration<PointType, FeatureType>::interestPointGenerationSIFT(const PCP input_cloud, const PCP interest_point_cloud)
{
	// EXPOSE THESE PARAMETERS
	//   sift_scale
	//   octave stuff
    typedef typename pcl::SIFTKeypoint<PointType, pcl::PointWithScale> SiftObject;
    SiftObject sift;
    pcl::PointCloud<pcl::PointWithScale>::Ptr sift_cloud(new pcl::PointCloud<pcl::PointWithScale>);
    typedef typename pcl::search::KdTree<PointType>::Ptr KDTreePtr;
    KDTreePtr sift_tree(new pcl::search::KdTree<PointType> ());
    sift.setSearchMethod(sift_tree);
    sift.setScales(0.01, 3, 4);
    sift.setMinimumContrast(0.001);
    sift.setInputCloud(input_cloud);
    sift.compute(*sift_cloud);

    ROS_DEBUG_STREAM("[PCRegistration] Found " << sift_cloud->points.size() << " sift keypoints in cloud.");

    // Build Interest Point Cloud
    for(int i=0; i<sift_cloud->points.size(); i++)
    {
    	PointType point;
    	point.x = sift_cloud->points[i].x;
    	point.y = sift_cloud->points[i].y;
    	point.z = sift_cloud->points[i].z;
    	interest_point_cloud->points.push_back(point);
    }
}

template <typename PointType, typename FeatureType>
void PCRegistration<PointType, FeatureType>::interestPointGenerationWallMaxima(const PCP input_cloud, const PCP interest_point_cloud)
{
	// EXPOSE THIS
	float search_radius = 0.05;

	ROS_INFO_STREAM("Maxima-based Interest Point search, with input cloud size: " << input_cloud->points.size() << " and search window size " << search_radius);
	
	typedef typename pcl::search::KdTree<PointType> KDTreePtr;
	KDTreePtr tree;
	tree.setInputCloud(input_cloud);
	
	// Used by PCL KDTree object
	std::vector<int> nearest_indices;
	std::vector<float> nearest_dist_squareds;
	// Iterate over all points in the cloud
	for (int i=0; i<input_cloud->points.size(); i++)
	{
		ROS_INFO_STREAM_THROTTLE(0.5, "starting a search " << i);
		if( tree.radiusSearch (input_cloud->points[i], search_radius, nearest_indices, nearest_dist_squareds) > 0 );
		{
			if ( checkIfMaximum(input_cloud, interest_point_cloud, nearest_indices, i) )
				interest_point_cloud->points.push_back(input_cloud->points[i]);
		}
	}
}



// ---------------------------------------------------------
// No Features Estimation (PointXYZ -> PointXYZ)
//   The following are functions for cases where no features are found - input type is the same as feature type 
template <>
void PCRegistration<pcl::PointXYZ, pcl::PointXYZ>::featureEstimation(const PCP input_cloud, const PCP interest_point_cloud, const FCP feature_cloud)
{
	*feature_cloud = *interest_point_cloud;
}
template <>
void PCRegistration<pcl::PointXYZI, pcl::PointXYZI>::featureEstimation(const PCP input_cloud, const PCP interest_point_cloud, const FCP feature_cloud)
{
	*feature_cloud = *interest_point_cloud;
}
template <>
void PCRegistration<pcl::PointNormal, pcl::PointNormal>::featureEstimation(const PCP input_cloud, const PCP interest_point_cloud, const FCP feature_cloud)
{
	*feature_cloud = *interest_point_cloud;
}
template <>
void PCRegistration<pcl::PointXYZINormal, pcl::PointXYZINormal>::featureEstimation(const PCP input_cloud, const PCP interest_point_cloud, const FCP feature_cloud)
{
	*feature_cloud = *interest_point_cloud;
}
template <>
void PCRegistration<pcl::PointXYZRGB, pcl::PointXYZRGB>::featureEstimation(const PCP input_cloud, const PCP interest_point_cloud, const FCP feature_cloud)
{
	*feature_cloud = *interest_point_cloud;
}
template <>
void PCRegistration<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>::featureEstimation(const PCP input_cloud, const PCP interest_point_cloud, const FCP feature_cloud)
{
	*feature_cloud = *interest_point_cloud;
}

// ---------------------------------------------------------
// ---------------------------------------------------------
// Normal Estimation
//   Evaluate surface normals at each point in an input cloud
// Templated for Individual Types: 
//   Each of the following individual type pair setups calls the normal estimation function below
template <>
void PCRegistration<pcl::PointXYZ, pcl::PointNormal>::featureEstimation(const PCP input_cloud, const PCP interest_point_cloud, const FCP feature_cloud)
{
	featureEstimationNormals(input_cloud, interest_point_cloud, feature_cloud);
}
template <>
void PCRegistration<pcl::PointXYZI, pcl::PointXYZINormal>::featureEstimation(const PCP input_cloud, const PCP interest_point_cloud, const FCP feature_cloud)
{
	featureEstimationNormals(input_cloud, interest_point_cloud, feature_cloud);
}
template <>
void PCRegistration<pcl::PointXYZRGB, pcl::PointXYZRGBNormal>::featureEstimation(const PCP input_cloud, const PCP interest_point_cloud, const FCP feature_cloud)
{
	featureEstimationNormals(input_cloud, interest_point_cloud, feature_cloud);
}
// Actual Normal Generation Function
template <typename PointType, typename FeatureType>
void PCRegistration<PointType, FeatureType>::featureEstimationNormals(const PCP input_cloud, const PCP interest_point_cloud, const FCP feature_cloud)
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
	norm_est.setKSearch (req_->feature_parameters[0]);
	norm_est.setInputCloud (input_cloud);

	// Perform Computation
	norm_est.compute (*feature_cloud);
	pcl::copyPointCloud(*input_cloud, *feature_cloud);
	ROS_DEBUG_STREAM("[PCRegistration]   Final PointNormal cloud size is " << feature_cloud->points.size());
}


// ---------------------------------------------------------
// ---------------------------------------------------------
// ITWF Estimation
template <>
void PCRegistration<pcl::PointXYZ, pcl::ITWFSignature84>::featureEstimation(const PCP input_cloud, const PCP interest_point_cloud, const FCP feature_cloud)
{
	featureEstimationITWFFull(input_cloud, interest_point_cloud, feature_cloud);
}

template <>
void PCRegistration<pcl::PointXYZI, pcl::ITWFSignature84>::featureEstimation(const PCP input_cloud, const PCP interest_point_cloud, const FCP feature_cloud)
{
	featureEstimationITWFFull(input_cloud, interest_point_cloud, feature_cloud);
}

template <>
void PCRegistration<pcl::PointXYZRGB, pcl::ITWFSignature84>::featureEstimation(const PCP input_cloud, const PCP interest_point_cloud, const FCP feature_cloud)
{
	featureEstimationITWFFull(input_cloud, interest_point_cloud, feature_cloud);
}

/* need to actually implement PointXYZRGBI still...
template <>
void PCRegistration<pcl::PointXYZRGBI, pcl::ITWFSignature84>::featureEstimationITWFFull(const PCP input_cloud, const PCP interest_point_cloud, const FCP feature_cloud)
{
	featureEstimationITWFFull(input_cloud, interest_point_cloud, feature_cloud);
} */



// ITWF Estimation
template <typename PointType, typename FeatureType>
void PCRegistration<PointType, FeatureType>::featureEstimationITWFBasic(const pcl::PointCloud<pcl::PointWallDamage>::Ptr input_cloud, const pcl::PointCloud<pcl::PointWallDamage>::Ptr interest_point_cloud, const pcl::PointCloud<pcl::ITWFSignature84>::Ptr feature_cloud)
{
	pcl::ITWFEstimation<pcl::PointWallDamage, pcl::PointWallDamage, pcl::ITWFSignature84> itwf_estimation;
	// Input Wall Coefficients
	std::vector<float> wall_coefficients;
	for(int i=0; i<4; i++)
		wall_coefficients.push_back(req_->feature_parameters[i]);
    itwf_estimation.setWallCoordinateFrame(wall_coefficients);
    // Input Radius
    itwf_estimation.setSearchRadius(req_->feature_parameters[4]);
    // Input Dimensions to Use (out of depth, normals, intensity, color)
	std::vector<bool> dimension_use;
	for(int i=5; i<12; i++)
		dimension_use.push_back(req_->feature_parameters[i]);
    itwf_estimation.setDimensionUse(dimension_use);
    // Generate Outputs
    itwf_estimation.compute(*input_cloud, *interest_point_cloud, *feature_cloud);
}

template <>
void PCRegistration<pcl::PointWallDamage, pcl::ITWFSignature84>::featureEstimation(const PCP input_cloud, const PCP interest_point_cloud, const FCP feature_cloud)
{
	featureEstimationITWFBasic(input_cloud, interest_point_cloud, feature_cloud);
}

// ITWF Estimation
template <typename PointType, typename FeatureType>
void PCRegistration<PointType, FeatureType>::featureEstimationITWFFull(const PCP input_cloud, const PCP interest_point_cloud, const FCP feature_cloud)
{
	// PointWallDamage
	if(0) // somehow check whether point_wall_damage server is live
	{
		ROS_ERROR_STREAM("[PCRegistration] PointWallDamage estimation requested, but server is not live. Registration failing...");
		return;
	}
	wall_features::wall_damage_service wall_dmg_srv;
    wall_dmg_srv.request.load_from_bags = false;
    wall_dmg_srv.request.segmentation_cloud_voxel_size = req_->feature_parameters[12];
    wall_dmg_srv.request.wall_damage_voxel_size = req_->feature_parameters[13];
    wall_dmg_srv.request.wall_damage_histogram_voxel_size = req_->feature_parameters[14];
    wall_dmg_srv.request.k_search_normals = req_->feature_parameters[15];
    wall_dmg_srv.request.k_search_histogram = req_->feature_parameters[16];
    wall_dmg_srv.request.automatically_set_bins = true;
    wall_dmg_srv.request.primitive_search_name = "wall_damage_plane_search";
    wall_dmg_srv.request.damage_inliers_given = false;
    wall_dmg_srv.request.histogram_inliers_given = false;
    
    ros::ServiceClient wall_damage_client = nh_.serviceClient<wall_features::wall_damage_service>("wall_damage_estimation");

    // Dense Cloud
    sensor_msgs::PointCloud2 input_msg;
    pcl::toROSMsg(*input_cloud, input_msg);
    wall_dmg_srv.request.input_cloud = input_msg;
    wall_damage_client.call(wall_dmg_srv);
    pcl::PointCloud<pcl::PointWallDamage>::Ptr wall_damage_cloud(new pcl::PointCloud<pcl::PointWallDamage>);
    pcl::fromROSMsg(wall_dmg_srv.response.wall_damage_cloud, *wall_damage_cloud);

    // Interest Points (inefficient)
    pcl::PointCloud<pcl::PointWallDamage>::Ptr interest_points_damage(new pcl::PointCloud<pcl::PointWallDamage>());
    typedef typename pcl::search::KdTree<PointType> KdTree;
    KdTree tree;
    tree.setInputCloud(input_cloud);
    std::vector<int> nearest_indices;
    std::vector<float> nearest_dist_squareds;
    for(int i=0; i<wall_damage_cloud->points.size(); i++)
    {
    	if(!tree.nearestKSearch(input_cloud->points[i], 1, nearest_indices, nearest_dist_squareds))
    	{
    		ROS_ERROR_STREAM("[PCRegistration] Warning - while finding PointWallDamage cloud for interest points, interest point index " << i << " had no neighbors in the input cloud... Skipping this point.");
    		continue;
    	}
    	pcl::PointWallDamage point;
    	point = wall_damage_cloud->points[nearest_indices[0]];
    	interest_points_damage->points.push_back(point);
    }

	// ITWF
    featureEstimationITWFBasic(wall_damage_cloud, interest_points_damage, feature_cloud);
}

// ---------------------------------------------------------


// --------------------------------------------------------------------------------------




// --------------------------------------------------------------------------------------
template <typename PointType, typename FeatureType>
void PCRegistration<PointType, FeatureType>::correspondenceEstimation(const FCP target_features, const FCP source_features, const pcl::CorrespondencesPtr correspondences, int correspondence_type)
{
	switch(correspondence_type)
	{
		case pointcloud_registration_server::registration_service::Request::CORRESP_TYPE_NONE:
		{
			break;
		}
		default:
		{
			if(typeid(PointType).name()==typeid(FeatureType).name())
			{
				ROS_WARN_STREAM("[PCRegistration] No correspondence estimation algorithm specified... continuing without correspondences.");
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
void PCRegistration<PointType, FeatureType>::transformEstimation(const FCP target_cloud, const FCP source_cloud, Eigen::Matrix4f &transform, pcl::CorrespondencesPtr correspondences, int transform_type)
{
	Eigen::Matrix4f initial_transform_guess = Eigen::Matrix4f::Identity();
	transformEstimation(source_cloud, target_cloud, transform, correspondences, initial_transform_guess, transform_type);
}
template <typename PointType, typename FeatureType>
void PCRegistration<PointType, FeatureType>::transformEstimation(const FCP target_cloud, const FCP source_cloud, Eigen::Matrix4f &transform, pcl::CorrespondencesPtr correspondences, Eigen::Matrix4f initial_transform_guess, int transform_type)
{
	switch(transform_type)
	{
		case pointcloud_registration_server::registration_service::Request::TRANSFORM_METHOD_NDT:
		{
			transformEstimationNDT(source_cloud, target_cloud, transform, initial_transform_guess);
			break;
		}
		case pointcloud_registration_server::registration_service::Request::TRANSFORM_METHOD_ICP:
		{
			transformEstimationICP(source_cloud, target_cloud, transform, initial_transform_guess);
			break;
		}
		default:
		{

			break;
		}
	}
}

template <typename PointType, typename FeatureType>
void PCRegistration<PointType, FeatureType>::transformEstimationNDT(const FCP target_cloud, const FCP source_cloud, Eigen::Matrix4f &transform, Eigen::Matrix4f init_guess)
{	
	ROS_DEBUG_STREAM("[PCRegistration] Starting NDT Process!  Epsilon: " << req_->transform_parameters[0] << "  Max_It: " << req_->transform_parameters[1] << "  Step Size: " << req_->transform_parameters[2] << "  Res: " << req_->transform_parameters[3]);

	// Initializing Normal Distributions Transform (NDT).
	pcl::NormalDistributionsTransform<FeatureType, FeatureType> ndt;

	// Setting scale dependent NDT parameters
	// Setting minimum transformation difference for termination condition.
	ndt.setTransformationEpsilon (req_->transform_parameters[0]);
	// Setting max number of registration iterations.
	ndt.setMaximumIterations (req_->transform_parameters[1]);
	// Setting maximum step size for More-Thuente line search.
	ndt.setStepSize (req_->transform_parameters[2]);
	//Setting Resolution of NDT grid structure (VoxelGridCovariance).
	ndt.setResolution (req_->transform_parameters[3]);

	// Setting point cloud to be aligned.
	ndt.setInputSource (source_cloud);
	// Setting point cloud to be aligned to.
	ndt.setInputTarget (target_cloud);

	// Calculating required rigid transform to align the input cloud to the target cloud.
	FCP aligned_cloud(new FC());
	ndt.align (*aligned_cloud, init_guess);
	transform = ndt.getFinalTransformation();
}
template <>
void PCRegistration<pcl::PointWallDamage, pcl::ITWFSignature84>::transformEstimationNDT(const FCP target_cloud, const FCP source_cloud, Eigen::Matrix4f &transform, Eigen::Matrix4f init_guess)
{	
}
template <>
void PCRegistration<pcl::PointXYZ, pcl::ITWFSignature84>::transformEstimationNDT(const FCP target_cloud, const FCP source_cloud, Eigen::Matrix4f &transform, Eigen::Matrix4f init_guess)
{	
}
template <>
void PCRegistration<pcl::PointXYZI, pcl::ITWFSignature84>::transformEstimationNDT(const FCP target_cloud, const FCP source_cloud, Eigen::Matrix4f &transform, Eigen::Matrix4f init_guess)
{	
}

template <typename PointType, typename FeatureType>
void PCRegistration<PointType, FeatureType>::transformEstimationICP(const FCP target_cloud, const FCP source_cloud, Eigen::Matrix4f &transform, Eigen::Matrix4f init_guess)
{	
	pcl::IterativeClosestPointNonLinear<FeatureType, FeatureType> reg;
	reg.setTransformationEpsilon (req_->transform_parameters[0]);
	// Set the maximum distance between two correspondences (src<->tgt) to 10cm
	// Note: adjust this based on the size of your datasets
	reg.setMaximumIterations(req_->transform_parameters[1]);
	reg.setMaxCorrespondenceDistance (req_->transform_parameters[2]);  
	reg.setEuclideanFitnessEpsilon(req_->transform_parameters[3]);

	// Remove NaNs, else ICP may crash
	std::vector<int> index_source_normal, index_target_normal;
	pcl::removeNaNFromPointCloud(*source_cloud, *source_cloud, index_source_normal);
	pcl::removeNaNFromPointCloud(*target_cloud, *target_cloud, index_target_normal);	
	ROS_DEBUG_STREAM("[PCRegistration]   Removed NaNs from clouds. New sizes are " << source_cloud->points.size() << " and " << target_cloud->points.size());

	// Actually Register
	ROS_DEBUG_STREAM("[PCRegistration] Starting a call to register two clouds via ICP. Epsilon: " << req_->transform_parameters[0] << " Max_iterations: " << req_->transform_parameters[1] << " Max_dist: " << req_->transform_parameters[2] );
	reg.setInputSource (source_cloud);
	reg.setInputTarget (target_cloud);
	FCP aligned_cloud(new FC());
	reg.align (*aligned_cloud);
	transform = reg.getFinalTransformation();
}
template <>
void PCRegistration<pcl::PointWallDamage, pcl::ITWFSignature84>::transformEstimationICP(const FCP target_cloud, const FCP source_cloud, Eigen::Matrix4f &transform, Eigen::Matrix4f init_guess)
{
}
template <>
void PCRegistration<pcl::PointXYZ, pcl::ITWFSignature84>::transformEstimationICP(const FCP target_cloud, const FCP source_cloud, Eigen::Matrix4f &transform, Eigen::Matrix4f init_guess)
{
}	
template <>
void PCRegistration<pcl::PointXYZI, pcl::ITWFSignature84>::transformEstimationICP(const FCP target_cloud, const FCP source_cloud, Eigen::Matrix4f &transform, Eigen::Matrix4f init_guess)
{
}


template <typename PointType, typename FeatureType>
void PCRegistration<PointType, FeatureType>::transformEstimationCorrespondenceRANSAC(const FCP target_cloud, const FCP source_cloud, Eigen::Matrix4f &transform, pcl::CorrespondencesPtr correspondences)
{
	pcl::registration::CorrespondenceRejectorSampleConsensus<FeatureType> sac;
	sac.setInputSource(target_cloud);
    sac.setInputTarget(source_cloud);
    sac.setInlierThreshold(0.02);
    sac.setMaximumIterations(10000);
    sac.setInputCorrespondences(correspondences);
    pcl::CorrespondencesPtr output_correspondences(new pcl::Correspondences);
    sac.getCorrespondences(*output_correspondences);
    sac.setInputCorrespondences(output_correspondences);
    transform = sac.getBestTransformation();
}
template <typename PointType, typename FeatureType>
void PCRegistration<PointType, FeatureType>::transformEstimationSVD(const FCP target_cloud, const FCP source_cloud, Eigen::Matrix4f &transform, pcl::CorrespondencesPtr correspondences)
{
	pcl::registration::TransformationEstimationSVD<FeatureType, FeatureType> svd;
    svd.estimateRigidTransformation(*target_cloud, *source_cloud, *correspondences, transform);
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
bool PCRegistration<PointType, FeatureType>::populateServiceOutputs()
{
	res_->preprocessing_results = preprocessing_results_;
	res_->postprocessing_results = postprocessing_results_;

	sensor_msgs::PointCloud2 full_cloud;
	sensor_msgs::PointCloud2 previous_full_cloud;

	ROS_INFO_STREAM("got to output population; list size " << transform_results_.size());
	for(int i=0; i<transform_results_.size(); i++)
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
		res_->transforms.push_back(transform_msg);
		// *** Transformed Raw Clouds ***
		Eigen::Affine3f affine_transform;
		affine_transform.matrix() = transform_results_[i];
		if(req_->transform_feature_clouds)
		{
			sensor_msgs::PointCloud2 transformed_feature_msg;
			pcl::toROSMsg(*transformed_feature_clouds_[i], transformed_feature_msg);
			res_->transformed_feature_clouds.push_back(transformed_feature_msg);
		}
		if(req_->transform_raw_clouds)
		{
			PCP transformed_raw_cloud(new PC());
			pcl::transformPointCloud(*input_clouds_[i], *transformed_raw_cloud, affine_transform);
			transformed_raw_clouds_.push_back(transformed_raw_cloud);
			sensor_msgs::PointCloud2 transformed_cloud_msg;
			pcl::toROSMsg(*transformed_raw_clouds_[i], transformed_cloud_msg);
			res_->transformed_clouds.push_back(transformed_cloud_msg);
		}
		// *** Transformed Postprocessed Clouds ***
		PCP transformed_postprocessed_cloud(new PC());
		pcl::transformPointCloud(*input_clouds_[i], *transformed_postprocessed_cloud, affine_transform);
		transformed_postprocessed_clouds_.push_back(transformed_postprocessed_cloud);
		sensor_msgs::PointCloud2 transformed_postprocessed_msg;
		pcl::toROSMsg(*transformed_postprocessed_clouds_[i], transformed_postprocessed_msg);
		if(req_->transform_postprocessed_clouds)
		{
			res_->transformed_clouds_postprocessed.push_back(transformed_postprocessed_msg);
		}
		// *** Build Full Cloud ***
		pcl::concatenatePointCloud(previous_full_cloud, transformed_postprocessed_msg, res_->full_cloud);
		previous_full_cloud = res_->full_cloud;
	}

	res_->success = true;

	return false;
}