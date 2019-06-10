
#ifndef REGISTER_POINTCLOUDS
#define REGISTER_POINTCLOUDS

#include "pointcloud_processing_server/server.h"
#include "pointcloud_registration_server/registration_service.h"

#include <eigen_conversions/eigen_msg.h>


// Point Includes
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/kdtree/kdtree.h>

// Feature Includes
#include <pcl/features/normal_3d_omp.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <wall_features/point_wall_damage.h>
#include <wall_features/wall_damage_histogram.h>
#include <wall_features/wall_damage_estimation.h>
#include <wall_features/wall_damage_estimation.hpp>
#include <wall_features/wall_damage_service.h>
#include <wall_features/itwf.h>
#include <wall_features/itwf_estimation.h>

// Transform Includes
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <geometry_msgs/Transform.h>

// -------------------------------------------
// *** Registration Class ***
//   Templated on input/output point type and feature type to be used in registration
template <typename PointType, typename FeatureType>
class PCRegistration
{
	typedef typename pcl::PointCloud<PointType> PC;
	typedef typename pcl::PointCloud<PointType>::Ptr PCP;
	typedef typename pcl::PointCloud<FeatureType> FC;
	typedef typename pcl::PointCloud<FeatureType>::Ptr FCP;

public:
	PCRegistration(pointcloud_registration_server::registration_service::Request& req, pointcloud_registration_server::registration_service::Response& res);
 	void setUpClouds();

private:
	// --------------------------------------------------------------------------------------
	// *** Basic Processing Subcalls ***
	//   Each of these handles choices between calls to different algorithms available (specified in algorithm_type parameter)
	//   eg Transform Estimation might choose between ICP and NDT estimation methods 
	// 1) PREPROCESSING: voxelization, clipping, transforms, segmentation...
	pointcloud_processing_server::pointcloud_task_result preprocessing(const PCP input_cloud, const PCP preprocessed_cloud, std::vector<pointcloud_processing_server::pointcloud_task> preprocessing_tasks);
  	// 2) INTEREST_POINTS: allows selection of a subset of points which are considered valuable for registration
  	void interestPointGeneration(const PCP preprocessed_cloud, const PCP interest_point_cloud, int algorithm_type); 
  	// 3) FEATURES: reparameterize cloud into some form which may be more fitting for registration 
  	void featureEstimation(const PCP input_cloud, const PCP interest_point_cloud, const FCP feature_cloud);
  	// 4) CORRESPONDENCES: estimates the relationships between individual points in the target clouds which may represent the same ground-truth points
  	void correspondenceEstimation(const FCP target_features, const FCP source_features, const pcl::CorrespondencesPtr correspondence_cloud, int correspondence_type);
  	// 5) TRANSFORM: determines the transform which satisfies the most correspondences
	  void transformEstimation(const FCP target_cloud, const FCP source_cloud, Eigen::Matrix4f &transform, pcl::CorrespondencesPtr correspondences, int algorithm_type);
	  void transformEstimation(const FCP target_cloud, const FCP source_cloud, Eigen::Matrix4f &transform, pcl::CorrespondencesPtr correspondences, Eigen::Matrix4f initial_transform_guess, int transform_type);
	// 6) POSTPROCESSING: voxelization, clipping, transforms, segmentation...
  	pointcloud_processing_server::pointcloud_task_result postprocessing(const PCP input_cloud, const PCP postprocessed_cloud, std::vector<pointcloud_processing_server::pointcloud_task> postprocessing_tasks);
	// --------------------------------------------------------------------------------------
  	// *** Interest Point Estimation Methods *** 
    void interestPointGenerationSIFTExplicit(const PCP input_cloud, const PCP interest_point_cloud) {
      interestPointDisagreement(input_cloud, interest_point_cloud); };
    void interestPointGenerationSIFT(const PCP input_cloud, const PCP interest_point_cloud);
    bool checkIfMaximum(const PCP input_cloud, const PCP interest_point_cloud, std::vector<int> nearest_indices, int source_index) { 
      interestPointDisagreement(input_cloud, interest_point_cloud); };
    void interestPointGenerationWallMaximaExplicit(const PCP input_cloud, const PCP interest_point_cloud) {
      interestPointDisagreement(input_cloud, interest_point_cloud); };
    void interestPointGenerationWallMaxima(const PCP input_cloud, const PCP interest_point_cloud);
    void interestPointDisagreement(const PCP input_cloud, const PCP interest_point_cloud)
    {
      ROS_WARN_STREAM("[PCRegistration] Wall Maxima Keypoints requested, but Wall Maxima cannot be computed on this input point type. Interest point cloud will be equal to input cloud.");
      *interest_point_cloud = *input_cloud;
    };

  	// -------------------------------------------
  	// *** Feature Estimation Methods ***
  	void featureEstimationNone(const PCP input_cloud, const PCP  interest_point_cloud, const FCP feature_cloud);
  	void featureEstimationNormals(const PCP input_cloud, const PCP  interest_point_cloud, const FCP feature_cloud);
    void featureDisagreement(const PCP input_cloud, const PCP interest_point_cloud)
    {
      ROS_WARN_STREAM("[PCRegistration] Wall Maxima Keypoints requested, but Wall Maxima cannot be computed on this input point type. Interest point cloud will be equal to input cloud.");
      *interest_point_cloud = *input_cloud;
    }; 
    void featureEstimationITWFFull(const PCP input_cloud, const PCP interest_point_cloud, const FCP feature_cloud);
    void featureEstimationITWFBasic(const pcl::PointCloud<pcl::PointWallDamage>::Ptr input_cloud, const pcl::PointCloud<pcl::PointWallDamage>::Ptr interest_point_cloud, const pcl::PointCloud<pcl::ITWFSignature84>::Ptr feature_cloud);


  	// -------------------------------------------
  	// *** Correspondence Estimation Methods ***

    // -------------------------------------------
  	// *** Transform Estimation Methods ***
  	void transformEstimationNDT(const FCP target_cloud, const FCP source_cloud, Eigen::Matrix4f &transform, Eigen::Matrix4f init_guess);
  	void transformEstimationICP(const FCP target_cloud, const FCP source_cloud, Eigen::Matrix4f &transform, Eigen::Matrix4f init_guess);
    void transformEstimationCorrespondenceRANSAC(const FCP target_cloud, const FCP source_cloud, Eigen::Matrix4f &transform, pcl::CorrespondencesPtr correspondences);
    void transformEstimationSVD(const FCP target_cloud, const FCP source_cloud, Eigen::Matrix4f &transform, pcl::CorrespondencesPtr correspondences);

  	// --------------------------------------------------------------------------------------
  	// *** Outputs ***
  	// Populate service outputs from clouds and transforms generated inside class. 
  	bool populateServiceOutputs();

  	// --------------------------------------------------------------------------------------
    // ----------- ROS Service Object Pointers -----------
    pointcloud_registration_server::registration_service::Request * req_;
    pointcloud_registration_server::registration_service::Response * res_;

  	// --------------------------------------------------------------------------------------
  	// ----------- Overall Paramters -----------
  	bool registration_failed_;
  	std::vector<PCP> input_clouds_;
  	ros::NodeHandle nh_;
  	std::vector<float> translation_offset_;	// Translational distance offset of the most recent transform
  	std::vector<float> rotation_offset_;	// Rotational distance offset of the most recent transform
 
  	// ----------- Preprocessing Paramters -----------
  	ros::ServiceClient pointcloud_processor_;
  	std::vector<PCP> preprocessed_clouds_;
  	std::vector<pointcloud_processing_server::pointcloud_task_result> preprocessing_results_;

  	// ----------- Interest Point Paramters -----------
  	int interest_point_type_; 
  	std::vector<PCP> interest_point_clouds_;

  	// ----------- Features Paramters -----------
  	int feature_type_;
  	std::vector<FCP> feature_clouds_;

  	// ----------- Correspondence Paramters -----------
  	int correspondence_type_;
  	std::vector<pcl::CorrespondencesPtr> correspondences_;

  	// ----------- Transform Paramters -----------
  	int transform_type_;
  	std::vector<Eigen::Matrix4f> transform_results_;
    std::vector<FCP> transformed_feature_clouds_;
  	std::vector<PCP> transformed_raw_clouds_;
  	std::vector<PCP> transformed_postprocessed_clouds_;

  	// ----------- Postprocessing Paramters -----------
  	std::vector<PCP> postprocessed_clouds_;
  	std::vector<pointcloud_processing_server::pointcloud_task_result> postprocessing_results_;

};


// Include implementation (here instead of in cpp, to allow for implicit template instantiations)
#include "pointcloud_registration_server/pointcloud_registration.hpp"


#endif // REGISTER_POINTCLOUDS