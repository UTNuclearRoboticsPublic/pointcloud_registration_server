
#ifndef REGISTER_POINTCLOUDS
#define REGISTER_POINTCLOUDS

#include "pointcloud_processing_server/server.h"
#include "pointcloud_registration_server/registration_service.h"
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/transforms.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>


//typedef pcl::PointXYZ PointT;
//typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointXYZRGBNormal PCLPointNormal;
typedef pcl::PointCloud<PCLPointNormal> PCN;
typedef pcl::PointCloud<PCLPointNormal>::Ptr PCNP;


// -------------------------------------------
// *** Service Call *** 
//   Not a part of the below class, so that the service itself doesn't have to be templated 
//   This means that subsequent runs can be made with the same instantiation, without restarting the service node 
//   For now, going to include point type specifications in the service object as the same #DEFINES listed above 
bool registerPointclouds(pointcloud_registration_server::registration_service::Request& req, pointcloud_registration_server::registration_service::Response& res);


// -------------------------------------------
// *** Registration Class ***
//   Templated on input/output point type and feature type to be used in registration
template <typename PointType, typename FeatureType>
class PCRegistration
{
public:
	PCRegistration();
	typedef typename pcl::PointCloud<PointType> PC;
	typedef typename pcl::PointCloud<PointType>::Ptr PCP;
	typedef typename pcl::PointCloud<FeatureType> FC;
	typedef typename pcl::PointCloud<FeatureType>::Ptr FCP;

	class MyPointRepresentation : public pcl::PointRepresentation <PCLPointNormal>
	{
    	using pcl::PointRepresentation<PCLPointNormal>::nr_dimensions_;
  	public:
    	MyPointRepresentation ()
    	{
	      	// Define the number of dimensions
	      	nr_dimensions_ = 4;
	    }

	    // Override the copyToFloatArray method to define our feature vector
	    virtual void copyToFloatArray (const PCLPointNormal &p, float * out) const
	    {
			// < x, y, z, curvature >
			out[0] = p.x;
			out[1] = p.y;
			out[2] = p.z;
			out[3] = p.curvature;
	    }
  	};

private:
	ros::NodeHandle nh_;
	// --------------------------------------------------------------------------------------
	// *** Basic Service Subcalls ***
	//   Each of these handles choices between calls to different algorithms available (specified in algorithm_type parameter)
	//   eg Transform Estimation might choose between ICP and NDT estimation methods 
	// 1) PREPROCESSING: voxelization, clipping, transforms, segmentation...
	bool preprocessing(pointcloud_registration_server::registration_service::Request& req, pointcloud_registration_server::registration_service::Response& res, int cloud_index);
  	// 2) INTEREST_POINTS: allows selection of a subset of points which are considered valuable for registration
  	void interestPointGeneration(const PCP input_cloud, const PCP interest_point_cloud, int algorithm_type); 
  	// 3) FEATURES: reparameterize cloud into some form which may be more fitting for registration 
  	void featureEstimation(const PCP input_cloud, const FCP feature_cloud, int algorithm_type);
  	// 4) CORRESPONDENCES: estimates the relationships between individual points in the target clouds which may represent the same ground-truth points
  	void correspondenceEstimation(const FCP feature_cloud, const PCP correspondence_cloud, int algorithm_type);
  	// 5) TRANSFORM: determines the transform which satisfies the most correspondences
	void transformEstimation(const FCP source_cloud, const FCP target_cloud, const FCP transformed_source, Eigen::Matrix4f transform, int algorithm_type);
	void transformEstimation(const FCP source_cloud, const FCP target_cloud, const FCP transformed_source, Eigen::Matrix4f transform, int transform_type, Eigen::Matrix4f initial_transform_guess);
	// 6) POSTPROCESSING: voxelization, clipping, transforms, segmentation...
  	bool postprocessing(pointcloud_registration_server::registration_service::Request& req, pointcloud_registration_server::registration_service::Response& res, int cloud_index);
	// --------------------------------------------------------------------------------------
  	// *** Interest Point Estimation Methods *** 

  	// -------------------------------------------
  	// *** Feature Estimation Methods ***
  	void featureEstimationNormals(const PCP input_cloud, const FCP feature_cloud);
  	// -------------------------------------------
  	// *** Correspondence Estimation Methods ***

	// -------------------------------------------
  	// *** Transform Estimation Methods ***
  	void transformEstimationNDT(const FCP source_cloud, const FCP target_cloud, const FCP transformed_source, Eigen::Matrix4f transform, Eigen::Matrix4f init_guess);
  	void transformEstimationICP(const FCP source_cloud, const FCP target_cloud, const FCP transformed_source, Eigen::Matrix4f transform, Eigen::Matrix4f init_guess);
  	// --------------------------------------------------------------------------------------

  	// ----------- Features Paramters -----------
  	int features_ksearch_;

  	// ----------- Transform Paramters -----------
  	int transform_epsilon_;
  	int transform_step_size_; 
  	int transform_resolution_;
  	int transform_max_iterations_;
  	float transform_max_dist_;
  	float transform_alpha_;
};

#endif // REGISTER_POINTCLOUDS