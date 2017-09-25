
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

class PCRegistration
{
public:
	PCRegistration();

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

  	bool registerPointclouds(pointcloud_registration_server::registration_service::Request& req, pointcloud_registration_server::registration_service::Response& res);
	bool preprocessing(pointcloud_registration_server::registration_service::Request& req, pointcloud_registration_server::registration_service::Response& res, int cloud_index);
  	bool postprocessing(pointcloud_registration_server::registration_service::Request& req, pointcloud_registration_server::registration_service::Response& res, PCP output_cloud);
  	void registerICP(const PCP source_cloud, const PCP target_cloud, Eigen::Matrix4f &final_transform, float epsilon, int max_iterations, int ksearch, float max_dist, float alpha[4]);
  	void registerNDT(const PCP source_cloud, const PCP target_cloud, Eigen::Matrix4f &final_transform, float epsilon, int max_iterations, float step_size, float resolution);
};

#endif // REGISTER_POINTCLOUDS