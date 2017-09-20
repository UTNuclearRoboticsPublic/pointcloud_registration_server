
#ifndef REG_CREATION_H
#define REG_CREATION_H

#include <ros/ros.h>
#include "pointcloud_registration_server/registration_service.h"
#include "pointcloud_processing_server/pointcloud_task_creation.h"

namespace RegCreation
{
	bool registrationFromYAML(pointcloud_registration_server::registration_service *srv, std::string yaml_file_name);

};

#endif //REG_CREATION_H