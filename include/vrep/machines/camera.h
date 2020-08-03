#pragma once

#include "machine.hpp"

#include <assembly_control_ros/camera_state.h>
#include <assembly_control_ros/camera_command.h>

using Camera = Machine<assembly_control_ros::camera_state,
                       assembly_control_ros::camera_command>;
