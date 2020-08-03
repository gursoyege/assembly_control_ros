#pragma once

#include "machine.hpp"

#include <assembly_control_ros/robot_state.h>
#include <assembly_control_ros/robot_command.h>

using Robot = Machine<assembly_control_ros::robot_state,
                      assembly_control_ros::robot_command>;
