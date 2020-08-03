#pragma once

#include "machine.hpp"

#include <assembly_control_ros/supply_conveyor_state.h>
#include <assembly_control_ros/supply_conveyor_command.h>

using SupplyConveyor = Machine<assembly_control_ros::supply_conveyor_state,
                               assembly_control_ros::supply_conveyor_command>;
