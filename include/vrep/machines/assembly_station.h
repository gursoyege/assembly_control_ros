#pragma once

#include "machine.hpp"

#include <assembly_control_ros/assembly_station_state.h>
#include <assembly_control_ros/assembly_station_command.h>

using AssemblyStation = Machine<assembly_control_ros::assembly_station_state,
                                assembly_control_ros::assembly_station_command>;
