#pragma once

#include "machine.hpp"

#include <assembly_control_ros/evacuation_conveyor_state.h>
#include <assembly_control_ros/evacuation_conveyor_command.h>

using EvacuationConveyor =
    Machine<assembly_control_ros::evacuation_conveyor_state,
            assembly_control_ros::evacuation_conveyor_command>;
