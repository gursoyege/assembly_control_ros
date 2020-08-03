#include <ros/ros.h>

#include <vrep/machines/machines.h>
#include <vrep/simulator.h>

#include <atomic>
#include <mutex>

int main(int argc, char* argv[]) {
    namespace assem = assembly_control_ros;

    ros::init(argc, argv, "vrep");

    ros::NodeHandle node;
    ros::NodeHandle node_params("~");

    std::string vrep_ip;
    if (not node_params.getParam("vrep_ip", vrep_ip)) {
        ROS_ERROR("Cannot get vrep_ip parameter");
        return -1;
    }

    Simulator simulator(50, vrep_ip);
    Simulator::Commands commands;

    SupplyConveyor supply_conveyor(node, simulator, "supply_conveyor",
                                   [](assem::supply_conveyor_state& message,
                                      const Simulator::State& state) {
                                       message.optical_barrier =
                                           state.optical_barrier;
                                   });

    EvacuationConveyor evacuation_conveyor(
        node, simulator, "evacuation_conveyor",
        [](assem::evacuation_conveyor_state& message,
           const Simulator::State& state) {
            message.stopped = state.evacuation_conveyor_stopped;
        });

    Camera camera(
        node, simulator, "camera",
        [](assem::camera_state& message, const Simulator::State& state) {
            message.done = state.recognition_complete;
            if (state.part1_detected) {
                message.part = 1;
            } else if (state.part2_detected) {
                message.part = 2;
            } else if (state.part3_detected) {
                message.part = 3;
            } else {
                message.part = 0;
            }
        });

    AssemblyStation assembly_station(node, simulator, "assembly_station",
                                     [](assem::assembly_station_state& message,
                                        const Simulator::State& state) {
                                         message.valid = state.assembly_valid;
                                         message.evacuated =
                                             state.assembly_evacuated;
                                     });

    Robot robot(node, simulator, "robot",
                [](assem::robot_state& message, const Simulator::State& state) {
                    message.part_grasped = state.grasped;
                    message.part_released = state.released;
                    message.at_supply_conveyor = state.at_supply_conveyor;
                    message.at_evacuation_conveyor =
                        state.at_evacuation_conveyor;
                    message.at_assembly_station = state.at_assembly_station;
                    message.part1_assembled = state.part1_assembled;
                    message.part2_assembled = state.part2_assembled;
                    message.part3_assembled = state.part3_assembled;
                });

    while (ros::ok()) {
        simulator.sync();

        auto supply_conveyor_command = supply_conveyor();
        auto evacuation_conveyor_command = evacuation_conveyor();
        auto camera_command = camera();
        auto assembly_station_command = assembly_station();
        auto robot_command = robot();

        commands.supply_conveyor_on = supply_conveyor_command.on;
        commands.evacuation_conveyor_on = evacuation_conveyor_command.on;
        commands.start_recognition = camera_command.process;
        commands.check_assembly = assembly_station_command.check;
        commands.move_right = robot_command.move_right;
        commands.move_left = robot_command.move_left;
        commands.grasp = robot_command.grasp;
        commands.release = robot_command.release;
        commands.assemble_part1 = robot_command.assemble_part1;
        commands.assemble_part2 = robot_command.assemble_part2;
        commands.assemble_part3 = robot_command.assemble_part3;
        simulator.setCommands(commands);

        ros::spinOnce();
    }
}