#include <ros/ros.h>

#include <assembly_control_ros/assembly_station_state.h>
#include <assembly_control_ros/assembly_station_command.h>
#include <assembly_control_ros/assembly_station_input.h>
#include <assembly_control_ros/assembly_station_output.h>

#include <common/machine_controller.hpp>

class AssemblyStation
    : public MachineController<assembly_control_ros::assembly_station_state,
                               assembly_control_ros::assembly_station_input,
                               assembly_control_ros::assembly_station_command,
                               assembly_control_ros::assembly_station_output> {
public:
    AssemblyStation(ros::NodeHandle node)
        : MachineController(node, "assembly_station"), state_(State::Ready) {
    }

    virtual void process() override {
        assembly_control_ros::assembly_station_command commands;
        assembly_control_ros::assembly_station_output outputs;

        auto& inputs = getInputs();

        switch (state_) {
        case State::Ready:
            if (inputs.check) {

                ROS_INFO("[AssemblyStation] Checking");
                state_ = State::Checking;
            }
            break;
        case State::Checking:
            commands.check = true;
            if (getState().valid) {
                commands.check = false;
                ROS_INFO("[AssemblyStation] Evacuation");
                state_ = State::Evacuation;
            }
            break;
        case State::Evacuation:
            if (getState().evacuated) {
                outputs.evacuated = true;
                sendOuputs(outputs);

                ROS_INFO("[AssemblyStation] Ready");
                state_ = State::Ready;
            }
            break;
        }

        sendCommands(commands);
    }

private:
    enum class State { Ready, Checking, Evacuation };

    State state_;
};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "assembly_station");

    ros::NodeHandle node;

    ros::Rate loop_rate(50); // 50 Hz

    AssemblyStation assembly_station(node);

    while (ros::ok()) {
        assembly_station.process();

        ros::spinOnce();

        loop_rate.sleep();
    }
}
