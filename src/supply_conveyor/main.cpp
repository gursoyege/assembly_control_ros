#include <ros/ros.h>

#include <assembly_control_ros/supply_conveyor_state.h>
#include <assembly_control_ros/supply_conveyor_command.h>
#include <assembly_control_ros/supply_conveyor_input.h>
#include <assembly_control_ros/supply_conveyor_output.h>

#include <common/machine_controller.hpp>

class SupplyConveyor
    : public MachineController<assembly_control_ros::supply_conveyor_state,
                               assembly_control_ros::supply_conveyor_input,
                               assembly_control_ros::supply_conveyor_command,
                               assembly_control_ros::supply_conveyor_output> {
public:
    SupplyConveyor(ros::NodeHandle node)
        : MachineController(node, "supply_conveyor"), state_(State::On) {
    }

    virtual void process() override {
        assembly_control_ros::supply_conveyor_command commands;
        assembly_control_ros::supply_conveyor_output outputs;

        auto& inputs = getInputs();

        switch (state_) {
        case State::On:
            commands.on = true;
            if (getState().optical_barrier) {
                outputs.part_available = true;
                sendOuputs(outputs);

                ROS_INFO("[SupplyConveyor] Off");
                state_ = State::Off;
            }
            break;
        case State::Off:
            if (inputs.restart) {
                inputs.restart = false;

                ROS_INFO("[SupplyConveyor] On");
                state_ = State::On;
            }
            break;
        }

        sendCommands(commands);
    }

private:
    enum class State { On, Off };

    State state_;
};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "supply_conveyor");

    ros::NodeHandle node;

    ros::Rate loop_rate(50); // 50 Hz

    SupplyConveyor conveyor(node);

    while (ros::ok()) {
        conveyor.process();

        ros::spinOnce();

        loop_rate.sleep();
    }
}