#include <ros/ros.h>

#include <assembly_control_ros/evacuation_conveyor_state.h>
#include <assembly_control_ros/evacuation_conveyor_command.h>
#include <assembly_control_ros/evacuation_conveyor_input.h>
#include <assembly_control_ros/evacuation_conveyor_output.h>

#include <common/machine_controller.hpp>

class EvacuationConveyor
    : public MachineController<assembly_control_ros::evacuation_conveyor_state,
                               assembly_control_ros::evacuation_conveyor_input,
                               assembly_control_ros::evacuation_conveyor_command,
                               assembly_control_ros::evacuation_conveyor_output> {
public:
    EvacuationConveyor(ros::NodeHandle node)
        : MachineController(node, "evacuation_conveyor"), state_(State::On) {
    }

    virtual void process() override {
        assembly_control_ros::evacuation_conveyor_command commands;
        assembly_control_ros::evacuation_conveyor_output outputs;

        auto& inputs = getInputs();

        switch (state_) {
        case State::Off:
            if (inputs.on) {
                inputs.on = false;
                commands.on = true;
                ROS_INFO("[EvacuationConveyor] On");
                state_ = State::On;
            }
            break;
        case State::On:
            commands.on = true;
            if (inputs.can_stop) {
                commands.on = false;
                inputs.can_stop = false;
                ROS_INFO("[EvacuationConveyor] Stopping");
                state_ = State::Stopping;
            }   
            break;
        case State::Stopping:
            if (getState().stopped) {
                outputs.stopped = true;
                sendOuputs(outputs);
                ROS_INFO("[EvacuationConveyor] Off");
                state_ = State::Off;
            }
            break;   
        }
        sendCommands(commands);
    }

private:
    enum class State { Off, On, Stopping };

    State state_;
};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "evacuation_conveyor");

    ros::NodeHandle node;

    ros::Rate loop_rate(50); // 50 Hz

    EvacuationConveyor evacuation_conveyor(node);

    while (ros::ok()) {
        evacuation_conveyor.process();

        ros::spinOnce();

        loop_rate.sleep();
    }
}
