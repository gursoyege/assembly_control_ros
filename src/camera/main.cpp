#include <ros/ros.h>

#include <assembly_control_ros/camera_state.h>
#include <assembly_control_ros/camera_command.h>
#include <assembly_control_ros/camera_input.h>
#include <assembly_control_ros/camera_output.h>

#include <common/machine_controller.hpp>

class Camera : public MachineController<assembly_control_ros::camera_state,
                                        assembly_control_ros::camera_input,
                                        assembly_control_ros::camera_command,
                                        assembly_control_ros::camera_output> {
public:
    Camera(ros::NodeHandle node)
        : MachineController(node, "camera"), state_(State::Wait) {
    }

    virtual void process() override {
        assembly_control_ros::camera_command commands;
        assembly_control_ros::camera_output outputs;

        auto& inputs = getInputs();

        switch (state_) {
        case State::Wait:
            if (inputs.start_recognition) {
                inputs.start_recognition = false;

                ROS_INFO("[Camera] Processing");
                state_ = State::Processing;
            }
            break;
        case State::Processing:
            commands.process = true;
            if (getState().done) {
                commands.process = false;

                ROS_INFO("[Camera] Ready");
                state_ = State::Ready;
            }
            break;
        case State::Ready:
            switch (getState().part) {
            case 1:
                ROS_INFO("[Camera] Part 1");
                outputs.part1 = true;
                outputs.part2 = false;
                outputs.part3 = false;
                outputs.part_analyzed = true;
                break;
            case 2:
                ROS_INFO("[Camera] Part 2");
                outputs.part1 = false;
                outputs.part2 = true;
                outputs.part3 = false;
                outputs.part_analyzed = true;
                break;
            case 3:
                ROS_INFO("[Camera] Part 3");
                outputs.part1 = false;
                outputs.part2 = false;
                outputs.part3 = true;
                outputs.part_analyzed = true;
                break;
            default:
                ROS_INFO("[Camera] Unknown part");
                outputs.part1 = false;
                outputs.part2 = false;
                outputs.part3 = false;
                outputs.part_analyzed = false;
                break;
            }

            sendOuputs(outputs);

            ROS_INFO("[Camera] Wait");
            state_ = State::Wait;
            break;
        }

        sendCommands(commands);
    }

private:
    enum class State { Wait, Processing, Ready };

    State state_;
};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "camera");

    ros::NodeHandle node;

    ros::Rate loop_rate(50); // 50 Hz

    Camera camera(node);

    while (ros::ok()) {
        camera.process();

        ros::spinOnce();

        loop_rate.sleep();
    }
}