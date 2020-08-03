#include <ros/ros.h>

#include <assembly_control_ros/robot_state.h>
#include <assembly_control_ros/robot_command.h>
#include <assembly_control_ros/robot_input.h>
#include <assembly_control_ros/robot_output.h>

#include <common/machine_controller.hpp>

class Robot
    : public MachineController<assembly_control_ros::robot_state,
                               assembly_control_ros::robot_input,
                               assembly_control_ros::robot_command,
                               assembly_control_ros::robot_output> {
bool part1_exist = false;
bool part2_exist = false;
bool part3_exist = false;
bool Decision_Right_ToSc = false;
bool Decision_Right_ToEc = false;
bool Decision_Left_ToAs = false;
bool Decision_Left_ToSc = false;
bool Decision_Part1_2 = false;
bool Decision_Part2_2 = false;
bool Decision_Part3_2 = false;
bool at_as_begin = false;
int part_count = 0;

public:
    Robot(ros::NodeHandle node)
        : MachineController(node, "robot"), state_(State::At_As) {
    }

    virtual void process() override {
        assembly_control_ros::robot_command commands;
        assembly_control_ros::robot_output outputs;

        auto& inputs = getInputs();


        switch (state_) {
        case State::Idle:
            if (inputs.cam_done) {
                inputs.cam_done = false;
                ROS_INFO("[Robot] Grasp");
                state_ = State::Grasp;
            }
            break;
        case State::Grasp:
            commands.grasp = true;
            if (getState().part_grasped) {
                commands.grasp = false;
                outputs.grasped_at_sc = true;
                sendOuputs(outputs);
                ROS_INFO("[Robot] Decision_1");
                state_ = State::Decision_1;
            }
            break;
        case State::Decision_1:
                ROS_INFO("[Robot] Decision 1");
            if (part_count < 3 && part_count >= 0) {
                ROS_INFO("[Robot] Decision_2");
                state_ = State::Decision_2;
            }
            else if (part_count == 3) {
              part_count = 0;
              outputs.part_all_ok = true;
              sendOuputs(outputs);
              ROS_INFO("[Robot] Parts at AS == 3");
              if (inputs.assembly_all_done) {
                  inputs.assembly_all_done = false;
                  ROS_INFO("[Robot] Decision_2");
                  state_ = State::Decision_2;
              }
            }
            else {
                
              ROS_INFO("[Robot] Error in Decision_1");
            }
            break;
        case State::Decision_2:
            if ((inputs.part1 && part1_exist) || (inputs.part2 && part2_exist) || (inputs.part3 && part3_exist)) {
                ROS_INFO("[Robot] [%d,%d],[%d,%d],[%d,%d]",inputs.part1,part1_exist,inputs.part2,part2_exist,inputs.part3,part3_exist);
                inputs.part1 = false;
                inputs.part2 = false;
                inputs.part3 = false;
                ROS_INFO("[Robot] Decision_Evacuation");
                state_ = State::Decision_Evacuation;
            }
            else if (inputs.part1 && part1_exist == 0){
                inputs.part1 = false;
                ROS_INFO("[Robot] Decision_Part1_1");
                state_ = State::Decision_Part1_1;
            }
            else if (inputs.part2 && part2_exist == 0){
                inputs.part2 = false;
                ROS_INFO("[Robot] Decision_Part2_1");
                state_ = State::Decision_Part2_1;
            }
            else if (inputs.part3 && part3_exist == 0){
                inputs.part3 = false;
                ROS_INFO("[Robot] Decision_Part3_1");
                state_ = State::Decision_Part3_1;
            }
            break;
        case State::Decision_Part1_1: 
                Decision_Part1_2 = true;
                Decision_Left_ToAs = true;
                ROS_INFO("[Robot] Left");
                state_ = State::Left;
            
            break;
        case State::Decision_Part2_1: 
                Decision_Part2_2 = true;
                Decision_Left_ToAs = true;
                ROS_INFO("[Robot] Left");
                state_ = State::Left;
            
            break;
        case State::Decision_Part3_1: 
                Decision_Part3_2 = true;
                Decision_Left_ToAs = true;
                ROS_INFO("[Robot] Left");
                state_ = State::Left;

            break;
        case State::Left:
            commands.move_left = true;
            if (getState().at_assembly_station) {
                commands.move_left = false;
                if (Decision_Left_ToAs) {
                    Decision_Left_ToAs = false;
                    ROS_INFO("[Robot] At_As");
                    state_ = State::At_As;
                }
            }
            if (getState().at_supply_conveyor) {
                if (Decision_Left_ToSc) {
                    Decision_Left_ToSc = false;
                    outputs.at_ec = false;
                    sendOuputs(outputs);
                    ROS_INFO("[Robot] Idle");
                    state_ = State::Idle;
                }
            }    
            
            break;
        case State::At_As:
            if (Decision_Part1_2 == false && Decision_Part2_2 == false && Decision_Part3_2 == false){
                at_as_begin = true;
                ROS_INFO("[Robot] Right");
                state_ = State::Right;
            }
            else if (Decision_Part1_2) {
                Decision_Part1_2 = false;
                ROS_INFO("[Robot] Assemble Part 1");
                state_ = State::Ass_Part1;
            }
            else if (Decision_Part2_2) {
                Decision_Part2_2 = false;
                ROS_INFO("[Robot] Assemble Part 2");
                state_ = State::Ass_Part2;
            }
            else if (Decision_Part3_2) {
                Decision_Part3_2 = false;
                ROS_INFO("[Robot] Assemble Part 3");
                state_ = State::Ass_Part3;
            }
            break;
        case State::Ass_Part1:
            commands.assemble_part1 = true;
            if (getState().part1_assembled) {
                commands.assemble_part1 = false;
                part1_exist = true;
                Decision_Right_ToSc = true;
                ++part_count;
                ROS_INFO("[Robot] Right");
                state_ = State::Right;
            }
            break;
        case State::Ass_Part2:
            commands.assemble_part2 = true;
            if (getState().part2_assembled) {
                commands.assemble_part2 = false;
                part2_exist = true;
                Decision_Right_ToSc = true;
                ++part_count;
                ROS_INFO("[Robot] Right");
                state_ = State::Right;
            }
            break;
        case State::Ass_Part3:
            commands.assemble_part3 = true;
            if (getState().part3_assembled) {
                commands.assemble_part3 = false;
                part3_exist = true;
                Decision_Right_ToSc = true;
                ++part_count;
                ROS_INFO("[Robot] Right");
                state_ = State::Right;
            }
            break;
        case State::Right:
            commands.move_right = true;
            if (at_as_begin && getState().at_supply_conveyor){
                commands.move_right = false;
                at_as_begin = false;
                ROS_INFO("[Robot] Idle");
                state_ = State::Idle;
            }
            else if (Decision_Right_ToSc && getState().at_supply_conveyor) {
                commands.move_right = false;
                Decision_Right_ToSc = false;
                if (part_count == 3) {
                    part_count = 0;
                    part1_exist = 0;
                    part2_exist = 0;
                    part3_exist = 0;
                    outputs.part_all_ok = true;
                    sendOuputs(outputs);
                    ROS_INFO("[Robot] Parts at AS == 3");
                    if (inputs.assembly_all_done) {
                        inputs.assembly_all_done = false;
                    }
                }
                ROS_INFO("[Robot] Idle");
                state_ = State::Idle;
            }
            else if (Decision_Right_ToEc && getState().at_evacuation_conveyor) {
                commands.move_right = false;
                Decision_Right_ToEc = false;
                outputs.at_ec = true;
                sendOuputs(outputs);
                ROS_INFO("[Robot] At EC");
                state_ = State::At_Ec;
            }
            break;
        case State::Decision_Evacuation:
            Decision_Right_ToEc = true;
            ROS_INFO("[Robot] Right");
            state_ = State::Right;
            break;
        case State::At_Ec:
            if (inputs.can_release) {
                inputs.can_release = false;
                ROS_INFO("[Robot] Release");
                state_ = State::Release;
            }
            break;
        case State::Release:
            commands.release = true;
            if (getState().part_released) {
                commands.release = false;
                outputs.released_to_ec = true;
                sendOuputs(outputs);
                Decision_Left_ToSc = true;
                ROS_INFO("[Robot] Left");
                state_ = State::Left;
            }
            break;
        }
        sendCommands(commands);
    }

private:
    enum class State { Idle, Grasp, Decision_1, Decision_2,
      Decision_Part1_1, Decision_Part2_1, Decision_Part3_1, Decision_Evacuation,
      Left, Right, Release, At_As, At_Ec,
      Ass_Part1, Ass_Part2, Ass_Part3 };

    State state_;
};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "robot");

    ros::NodeHandle node;

    ros::Rate loop_rate(50); // 50 Hz

    Robot robot(node);

    while (ros::ok()) {
        robot.process();

        ros::spinOnce();

        loop_rate.sleep();
    }
}
