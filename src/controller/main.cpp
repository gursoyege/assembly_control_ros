#include <ros/ros.h>

#include <assembly_control_ros/assembly_station_input.h>
#include <assembly_control_ros/assembly_station_output.h>
#include <assembly_control_ros/evacuation_conveyor_input.h>
#include <assembly_control_ros/evacuation_conveyor_output.h>
#include <assembly_control_ros/robot_input.h>
#include <assembly_control_ros/robot_output.h>
#include <assembly_control_ros/supply_conveyor_input.h>
#include <assembly_control_ros/supply_conveyor_output.h>
#include <assembly_control_ros/camera_input.h>
#include <assembly_control_ros/camera_output.h>

#include <common/machine_controller.hpp>

class Controller {
public:
    Controller(ros::NodeHandle node) : node_(node), state_(State::Wait) {

        input_publisher_assembly_station_ = node.advertise<assembly_control_ros::assembly_station_input>("/assembly_station/input", 10);
        input_publisher_camera_ = node.advertise<assembly_control_ros::camera_input>("/camera/input", 10);
        input_publisher_robot_ = node.advertise<assembly_control_ros::robot_input>("/robot/input", 10);
        input_publisher_evacuation_conveyor_ =node.advertise<assembly_control_ros::evacuation_conveyor_input>("/evacuation_conveyor/input", 10);
        input_publisher_supply_conveyor_ = node.advertise<assembly_control_ros::supply_conveyor_input>("/supply_conveyor/input", 10);

        output_subscriber_assembly_station_ = node.subscribe("/assembly_sattion/output", 10, &Controller::assembly_station_outputCallback, this);
        output_subscriber_camera_ = node.subscribe("/camera/output", 10, &Controller::camera_outputCallback, this);
        output_subscriber_robot_ = node.subscribe("/robot/output", 10, &Controller::robot_outputCallback, this);
        output_subscriber_evacuation_conveyor_ = node.subscribe("/evacuation_conveyor/output", 10, &Controller::evacuation_conveyor_outputCallback, this);
        output_subscriber_supply_conveyor_ = node.subscribe("/supply_conveyor/output", 10, &Controller::supply_conveyor_outputCallback, this);
    }

    virtual void process() {
        switch (state_) {
        case State::Wait:
             if (robot_output_message_.part_all_ok) {
                robot_output_message_.part_all_ok = false;
                assembly_station_input_message_.check = true;
                input_publisher_assembly_station_.publish(assembly_station_input_message_);
                ROS_INFO("[Controller] publish robot to as");
            }
             if (robot_output_message_.at_ec) {
                robot_output_message_.at_ec = false;
                evacuation_conveyor_input_message_.can_stop = true;
                input_publisher_evacuation_conveyor_.publish(evacuation_conveyor_input_message_);
                evacuation_conveyor_input_message_.on = false;
                input_publisher_evacuation_conveyor_.publish(evacuation_conveyor_input_message_);
                ROS_INFO("[Controller] publish robot to ec");
            }
             if (robot_output_message_.released_to_ec) {
                robot_output_message_.released_to_ec = false;
                evacuation_conveyor_input_message_.on = true;
                input_publisher_evacuation_conveyor_.publish(evacuation_conveyor_input_message_);
                evacuation_conveyor_input_message_.can_stop = false;
                input_publisher_evacuation_conveyor_.publish(evacuation_conveyor_input_message_);
                robot_input_message_.can_release = false;
                input_publisher_robot_.publish(robot_input_message_);
                ROS_INFO("[Controller] publish robot to ec");
            }
             if (evacuation_conveyor_output_message_.stopped) {
                evacuation_conveyor_output_message_.stopped = false;
                robot_input_message_.can_release = true;
                input_publisher_robot_.publish(robot_input_message_);
                ROS_INFO("[Controller] publish ec to rob");
            }
             if (robot_output_message_.grasped_at_sc) {
                robot_output_message_.grasped_at_sc = false;
                supply_conveyor_input_message_.restart = true;
                input_publisher_supply_conveyor_.publish(supply_conveyor_input_message_);
                assembly_station_input_message_.check = false;
                input_publisher_assembly_station_.publish(assembly_station_input_message_);
                ROS_INFO("[Controller] publish robot to sc");
            }
            if (camera_output_message_.part_analyzed) {
                camera_output_message_.part_analyzed = false;
                robot_input_message_.cam_done = true;
                ROS_INFO("[Controller] publish camera to robot");
                input_publisher_robot_.publish(robot_input_message_);
            }
            if (camera_output_message_.part1 && camera_output_message_.part2 == false && camera_output_message_.part3 == false) {
                camera_output_message_.part1 = false;
                robot_input_message_.part1 = true;
                input_publisher_robot_.publish(robot_input_message_);
                robot_input_message_.part2 = false;
                input_publisher_robot_.publish(robot_input_message_);
                robot_input_message_.part3 = false;
                input_publisher_robot_.publish(robot_input_message_);    
                ROS_INFO("[Controller] publish camera to robot");            
            }
            if (camera_output_message_.part2 && camera_output_message_.part1 == false && camera_output_message_.part3 == false ) {
                camera_output_message_.part2 = false;
                robot_input_message_.part2 = true;
                input_publisher_robot_.publish(robot_input_message_);
                robot_input_message_.part1 = false;
                input_publisher_robot_.publish(robot_input_message_);
                robot_input_message_.part3 = false;
                input_publisher_robot_.publish(robot_input_message_); 
                ROS_INFO("[Controller] publish camera to robot");
            }
            if (camera_output_message_.part3 && camera_output_message_.part1 == false && camera_output_message_.part2 == false) {
                camera_output_message_.part3 = false;
                robot_input_message_.part3 = true;
                input_publisher_robot_.publish(robot_input_message_);
                robot_input_message_.part1 = false;
                input_publisher_robot_.publish(robot_input_message_);
                robot_input_message_.part2 = false;
                input_publisher_robot_.publish(robot_input_message_);
                ROS_INFO("[Controller] publish camera to robot");
            }

             if (supply_conveyor_output_message_.part_available) {
                supply_conveyor_output_message_.part_available = false;
                camera_input_message_.start_recognition = true;
                input_publisher_camera_.publish(camera_input_message_);
                ROS_INFO("[Controller] publish sc to camera");
            }
            state_ = State::ToWait;
            break;
        case State::ToWait:

            state_ = State::Wait;
            break;
        }
    }

protected:
    assembly_control_ros::assembly_station_output assembly_station_output_message_;
    assembly_control_ros::camera_output camera_output_message_;
    assembly_control_ros::robot_output robot_output_message_;
    assembly_control_ros::evacuation_conveyor_output evacuation_conveyor_output_message_;
    assembly_control_ros::supply_conveyor_output supply_conveyor_output_message_;

    assembly_control_ros::assembly_station_input assembly_station_input_message_;
    assembly_control_ros::camera_input camera_input_message_;
    assembly_control_ros::robot_input robot_input_message_;
    assembly_control_ros::evacuation_conveyor_input evacuation_conveyor_input_message_;
    assembly_control_ros::supply_conveyor_input supply_conveyor_input_message_;

private:
    ros::NodeHandle node_;

    enum class State { Wait,ToWait};
    State state_;

    void assembly_station_outputCallback(
        const assembly_control_ros::assembly_station_output::ConstPtr& message) { assembly_station_output_message_ = *message;
    }
    void evacuation_conveyor_outputCallback(
        const assembly_control_ros::evacuation_conveyor_output::ConstPtr& message) {
        evacuation_conveyor_output_message_ = *message;
    }
    void
    robot_outputCallback(const assembly_control_ros::robot_output::ConstPtr& message) {
        robot_output_message_ = *message;
    }
    void
    camera_outputCallback(const assembly_control_ros::camera_output::ConstPtr& message) {
        camera_output_message_ = *message;
    }
    void supply_conveyor_outputCallback(const assembly_control_ros::supply_conveyor_output::ConstPtr& message) {
        supply_conveyor_output_message_ = *message;

    }

    ros::Publisher input_publisher_assembly_station_;
    ros::Publisher input_publisher_camera_;
    ros::Publisher input_publisher_robot_;
    ros::Publisher input_publisher_supply_conveyor_;
    ros::Publisher input_publisher_evacuation_conveyor_;

    ros::Subscriber output_subscriber_assembly_station_;
    ros::Subscriber output_subscriber_camera_;
    ros::Subscriber output_subscriber_robot_;
    ros::Subscriber output_subscriber_supply_conveyor_;
    ros::Subscriber output_subscriber_evacuation_conveyor_;
};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "controller");

    ros::NodeHandle node_;

    ros::Rate loop_rate(50); 

    Controller master(node_);

    while (ros::ok()) {
        master.process();

        ros::spinOnce();

        loop_rate.sleep();
    }
}
