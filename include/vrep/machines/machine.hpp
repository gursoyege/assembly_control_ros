#pragma once

#include <ros/ros.h>

#include <vrep/simulator.h>

#include <string>
#include <functional>
#include <mutex>

template <typename StateMsgT, typename CommandMsgT> class Machine {
public:
    using prepare_message_function =
        std::function<void(StateMsgT&, const Simulator::State&)>;

    Machine(ros::NodeHandle node, Simulator& simulator,
            const std::string& topic, prepare_message_function prepare_message)
        : node_(node),
          simulator_(simulator),
          prepare_message_(prepare_message) {

        pub_ = node.advertise<StateMsgT>(topic + "/state", 10);
        sub_ = node.subscribe(topic + "/command", 10, &Machine::callback, this);
    }

    CommandMsgT process() {
        auto sim_state = simulator_.getState();
        CommandMsgT command;
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            command = command_;
        }

        prepare_message_(state_, sim_state);

        pub_.publish(state_);

        return command;
    }

    auto operator()() {
        return process();
    }

    void callback(const typename CommandMsgT::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(command_mutex_);
        command_ = *msg;
    }

private:
    ros::NodeHandle node_;
    Simulator& simulator_;

    ros::Publisher pub_;
    ros::Subscriber sub_;

    StateMsgT state_;
    CommandMsgT command_;
    std::mutex command_mutex_;

    prepare_message_function prepare_message_;
};
