#pragma once

#include <ros/ros.h>

#include <string>

//! \brief Base class for machines present in the assembly cell
//!
//! \tparam StateMsgT The type of the message containing the machine state
//! \tparam InputMsgT The type of the message containing the machine input signals
//! \tparam CommandMsgT The type of the message containing the machine commands
//! \tparam OutputMsgT The type of the message containing the machine output signals
template <typename StateMsgT, typename InputMsgT, typename CommandMsgT,
          typename OutputMsgT>
class MachineController {
public:
    //! \brief Construct a new Machine Controller given a node handle and a topic
    //!
    //! \param node The node controlling the machine
    //! \param topic The topic associated with the machine
    MachineController(ros::NodeHandle node, const std::string& topic)
        : node_(node) {
        command_publisher_ =
            node.advertise<CommandMsgT>(topic + "/command", 10);

        output_publisher_ = node.advertise<OutputMsgT>(topic + "/output", 10);

        state_subscriber_ = node.subscribe(
            topic + "/state", 10, &MachineController::stateCallback, this);

        input_subscriber_ = node.subscribe(
            topic + "/input", 10, &MachineController::inputCallback, this);
    }

    //! \brief Where the handling of the machine's i/o must be performed
    virtual void process() = 0;

protected:
    //! \brief Send a command message
    //!
    //! \param command_message The message to send
    void sendCommands(CommandMsgT command_message) {
        command_publisher_.publish(command_message);
    }

    //! \brief Send the output signals
    //!
    //! \param output_message The message to send
    void sendOuputs(OutputMsgT output_message) {
        output_publisher_.publish(output_message);
    }

    //! \brief Get the last received received state message
    //!
    //! \return const StateMsgT& Message with the machine state
    const StateMsgT& getState() const {
        return state_message_;
    }

    //! \brief Get the last received input signals
    //!
    //! \return InputMsgT& Message with the input signals
    InputMsgT& getInputs() {
        return input_message_;
    }

private:
    //! \brief Called when a new state message is available
    //!
    //! \param message The new state
    void stateCallback(const typename StateMsgT::ConstPtr& message) {
        state_message_ = *message;
    }

    //! \brief Called when new input signals are available
    //!
    //! \param message The new input signals
    void inputCallback(const typename InputMsgT::ConstPtr& message) {
        input_message_ = *message;
    }

    ros::NodeHandle node_;
    ros::Publisher command_publisher_;
    ros::Publisher output_publisher_;
    ros::Subscriber state_subscriber_;
    ros::Subscriber input_subscriber_;

    StateMsgT state_message_;
    InputMsgT input_message_;
};
