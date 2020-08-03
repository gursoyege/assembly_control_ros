/**
 * @file simulator.cpp
 * @brief Simulator class implementation
 * @author Benjamin Navarro
 * @version 1.0.0
 * @date 2015-10-12
 */

#include <vrep/simulator.h>

#include <iostream>
#include <chrono>
#include <thread>

#include <cstring>
#include <chrono>
#include <sys/time.h>

extern "C" {
#include "extApi.h"
}

constexpr bool LAZY_MODE = false;

Signal::Signal() : signaled_(false) {
}

void Signal::wait() {
    std::unique_lock<std::mutex> lock(m_);
    cv_.wait(lock, [this]() { return signaled_.load(); });
    signaled_.store(false);
}

bool Signal::wait_for(int ms) {
    std::unique_lock<std::mutex> lock(m_);
    if (cv_.wait_for(lock, std::chrono::milliseconds(ms),
                     [this]() { return signaled_.load(); })) {
        signaled_.store(false);
        return true;
    } else {
        return false;
    }
}

void Signal::notify() {
    signaled_.store(true);
    cv_.notify_one();
}

Simulator::Commands::Commands() noexcept
    : supply_conveyor_on(true),
      evacuation_conveyor_on(true),
      start_recognition(false),
      move_right(false),
      move_left(false),
      grasp(false),
      release(false),
      assemble_part1(false),
      assemble_part2(false),
      assemble_part3(false),
      check_assembly(false) {
}

Simulator::State::State() noexcept
    : optical_barrier(false),
      grasped(false),
      released(false),
      at_supply_conveyor(false),
      at_evacuation_conveyor(false),
      at_assembly_station(false),
      evacuation_conveyor_stopped(false),
      recognition_complete(false),
      part1_detected(false),
      part2_detected(false),
      part3_detected(false),
      part1_assembled(false),
      part2_assembled(false),
      part3_assembled(false),
      assembly_valid(false),
      assembly_evacuated(false) {
}

Simulator::Simulator()
    : client_id_(-1),
      run_(false),
      prev_position_(PosAssembly),
      prev_gripper_state_(0),
      prev_optical_barrier_state_(0),
      prev_evac_conveyor_state_(1),
      last_created_object_time_(0),
      last_created_object_type_(0) {
}

Simulator::Simulator(int cycle_ms, const std::string& ip_address)
    : Simulator() {
    start(cycle_ms, ip_address);
}

Simulator::~Simulator() {
    stop();
}

bool Simulator::start(int cycle_ms, const std::string& ip_address) {
    client_id_ =
        simxStart((simxChar*)ip_address.c_str(), 19997, true, true, 2000, 5);
    if (client_id_ != -1) {
        std::cout << "Connected to V-REP" << std::endl;

        if (not get_Handles()) {
            std::cout << "Failed to get object handles" << std::endl;
            return false;
        }

        if (not start_Streaming()) {
            std::cout << "Starting streaming values failed" << std::endl;
            return false;
        }

        simxStartSimulation(client_id_, simx_opmode_oneshot_wait);

        std::cout << "Simulation started" << std::endl;

        std::this_thread::sleep_for(std::chrono::seconds(1));

        run_ = true;
        thread_ = std::thread(&Simulator::process, this, cycle_ms);
    } else {
        simxFinish(client_id_);
        throw std::runtime_error(
            "Simulator::Simulator: Cannot start the communication with V-REP. "
            "Make sure it is open and that the following IP address is the "
            "correct one: " +
            ip_address);

        return false;
    }

    return true;
}

void Simulator::stop() {
    if (run_) {
        run_ = false;
        thread_.join();
    }

    if (client_id_ >= 0) {
        simxStopSimulation(client_id_, simx_opmode_oneshot_wait);
        simxFinish(client_id_);
        std::cout << "Simulation ended" << std::endl;
    }
}

void Simulator::sync() {
    sync_signal_.wait();
}

Simulator::State Simulator::getState() const {
    return signals_.load();
}

void Simulator::setCommands(const Commands& commands) {
    commands_.store(commands);
}

bool Simulator::get_Handles() {
    int ret_code = simxGetObjectHandle(client_id_, "appro_proximity_sensor#",
                                       &appro_prox_sensor_handle_,
                                       simx_opmode_oneshot_wait);
    if (ret_code != simx_return_ok) {
        std::cout << "can't get appro_proximity_sensor handle (error "
                  << ret_code << ")" << std::endl;
        return false;
    }

    return true;
}

bool Simulator::start_Streaming() {
    simxInt tmp;
    bool all_ok = true;

    all_ok &= ((simxGetIntegerSignal(client_id_, "optical_barrier_state", &tmp,
                                     simx_opmode_streaming) &
                0xFE) == 0);
    all_ok &= ((simxGetIntegerSignal(client_id_, "gripper_closed", &tmp,
                                     simx_opmode_streaming) &
                0xFE) == 0);
    all_ok &= ((simxGetIntegerSignal(client_id_, "current_position", &tmp,
                                     simx_opmode_streaming) &
                0xFE) == 0);
    all_ok &= ((simxGetIntegerSignal(client_id_, "evac_conveyor_stopped", &tmp,
                                     simx_opmode_streaming) &
                0xFE) == 0);

    all_ok &= ((simxGetIntegerSignal(client_id_, "end_identification", &tmp,
                                     simx_opmode_streaming) &
                0xFE) == 0);
    all_ok &= ((simxGetIntegerSignal(client_id_, "box_type", &tmp,
                                     simx_opmode_streaming) &
                0xFE) == 0);
    all_ok &= ((simxGetIntegerSignal(client_id_, "end_operation", &tmp,
                                     simx_opmode_streaming) &
                0xFE) == 0);
    all_ok &= ((simxGetIntegerSignal(client_id_, "assembly_ok", &tmp,
                                     simx_opmode_streaming) &
                0xFE) == 0);
    all_ok &= ((simxGetIntegerSignal(client_id_, "assembly_evacuated", &tmp,
                                     simx_opmode_streaming) &
                0xFE) == 0);

    return all_ok;
}

double Simulator::get_Current_Time() {
    using namespace std::chrono;
    auto now = high_resolution_clock::now();
    return duration_cast<milliseconds>(now.time_since_epoch()).count() * 1e-3;
}

void Simulator::process(int cycle_ms) {
    using cycle = std::chrono::duration<int, std::chrono::milliseconds::period>;

    simxInt optical_barrier_state, gripper_state, position,
        evac_conveyor_stopped;
    simxInt end_identification, box_type, end_operation, assembly_ok,
        assembly_evacuated;

    simxGetIntegerSignal(client_id_, "optical_barrier_state",
                         &optical_barrier_state, simx_opmode_oneshot_wait);
    simxGetIntegerSignal(client_id_, "gripper_closed", &gripper_state,
                         simx_opmode_oneshot_wait);
    simxGetIntegerSignal(client_id_, "current_position", &position,
                         simx_opmode_oneshot_wait);
    simxGetIntegerSignal(client_id_, "evac_conveyor_stopped",
                         &evac_conveyor_stopped, simx_opmode_oneshot_wait);

    simxGetIntegerSignal(client_id_, "end_identification", &end_identification,
                         simx_opmode_oneshot_wait);
    simxGetIntegerSignal(client_id_, "box_type", &box_type,
                         simx_opmode_oneshot_wait);
    simxGetIntegerSignal(client_id_, "end_operation", &end_operation,
                         simx_opmode_oneshot_wait);
    simxGetIntegerSignal(client_id_, "assembly_ok", &assembly_ok,
                         simx_opmode_oneshot_wait);
    simxGetIntegerSignal(client_id_, "assembly_evacuated", &assembly_evacuated,
                         simx_opmode_oneshot_wait);

    std::cout << "Simulator communication thread started. Cycle time = "
              << cycle_ms << "ms" << std::endl;

    while (run_) {
        auto start_time = std::chrono::steady_clock::now();
        auto end_time = start_time + cycle(cycle_ms);

        auto signals = signals_.load();
        auto commands = commands_.load();

        /***********************		Signals ************************/
        simxGetIntegerSignal(client_id_, "optical_barrier_state",
                             &optical_barrier_state, simx_opmode_streaming);
        simxGetIntegerSignal(client_id_, "gripper_closed", &gripper_state,
                             simx_opmode_streaming);
        simxGetIntegerSignal(client_id_, "current_position", &position,
                             simx_opmode_streaming);
        simxGetIntegerSignal(client_id_, "evac_conveyor_stopped",
                             &evac_conveyor_stopped, simx_opmode_streaming);

        simxGetIntegerSignal(client_id_, "end_identification",
                             &end_identification, simx_opmode_streaming);
        simxGetIntegerSignal(client_id_, "box_type", &box_type,
                             simx_opmode_streaming);
        simxGetIntegerSignal(client_id_, "end_operation", &end_operation,
                             simx_opmode_streaming);
        simxGetIntegerSignal(client_id_, "assembly_ok", &assembly_ok,
                             simx_opmode_streaming);
        simxGetIntegerSignal(client_id_, "assembly_evacuated",
                             &assembly_evacuated, simx_opmode_streaming);

        if (optical_barrier_state != prev_optical_barrier_state_) {
            prev_optical_barrier_state_ = optical_barrier_state;

            signals.optical_barrier = optical_barrier_state;
        }

        signals.grasped = gripper_state;
        signals.released = not gripper_state;

        if (position != prev_position_) {
            signals.at_assembly_station = false;
            signals.at_supply_conveyor = false;
            signals.at_evacuation_conveyor = false;

            switch (position) {
            case PosAssembly:
                signals.at_assembly_station = true;
                break;
            case PosAppro:
                signals.at_supply_conveyor = true;
                break;
            case PosEvac:
                signals.at_evacuation_conveyor = true;
                break;
            }

            prev_position_ = Position(position);
        }

        if (evac_conveyor_stopped != prev_evac_conveyor_state_) {
            prev_evac_conveyor_state_ = evac_conveyor_stopped;

            signals.evacuation_conveyor_stopped = evac_conveyor_stopped;
        }

        signals.recognition_complete = end_identification;
        signals.part1_detected = (box_type == 1);
        signals.part2_detected = (box_type == 2);
        signals.part3_detected = (box_type == 3);
        signals.part1_assembled = (end_operation == 1);
        signals.part2_assembled = (end_operation == 2);
        signals.part3_assembled = (end_operation == 3);
        signals.assembly_valid = assembly_ok;
        signals.assembly_evacuated = assembly_evacuated;

        /***********************        Commands		***********************/
        static bool test_t1 = true;
        if (commands.supply_conveyor_on) {
            simxSetIntegerSignal(client_id_, "appro_conveyor_command", 1,
                                 simx_opmode_oneshot);
            // Add new boxes to the conveyor
            if (test_t1) {
                last_created_object_time_ = get_Current_Time();
                test_t1 = false;
            }
            if ((get_Current_Time() - last_created_object_time_) > 2.) {
                last_created_object_time_ = get_Current_Time();

                if (LAZY_MODE) {
                    static int type = 0;
                    ++type;
                    if (type > 3)
                        type = 1;
                    simxSetIntegerSignal(client_id_, "add_object", type,
                                         simx_opmode_oneshot);

                } else {
                    int object_to_add = rand() % 100;
                    int type;

                    if (last_created_object_type_ == 0) {
                        if (object_to_add < 33)
                            type = 1;
                        else if (object_to_add < 66)
                            type = 2;
                        else
                            type = 3;
                    } else {
                        if (last_created_object_type_ == 1) {
                            if (object_to_add < 60)
                                type = 2;
                            else
                                type = 3;
                        } else if (last_created_object_type_ == 2) {
                            if (object_to_add < 40)
                                type = 1;
                            else
                                type = 3;
                        } else {
                            if (object_to_add < 60)
                                type = 1;
                            else
                                type = 2;
                        }
                    }

                    last_created_object_type_ = type;
                    simxSetIntegerSignal(client_id_, "add_object", type,
                                         simx_opmode_oneshot);
                }
            }
        } else {
            simxSetIntegerSignal(client_id_, "appro_conveyor_command", 0,
                                 simx_opmode_oneshot);
            test_t1 = true;
        }

        if (commands.evacuation_conveyor_on)
            simxSetIntegerSignal(client_id_, "evac_conveyor_command", 1,
                                 simx_opmode_oneshot);
        else
            simxSetIntegerSignal(client_id_, "evac_conveyor_command", 0,
                                 simx_opmode_oneshot);

        if (commands.start_recognition)
            simxSetIntegerSignal(client_id_, "reccam", 1, simx_opmode_oneshot);
        else
            simxSetIntegerSignal(client_id_, "reccam", 0, simx_opmode_oneshot);

        if (commands.move_right)
            simxSetIntegerSignal(client_id_, "go_right", 1,
                                 simx_opmode_oneshot);
        else
            simxSetIntegerSignal(client_id_, "go_right", 0,
                                 simx_opmode_oneshot);

        if (commands.move_left)
            simxSetIntegerSignal(client_id_, "go_left", 1, simx_opmode_oneshot);
        else
            simxSetIntegerSignal(client_id_, "go_left", 0, simx_opmode_oneshot);

        if (commands.grasp)
            simxSetIntegerSignal(client_id_, "take", 1, simx_opmode_oneshot);
        else
            simxSetIntegerSignal(client_id_, "take", 0, simx_opmode_oneshot);

        if (commands.release)
            simxSetIntegerSignal(client_id_, "put_down", 1,
                                 simx_opmode_oneshot);
        else
            simxSetIntegerSignal(client_id_, "put_down", 0,
                                 simx_opmode_oneshot);

        if (commands.assemble_part1)
            simxSetIntegerSignal(client_id_, "OP1", 1, simx_opmode_oneshot);
        else
            simxSetIntegerSignal(client_id_, "OP1", 0, simx_opmode_oneshot);

        if (commands.assemble_part2)
            simxSetIntegerSignal(client_id_, "OP2", 1, simx_opmode_oneshot);
        else
            simxSetIntegerSignal(client_id_, "OP2", 0, simx_opmode_oneshot);

        if (commands.assemble_part3)
            simxSetIntegerSignal(client_id_, "OP3", 1, simx_opmode_oneshot);
        else
            simxSetIntegerSignal(client_id_, "OP3", 0, simx_opmode_oneshot);

        if (commands.check_assembly)
            simxSetIntegerSignal(client_id_, "verif", 1, simx_opmode_oneshot);
        else
            simxSetIntegerSignal(client_id_, "verif", 0, simx_opmode_oneshot);

        signals_.store(signals);

        sync_signal_.notify();

        std::this_thread::sleep_until(end_time);
    }
}
