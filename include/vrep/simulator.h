/**
 * @file simulator.h
 * @brief Implement a Simulator class to interact with V-REP as well as the
 * Signal class
 * @author Benjamin Navarro
 * @version 2.0.0
 * @date 2019-07-25
 */

#pragma once

#include <thread>
#include <mutex>
#include <chrono>
#include <condition_variable>
#include <atomic>

/**
 * @brief Implementation of a synchronization signal
 */
class Signal {
public:
    Signal();

    ~Signal() = default;

    /**
     * @brief Wait for the signal to come (blocking call)
     */
    void wait();

    /**
     * @brief Wait for the signal to come (blocking call) for a certain amount
     * of time
     * @param ms Number of milliseconds to wait before returning
     * @return true is the signal has arrived during the timeout period, false
     * otherwise
     */

    bool wait_for(int ms);

    /**
     * @brief Notify all the waiters that the signal has arrived
     */
    void notify();

protected:
    std::mutex m_;
    std::condition_variable cv_;
    std::atomic<bool> signaled_;
};

/**
 * @brief Interface with the V-REP simulator
 */
class Simulator {
public:
    /**
     * @brief Commands sent to V-REP
     */
    struct Commands {
        Commands() noexcept;

        bool supply_conveyor_on;
        bool evacuation_conveyor_on;
        bool start_recognition;
        bool move_right;
        bool move_left;
        bool grasp;
        bool release;
        bool assemble_part1;
        bool assemble_part2;
        bool assemble_part3;
        bool check_assembly;
    };

    /**
     * @brief Signals read from V-REP
     */
    struct State {
        State() noexcept;

        bool optical_barrier;
        bool grasped;
        bool released;
        bool at_supply_conveyor;
        bool at_evacuation_conveyor;
        bool at_assembly_station;
        bool evacuation_conveyor_stopped;
        bool recognition_complete;
        bool part1_detected;
        bool part2_detected;
        bool part3_detected;
        bool part1_assembled;
        bool part2_assembled;
        bool part3_assembled;
        bool assembly_valid;
        bool assembly_evacuated;
    };

    Simulator();

    Simulator(int cycle_ms, const std::string& ip_address);

    ~Simulator();

    /**
     * @brief Start the simulation in V-REP and the communication thread
     *
     * @param cycle_ms Communication thread cycle time (milliseconds)
     * @param ip_address IP address of the V-REP remote API server
     *
     * @return True if successfully started, false otherwise
     */
    bool start(int cycle_ms, const std::string& ip_address);

    /**
     * @brief Stop the simulation in V-REP and the communication thread
     */
    void stop();

    /**
     * @brief Synchronize with the communication thread
     */
    void sync();

    State getState() const;
    void setCommands(const Commands& commands);

private:
    /**
     * @brief Get object handles from V-REP
     *
     * @return True if successful, false otherwise
     */
    bool get_Handles();
    /**
     * @brief Start data streaming from V-REP
     *
     * @return True if successful, false otherwise
     */
    bool start_Streaming();
    /**
     * @brief Get the current time since epoch
     *
     * @return Time in seconds (us precision)
     */
    double get_Current_Time();

    /**
     * @brief Communication thread
     *
     * @param cycle_ms Cycle time (milliseconds)
     */
    void process(int cycle_ms);

    /**
     * @brief Current robot position
     */
    enum Position {
        PosAssembly = 1,
        PosAppro = 2,
        PosEvac = 3,
    };
    Position prev_position_;

    std::atomic<Commands> commands_;

    std::atomic<State> signals_;

    int prev_gripper_state_;
    int prev_optical_barrier_state_;
    int prev_evac_conveyor_state_;

    double last_created_object_time_;
    int last_created_object_type_;

    std::thread thread_;
    bool run_;
    Signal sync_signal_;

    int client_id_;
    int appro_prox_sensor_handle_;
};
