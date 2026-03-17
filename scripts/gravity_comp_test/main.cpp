#include <memory>
#include <yandy/modules/ArmHW.hpp>
#include <yandy/modules/DynamicsSolver.hpp>
#include <yandy/core/Logger.hpp>
#include <spdlog/spdlog.h>
#include <csignal>
#include <thread>
#include <atomic>
#include <chrono>
#include <iostream>

using namespace yandy;
using namespace std::chrono_literals;

std::atomic<bool> g_running{true};

void signal_handler(int signum)
{
    g_running = false;
}

int main()
{
    // Register signal handler for clean shutdown
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    auto logger = core::create_logger("GravityTest", spdlog::level::info);
    logger->info("Starting Gravity Compensation Test (Drag Mode)...");
    logger->warn("================================================================");
    logger->warn("IMPORTANT: Ensure Kp=0 and Kd=0.5 (damping) in config/joint.toml");
    logger->warn("If Kp is high, the robot will hold position and fight you!");
    logger->warn("================================================================");

    // Initialize Hardware
    logger->info("Initializing ArmHW...");
    auto arm_hw = std::make_unique<modules::ArmHW>();

    // Initialize Dynamics Solver
    logger->info("Initializing DynamicsSolver...");
    auto solver = std::make_unique<modules::DynamicsSolver>();

    // Enable Motors
    logger->info("Enabling motors...");
    arm_hw->enable();

    common::JointState current_state;
    common::JointCommand cmd;
    
    // Initialize command
    cmd.q_des.setZero();
    cmd.v_des.setZero();
    cmd.kp.setZero();      // Hope ArmHW uses this (currently might be ignored)
    cmd.kd.fill(0.5);      // Small damping
    cmd.tau_ff.setZero();

    logger->info("Entering control loop. Press Ctrl+C to stop.");

    auto next_time = std::chrono::steady_clock::now();
    const auto period = 4ms; // 250Hz

    while (g_running)
    {
        // 1. Read State
        arm_hw->read(current_state);

        // 2. Update Kinematics
        solver->updateKinematics(current_state.q, current_state.v);

        // 3. Compute Gravity Compensation (RNEA with 0 acceleration)
        // Note: computeGravity() is a wrapper for computeRNEA(0, 0)
        common::VectorJ tau_g = solver->computeGravity();

        // 4. Set Command
        // We set q_des to current_q to minimize position error term if Kp > 0
        cmd.q_des = current_state.q;
        cmd.v_des.setZero(); // We want it to stop if we let go
        cmd.tau_ff = tau_g;  // Feedforward gravity torque

        // 5. Write Command
        arm_hw->write(cmd);

        // 6. Timing
        next_time += period;
        std::this_thread::sleep_until(next_time);
    }

    logger->info("Stopping...");
    arm_hw->disable();
    logger->info("Done.");

    return 0;
}
