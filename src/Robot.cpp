#include <yandy/Robot.hpp>

yandy::Robot::Robot()
{
    m_input.setCommandCb([this](const YandyControlCmd cmd)
    {
        m_fsm.processCmd(cmd);
    });
    m_logger = core::create_logger("YandyRobot", spdlog::level::info);
}

yandy::Robot::~Robot()
{
    stop();
    // join the other thread
}

void yandy::Robot::stop()
{
    m_running.store(false, std::memory_order_release);
    while (m_running.load(std::memory_order_acquire) == false)
    {
        std::this_thread::yield(); // waiting for the start loop to the end
    }
}

void yandy::Robot::start()
{
    m_logger->info("Starting Robot...");
    while (m_running.load(std::memory_order_relaxed))
    {
    }
    m_running.store(true, std::memory_order_release); // ready for start
}
