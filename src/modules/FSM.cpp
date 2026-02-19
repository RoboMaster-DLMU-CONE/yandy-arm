#include <yandy/modules/FSM.hpp>
#include <boost/msm/back/metafunctions.hpp>

using enum YandyControlCmd;
using boost::msm::front::puml::Event;
using boost::msm::front::puml::by_name;
using boost::msm::front::puml::State;

namespace yandy::modules
{
    // Compile-time state ID lookup — auto-updates when PUML changes
    namespace
    {
        using stt = detail::YandyArmFSMBackend::stt;
        template <auto Hash>
        constexpr int id_of = msm::back::get_state_id<stt, State<Hash>>::value;
    }

    YandyArmFSM::YandyArmFSM()
    {
        m_fsm.start();
        m_logger = m_fsm.m_logger;
        sync_state();
    }

    void YandyArmFSM::processCmd(const YandyControlCmd cmd)
    {
        switch (cmd)
        {
        case CMD_ERROR:
            m_fsm.process_event(Event<by_name("CMD_ERROR")>{});
            break;
        case CMD_SWITCH_ENABLE:
            m_fsm.process_event(Event<by_name("CMD_SWITCH_ENABLE")>{});
            break;
        case CMD_RESET:
            m_fsm.process_event(Event<by_name("CMD_RESET")>{});
            break;
        case CMD_SWITCH_FETCH:
            m_fsm.process_event(Event<by_name("CMD_SWITCH_FETCH")>{});
            break;
        case CMD_SWITCH_STORE:
            m_fsm.process_event(Event<by_name("CMD_SWITCH_STORE")>{});
            break;
        case CMD_SWITCH_GRIP:
            m_fsm.process_event(Event<by_name("CMD_SWITCH_GRIP")>{});
            break;
        case CMD_TOGGLE_HELD:
            m_fsm.process_event(Event<by_name("CMD_TOGGLE_HELD")>{});
            break;
        case CMD_INC_STORE:
            m_fsm.process_event(Event<by_name("CMD_INC_STORE")>{});
            break;
        case CMD_DEC_STORE:
            m_fsm.process_event(Event<by_name("CMD_DEC_STORE")>{});
            break;
        default:
            break;
        }
        sync_state();
    }

    void YandyArmFSM::logState() const
    {
        m_logger->info("Current State:{}", format_as(m_current_state.load(std::memory_order_acquire)));
    }

    YandyState YandyArmFSM::getState() const
    {
        return m_current_state.load(std::memory_order_acquire);
    }

    // PUML produces 2 orthogonal regions:
    //   region[0]: Operational / ErrorMode  (outer layer)
    //   region[1]: Disabled / ManualControl / FetchingMode / StorageMode (inner layer)
    // The IDs below are derived at compile time from the transition table,
    // so they stay correct even if the PUML is modified.
    void YandyArmFSM::sync_state()
    {
        const int* cs = m_fsm.current_state();

        if (cs[0] == id_of<by_name("ErrorMode")>)
        {
            m_current_state.store(YandyState::Error, std::memory_order_release);
            return;
        }

        const int inner = cs[1];
        if (inner == id_of<by_name("Disabled")>)
            m_current_state.store(YandyState::Disabled, std::memory_order_release);
        else if (inner == id_of<by_name("ManualControl")>)
            m_current_state.store(YandyState::Manual, std::memory_order_release);
        else if (inner == id_of<by_name("FetchingMode")>)
            m_current_state.store(YandyState::Fetching, std::memory_order_release);
        else if (inner == id_of<by_name("StorageMode")>)
            m_current_state.store(YandyState::Store, std::memory_order_release);
        else
            m_current_state.store(YandyState::Unknown, std::memory_order_release);
    }
}
