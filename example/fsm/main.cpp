#include <yandy/modules/FSM.hpp>

using namespace boost::msm::front::puml;

int main()
{
    yandy::core::init_logging();
    yandy::modules::YandyArmFSM fsm;
    fsm.start();
    fsm.process_event(Event<by_name("CMD_SWITCH_ENABLE")>());
    fsm.process_event(Event<by_name("CMD_SWITCH_ENABLE")>());
    fsm.process_event(Event<by_name("CMD_ERROR")>());
    fsm.process_event(Event<by_name("CMD_RESET")>());
    fsm.process_event(Event<by_name("CMD_SWITCH_ENABLE")>());

    fsm.process_event(Event<by_name("CMD_RESET")>());
}
