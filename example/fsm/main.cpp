#include <yandy/modules/FSM.hpp>

using namespace boost::msm::front::puml;
using enum YandyControlCmd;

int main()
{
    yandy::core::init_logging();
    yandy::modules::YandyArmFSM fsm;

    fsm.processCmd(CMD_SWITCH_ENABLE);
    fsm.logState();
    fsm.processCmd(CMD_SWITCH_ENABLE);
    fsm.logState();
    fsm.processCmd(CMD_ERROR);
    fsm.logState();
    fsm.processCmd(CMD_RESET);
    fsm.logState();
    fsm.processCmd(CMD_SWITCH_ENABLE);
    fsm.logState();
    fsm.processCmd(CMD_RESET);
    fsm.logState();
    fsm.processCmd(CMD_SWITCH_FETCH);
    fsm.logState();
    fsm.processCmd(CMD_SWITCH_FETCH);
    fsm.logState();
}
