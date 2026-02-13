#ifndef YANDY_ARM_FSM_HPP
#define YANDY_ARM_FSM_HPP

#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/front/state_machine_def.hpp>
#include <yandy/core/Logger.hpp>

namespace yandy::modules
{
    namespace msm = boost::msm;
    namespace mpl = boost::mpl;

    struct EvMove
    {
        std::array<float, 6> pos_arr{};

        EvMove(const std::array<float, 6>& pos_array) : pos_arr(pos_array)
        {
        };
    };

    struct EvSwitchEnable
    {
    };

    struct EvReset
    {
    };

    struct EvFetch
    {
    };

    struct EvSwitchStore
    {
    };

    struct EvSwitchGrip
    {
    };

    namespace detail
    {
        class YandyArmFSMDef : public msm::front::state_machine_def<YandyArmFSMDef>
        {
        public:
            YandyArmFSMDef()
            {
                m_logger = core::create_logger("YandyArmFSM", spdlog::level::info);
            }

            struct Idle : public msm::front::state<>
            {
                template <class Event, class FSM>
                void on_entry(Event const&, FSM& fsm) { fsm.m_logger->info("(Idle)"); }
            };

            typedef Idle initial_state;


            struct Moving : public msm::front::state<>
            {
                template <class Event, class FSM>
                void on_entry(Event const&, FSM& fsm) { fsm.m_logger->info("[状态] 机械臂开始运动 (Moving)"); }
            };

            void send_move_cmd(EvMove const& e)
            {
            }

            struct transition_table : mpl::vector<
                    a_row<Idle, EvMove, Moving, &YandyArmFSMDef::send_move_cmd>
                >
            {
            };

        private:
            bool mineral_attached = false;
            std::shared_ptr<spdlog::logger> m_logger;
        };
    }

    typedef msm::back::state_machine<detail::YandyArmFSMDef> YandyArmFSM;
}


#endif //YANDY_ARM_FSM_HPP
