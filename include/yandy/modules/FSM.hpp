#ifndef YANDY_ARM_FSM_HPP
#define YANDY_ARM_FSM_HPP

#include <boost/msm/front/state_machine_def.hpp>
#include <boost/msm/front/puml/puml.hpp>
#include <boost/msm/back11/state_machine.hpp>

#include <yandy/core/Logger.hpp>
#include <fsm_puml.h>
#include <iostream>

#include "yandy/common/Types.hpp"

namespace boost::msm::front::puml
{
    template <>
    struct Action<by_name("enable_output")>
    {
        template <class EVT, class FSM, class S, class T>
        void operator()(EVT const&, FSM& fsm, S&, T&)
        {
            fsm.m_logger->info("系统已启用，进入手动控制模式");
        }
    };

    template <>
    struct Action<by_name("disable_output")>
    {
        template <class EVT, class FSM, class S, class T>
        void operator()(EVT const&, FSM& fsm, S&, T&)
        {
            fsm.m_logger->info("系统已禁用");
        }
    };

    template <>
    struct Action<by_name("emergency_stop")>
    {
        template <class EVT, class FSM, class S, class T>
        void operator()(EVT const&, FSM& fsm, S&, T&)
        {
            fsm.m_logger->info("!!! 急停触发 !!!");
        }
    };

    template <>
    struct Action<by_name("clear_error")>
    {
        template <class EVT, class FSM, class S, class T>
        void operator()(EVT const&, FSM& fsm, S&, T&)
        {
            fsm.m_logger->info("错误已清除，系统待机");
        }
    };

    // --- Fetch 流程 ---
    template <>
    struct Action<by_name("enter_fetch")>
    {
        template <class EVT, class FSM, class S, class T>
        void operator()(EVT const&, FSM& fsm, S&, T&)
        {
            fsm.m_logger->info("启动视觉抓取流程...");
            fsm.m_logger->info("[提示] 请操作摇杆微调位置，再次发送指令以抓取。");
        }
    };

    template <>
    struct Action<by_name("leave_fetch_mode")>
    {
        template <class EVT, class FSM, class S, class T>
        void operator()(EVT const&, FSM& fsm, S&, T&)
        {
            // TODO: replace with fsm.check_grip_sensor()
            bool success = true;
            if (success)
            {
                fsm.mineral_attached = true;
                fsm.m_logger->info("抓取成功！已返回原位");
            }
            else
            {
                fsm.mineral_attached = false;
                fsm.m_logger->warn("传感器未检测到物体！抓取失败，已返回。");
            }
        }
    };

    // --- Store 流程 ---

    template <>
    struct Action<by_name("enter_store_mode")>
    {
        template <class EVT, class FSM, class S, class T>
        void operator()(EVT const&, FSM& fsm, S&, T&)
        {
            if (fsm.mineral_attached)
            {
                fsm.m_logger->info("准备存矿...");
                fsm.m_logger->info("[提示] 请微调位置，再次发送指令以放置。");
            }
            else
            {
                fsm.m_logger->info("准备取矿...");
                fsm.m_logger->info("[提示] 请微调位置，再次发送指令以抓取。");
            }
        }
    };

    template <>
    struct Action<by_name("store_finish")>
    {
        template <class EVT, class FSM, class S, class T>
        void operator()(EVT const&, FSM& fsm, S&, T&)
        {
            // 根据进入状态前的 mineral_attached 判断是存还是取
            if (fsm.mineral_attached)
            {
                // 存矿动作
                // fsm.hardware_grip(true); // 张开扔掉
                fsm.mineral_attached = false;
                ++fsm.stored_count;
                fsm.m_logger->info("存矿完成");
            }
            else
            {
                // 取矿动作
                // fsm.hardware_grip(false); // 闭合抓取
                fsm.mineral_attached = true;
                --fsm.stored_count;
                fsm.m_logger->info("取矿完成");
            }
            // fsm.hardware_move("HOME_POS");
            // 这里也可以顺便清除累积误差
        }
    };

    template <>
    struct Action<by_name("log_store_conflict")>
    {
        template <class EVT, class FSM, class S, class T>
        void operator()(EVT const&, FSM& fsm, S&, T&)
        {
            fsm.m_logger->error("逻辑冲突！手持状态与库存状态不匹配，无法执行。");
        }
    };

    // --- 手动与调试 ---
    template <>
    struct Action<by_name("clear_accumulators")>
    {
        template <class EVT, class FSM, class S, class T>
        void operator()(EVT const&, FSM& fsm, S&, T&)
        {
            fsm.m_logger->info("清除位置累积误差");
        }
    };

    template <>
    struct Action<by_name("toggle_gripper")>
    {
        template <class EVT, class FSM, class S, class T>
        void operator()(EVT const&, FSM& fsm, S&, T&)
        {
            // 简单模拟切换
            static bool open = false;
            open = !open;
            // fsm.hardware_grip(open);
        }
    };

    template <>
    struct Action<by_name("toggle_held_flag")>
    {
        template <class EVT, class FSM, class S, class T>
        void operator()(EVT const&, FSM& fsm, S&, T&)
        {
            fsm.mineral_attached = !fsm.mineral_attached;
            fsm.m_logger->info("强制切换持有状态为 {}", fsm.mineral_attached);
        }
    };

    template <>
    struct Action<by_name("inc_storage_count")>
    {
        template <class EVT, class FSM, class S, class T>
        void operator()(EVT const&, FSM& fsm, S&, T&)
        {
            if (fsm.stored_count < 2)
            {
                ++fsm.stored_count;
                fsm.m_logger->info("库存+1 当前 {}", fsm.stored_count);
            }
        }
    };

    template <>
    struct Action<by_name("dec_storage_count")>
    {
        template <class EVT, class FSM, class S, class T>
        void operator()(EVT const&, FSM& fsm, S&, T&)
        {
            if (fsm.stored_count > 0)
            {
                --fsm.stored_count;
                fsm.m_logger->info("库存-1 当前 {}", fsm.stored_count);
            }
        }
    };

    // 检查是否有矿
    template <>
    struct Guard<by_name("has_mineral")>
    {
        template <class EVT, class FSM, class S, class T>
        bool operator()(EVT const&, FSM& fsm, S&, T&)
        {
            return fsm.mineral_attached;
        }
    };

    // 检查抓取传感器
    template <>
    struct Guard<by_name("grip_success")>
    {
        template <class EVT, class FSM, class S, class T>
        bool operator()(EVT const&, FSM& fsm, S&, T&)
        {
            // return fsm.check_grip_sensor();
            return true;
        }
    };

    // 检查能否存 (库存 < 2)
    template <>
    struct Guard<by_name("can_deposit")>
    {
        template <class EVT, class FSM, class S, class T>
        bool operator()(EVT const&, FSM& fsm, S&, T&)
        {
            return fsm.stored_count < 2;
        }
    };

    // 检查能否取 (库存 > 0)
    template <>
    struct Guard<by_name("can_retrieve")>
    {
        template <class EVT, class FSM, class S, class T>
        bool operator()(EVT const&, FSM& fsm, S&, T&)
        {
            return fsm.stored_count > 0;
        }
    };

    // 检查能否进入存取模式 (合并逻辑冲突+存取判断)
    template <>
    struct Guard<by_name("can_enter_store")>
    {
        template <class EVT, class FSM, class S, class T>
        bool operator()(EVT const&, FSM& fsm, S&, T&)
        {
            // 冲突: 有矿但仓库满 / 没矿但仓库空
            if (fsm.mineral_attached && fsm.stored_count >= 2)
            {
                fsm.m_logger->error("逻辑冲突！手持状态与库存状态不匹配，无法执行。");
                return false;
            }
            if (!fsm.mineral_attached && fsm.stored_count <= 0)
            {
                fsm.m_logger->error("逻辑冲突！手持状态与库存状态不匹配，无法执行。");
                return false;
            }
            return true;
        }
    };
}

namespace yandy::modules
{
    namespace msm = boost::msm;

    namespace detail
    {
        using namespace msm::front::puml;

        class YandyArmFSMDef : public msm::front::state_machine_def<YandyArmFSMDef>
        {
        public:
            YandyArmFSMDef()
            {
                m_logger = core::create_logger("YandyArmFSM", spdlog::level::info);
            }

            BOOST_MSM_PUML_DECLARE_TABLE(YANDY_ARM_PUML_CONTENT);

            template <class FSM, class Event>
            void no_transition(Event const&, FSM& fsm, int)
            {
                m_logger->warn("当前状态不支持此指令");
            }

            std::shared_ptr<spdlog::logger> m_logger;
            bool mineral_attached = false;
            int stored_count = 0;
        };

        typedef msm::back11::state_machine<YandyArmFSMDef> YandyArmFSMBackend;
    }

    class YandyArmFSM
    {
    public:
        YandyArmFSM();
        void processCmd(YandyControlCmd cmd);
        void logState() const;
        YandyState getState() const;
        bool hasMineralAttached() const { return m_fsm.mineral_attached; }
        int getStoredCount() const { return m_fsm.stored_count; }

    private:
        detail::YandyArmFSMBackend m_fsm;
        std::shared_ptr<spdlog::logger> m_logger;
        void sync_state();
        std::atomic<YandyState> m_current_state{YandyState::Disabled};
    };
}

#endif //YANDY_ARM_FSM_HPP
