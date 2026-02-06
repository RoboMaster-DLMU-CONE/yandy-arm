#include <yandy/modules/DynamicsSolver.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include "yandy/core/Logger.hpp"

#define YANDY_URDF_PATH YANDY_CONFIG_PATH "urdf/yandy_urdf.urdf"

namespace yandy::modules
{
    DynamicsSolver::DynamicsSolver()
    {
        m_logger = core::create_logger("Solver", spdlog::level::info);
        m_logger->info("Initializing solver, loading urdf from {}", YANDY_URDF_PATH);
        try
        {
            pinocchio::urdf::buildModel(YANDY_URDF_PATH, m_model);
        }
        catch (const std::exception& e)
        {
            m_logger->error("Error loading URDF: {}", e.what());
            throw;
        }
        m_data = pinocchio::Data(m_model);
        if (m_model.existJointName("joint_5"))
        {
            ee_joint_id_ = m_model.getJointId("joint_5");
        }
        else
        {
            m_logger->error("No valid 'joint_5' joint found, aborting...");
            throw std::runtime_error("No valid 'joint_5' joint found");
        }
        f_ext_.resize(m_model.njoints, pinocchio::Force::Zero());

        m_logger->info(" Model loaded. Joints: {}, DoF: {}", m_model.njoints, m_model.nv);
    }

    void DynamicsSolver::updateKinematics(const common::VectorJ& q, const common::VectorJ& v)
    {
        m_current_q = q;
        m_current_v = v;
        pinocchio::forwardKinematics(m_model, m_data, m_current_q, m_current_v);
        pinocchio::updateFramePlacements(m_model, m_data);
    }

    common::VectorJ DynamicsSolver::computeRNEA(const common::VectorJ& acc_des, const common::Vector6& ext_wrench_world)
    {
        std::ranges::fill(f_ext_, pinocchio::Force::Zero());
        if (!ext_wrench_world.isZero())
        {
            // Pinocchio 的 f_ext 要求定义在“关节局部坐标系”下
            // 我们输入的是“世界坐标系”下的力，所以需要转换
            // 获取末端关节在世界坐标系下的位姿 (Rotation Matrix)
            // data.oMi[id] 存储了从 Local 到 World 的变换
            const auto& iso_world_to_local = m_data.oMi[ee_joint_id_].inverse();

            // 将世界坐标系的力 (Force + Torque) 转换到局部坐标系
            // act() 是 Pinocchio 的空间变换函数
            const pinocchio::Force f_world(ext_wrench_world.head<3>(), ext_wrench_world.tail<3>());
            const pinocchio::Force f_local = iso_world_to_local.act(f_world);

            // 施加到对应关节
            f_ext_[ee_joint_id_] = f_local;
        }
        // 运行 RNEA
        return pinocchio::rnea(m_model, m_data,
                               m_current_q, // 使用 updateKinematics 时存下的位置
                               m_current_v,
                               acc_des,
                               f_ext_);
    }

    common::VectorJ DynamicsSolver::computeGravity()
    {
        return pinocchio::rnea(m_model, m_data, m_current_q, common::VectorJ::Zero(), common::VectorJ::Zero());
    }

    Eigen::Isometry3d DynamicsSolver::getEndEffectorPose() const
    {
        pinocchio::SE3 se3 = m_data.oMi[ee_joint_id_];

        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translation() = se3.translation();
        pose.linear() = se3.rotation();

        return pose;
    }
}
