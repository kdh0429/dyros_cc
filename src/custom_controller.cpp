#include "custom_controller.h"

CustomController::CustomController(DataContainer &dc, RobotData &rd) : dc_(dc), rd_(rd), wbc_(dc.wbc_)
{
    ControlVal_.setZero();
}

Eigen::VectorQd CustomController::getControl()
{
    return ControlVal_;
}
void CustomController::taskCommandToCC(TaskCommand tc_)
{
    tc = tc_;
}

void CustomController::computeSlow()
{
    if (tc.mode == 10)
    {
        wbc_.set_contact(rd_, 1, 1);

        int task_number = 6 + 6 + 6;
        rd_.J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
        rd_.f_star.setZero(task_number);

        rd_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[COM_id].Jac;
        rd_.J_task.block(6, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Left_Hand].Jac;
        rd_.J_task.block(12, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Right_Hand].Jac;

        rd_.link_[COM_id].x_desired = rd_.link_[COM_id].x_init;
        rd_.link_[COM_id].rot_desired = rd_.link_[COM_id].rot_init;
        rd_.link_[COM_id].Set_Trajectory_from_quintic(rd_.control_time_, tc.command_time, tc.command_time + tc.traj_time);
        rd_.link_[COM_id].Set_Trajectory_rotation(rd_.control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

        rd_.link_[Left_Hand].Set_Trajectory_from_quintic(rd_.control_time_, tc.command_time, tc.command_time + tc.traj_time);
        rd_.link_[Left_Hand].Set_Trajectory_rotation(rd_.control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

        rd_.link_[Right_Hand].Set_Trajectory_from_quintic(rd_.control_time_, tc.command_time, tc.command_time + tc.traj_time);
        rd_.link_[Right_Hand].Set_Trajectory_rotation(rd_.control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

        rd_.f_star.segment(0, 6) = wbc_.getfstar6d(rd_, COM_id);
        rd_.f_star.segment(6, 6) = wbc_.getfstar6d(rd_, Left_Hand);
        rd_.f_star.segment(12, 6) = wbc_.getfstar6d(rd_, Right_Hand);

        ControlVal_ = wbc_.task_control_torque_QP2(rd_, rd_.J_task, rd_.f_star);
    }
    else if(tc.mode == 11)
    {
        const int arm_task_number = 6;
        const int arm_dof = 8;
        ////////// CoM Control //////////////////////
        // wbc_.set_contact(rd_, 1, 1);
        // int task_number = 6;
        // rd_.J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
        // rd_.f_star.setZero(task_number);

        // rd_.J_task = rd_.link_[COM_id].Jac;
        // rd_.J_task.block(0, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[COM_id].Jac_COM_p;
        // rd_.J_task.block(0, 21, 3, arm_dof).setZero(); // Exclude Left Arm Jacobian
        // rd_.J_task.block(0, 31, 3, arm_dof).setZero(); // Exclude Right Arm Jacobian

        // rd_.link_[COM_id].x_desired = rd_.link_[COM_id].x_init;
        // rd_.link_[COM_id].Set_Trajectory_from_quintic(rd_.control_time_, tc.command_time, tc.command_time + tc.traj_time);

        // rd_.f_star = wbc_.getfstar6d(rd_, COM_id);
        // ControlVal_ = wbc_.task_control_torque_QP2(rd_, rd_.J_task, rd_.f_star);
        ControlVal_ = wbc_.gravity_compensation_torque(rd_, true, false)

        ///////// Jacobian based ik arm controller (Daegyu, Donghyeon)/////////////////
        Eigen::Matrix<double, 2*arm_task_number, 2*arm_dof> J_task_Arm;
        J_task_Arm.setZero();
        J_task_Arm.block(0, 0, arm_task_number, arm_dof) = rd_.link_[Left_Hand].Jac.block(0,21,arm_task_number,arm_dof);
        J_task_Arm.block(arm_task_number, arm_dof, arm_task_number, arm_dof) = rd_.link_[Right_Hand].Jac.block(0,31,arm_task_number,arm_dof);
        Eigen::Matrix<double, 2*arm_dof, 2*arm_task_number> J_task_inv;
        J_task_inv = DyrosMath::pinv_SVD(J_task_Arm);

        rd_.link_[Left_Hand].Set_Trajectory_from_quintic(rd_.control_time_, tc.command_time, tc.command_time + tc.traj_time);
        rd_.link_[Left_Hand].Set_Trajectory_rotation(rd_.control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

        rd_.link_[Right_Hand].Set_Trajectory_from_quintic(rd_.control_time_, tc.command_time, tc.command_time + tc.traj_time);
        rd_.link_[Right_Hand].Set_Trajectory_rotation(rd_.control_time_, tc.command_time, tc.command_time + tc.traj_time, false);
        
        Eigen::Vector12d x_dot_desired;
        Eigen::Vector6d error_v;
        Eigen::Vector6d error_w;                
        Eigen::Vector6d k_pos;
        Eigen::Vector6d k_rot;

        for (int i = 0; i<6; i++)
        {
            k_pos(i) = 10;
            k_rot(i) = 4;
        }

        error_v.segment<3>(0) = rd_.link_[Left_Hand].x_traj -  rd_.link_[Left_Hand].xpos;
        error_v.segment<3>(3) = rd_.link_[Right_Hand].x_traj -  rd_.link_[Right_Hand].xpos;

        error_w.segment<3>(0) = -DyrosMath::getPhi(rd_.link_[Left_Hand].Rotm, rd_.link_[Left_Hand].r_traj);
        error_w.segment<3>(3) = -DyrosMath::getPhi(rd_.link_[Right_Hand].Rotm, rd_.link_[Right_Hand].r_traj);

        for(int i = 0; i<3; i++)
        {
            x_dot_desired(i) = rd_.link_[Left_Hand].v_traj(i) + k_pos(i)*error_v(i); // linear velocity
            x_dot_desired(i+3) = rd_.link_[Left_Hand].w_traj(i) + k_rot(i)*error_w(i);
            x_dot_desired(i+6) = rd_.link_[Right_Hand].v_traj(i) + k_pos(i+3)*error_v(i+3); // linear velocity
            x_dot_desired(i+9) = rd_.link_[Right_Hand].w_traj(i) + k_rot(i+3)*error_w(i+3);
        }
        VectorXd q_dot_arm;
        q_dot_arm = J_task_inv*x_dot_desired;
        for (int i=0; i<arm_dof; i++)
        {
            rd_.q_dot_desired_(15+i) = q_dot_arm(i);
            rd_.q_dot_desired_(25+i) = q_dot_arm(i+arm_dof);
        }
        rd_.q_desired_.segment<8>(15) = rd_.q_.segment<8>(15) + rd_.q_dot_desired_.segment<8>(15)*(rd_.control_time_ - rd_.control_time_pre_);
        rd_.q_desired_.segment<8>(25) = rd_.q_.segment<8>(25) + rd_.q_dot_desired_.segment<8>(25)*(rd_.control_time_ - rd_.control_time_pre_);

        Eigen::MatrixXd kp(8,1);
        Eigen::MatrixXd kv(8,1);
        
        for(int i = 0; i<8; i++)
        {
            kp(i) = 9;
            kv(i) = 6;
        }

        for(int i = 0; i<8; i++)
        {
            ControlVal_(i+15) += kp(i)*(rd_.q_desired_(i+15) - rd_.q_(i+15)) + kv(i)*(rd_.q_dot_desired_(i+15) - rd_.q_dot_(i+15));
            ControlVal_(i+25) += kp(i)*(rd_.q_desired_(i+25) - rd_.q_(i+25)) + kv(i)*(rd_.q_dot_desired_(i+25) - rd_.q_dot_(i+25));
        }           
        rd_.control_time_pre_ = rd_.control_time_;
    }
    else if(tc.mode == 12)
    {
        const int arm_task_number = 6;
        const int arm_dof = 8;
        ///////// Jacobian based ik arm controller (Daegyu, Donghyeon)/////////////////
        Eigen::Matrix<double, 2*arm_task_number, 2*arm_dof> J_task_Arm;
        J_task_Arm.setZero();
        J_task_Arm.block(0, 0, arm_task_number, arm_dof) = rd_.link_[Left_Hand].Jac.block(0,21,arm_task_number,arm_dof);
        J_task_Arm.block(arm_task_number, arm_dof, arm_task_number, arm_dof) = rd_.link_[Right_Hand].Jac.block(0,31,arm_task_number,arm_dof);
        Eigen::Matrix<double, 2*arm_dof, 2*arm_task_number> J_task_inv;
        J_task_inv = DyrosMath::pinv_SVD(J_task_Arm);

        rd_.link_[Left_Hand].Set_Trajectory_from_quintic(rd_.control_time_, tc.command_time, tc.command_time + tc.traj_time);
        rd_.link_[Left_Hand].Set_Trajectory_rotation(rd_.control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

        rd_.link_[Right_Hand].Set_Trajectory_from_quintic(rd_.control_time_, tc.command_time, tc.command_time + tc.traj_time);
        rd_.link_[Right_Hand].Set_Trajectory_rotation(rd_.control_time_, tc.command_time, tc.command_time + tc.traj_time, false);
        
        Eigen::Vector12d x_dot_desired;
        Eigen::Vector6d error_v;
        Eigen::Vector6d error_w;                
        Eigen::Vector6d k_pos;
        Eigen::Vector6d k_rot;

        for (int i = 0; i<6; i++)
        {
            k_pos(i) = 10;
            k_rot(i) = 4;
        }

        error_v.segment<3>(0) = rd_.link_[Left_Hand].x_traj -  rd_.link_[Left_Hand].xpos;
        error_v.segment<3>(3) = rd_.link_[Right_Hand].x_traj -  rd_.link_[Right_Hand].xpos;

        error_w.segment<3>(0) = -DyrosMath::getPhi(rd_.link_[Left_Hand].Rotm, rd_.link_[Left_Hand].r_traj);
        error_w.segment<3>(3) = -DyrosMath::getPhi(rd_.link_[Right_Hand].Rotm, rd_.link_[Right_Hand].r_traj);

        for(int i = 0; i<3; i++)
        {
            x_dot_desired(i) = rd_.link_[Left_Hand].v_traj(i) + k_pos(i)*error_v(i); // linear velocity
            x_dot_desired(i+3) = rd_.link_[Left_Hand].w_traj(i) + k_rot(i)*error_w(i);
            x_dot_desired(i+6) = rd_.link_[Right_Hand].v_traj(i) + k_pos(i+3)*error_v(i+3); // linear velocity
            x_dot_desired(i+9) = rd_.link_[Right_Hand].w_traj(i) + k_rot(i+3)*error_w(i+3);
        }
        VectorXd q_dot_arm;
        
        q_dot_arm = J_task_inv*x_dot_desired;
        for (int i=0; i<arm_dof; i++)
        {
            rd_.q_dot_desired_(15+i) = q_dot_arm(i);
            rd_.q_dot_desired_(25+i) = q_dot_arm(i+arm_dof);
        }
        rd_.q_desired_.segment<8>(15) = rd_.q_.segment<8>(15) + rd_.q_dot_desired_.segment<8>(15)*(rd_.control_time_ - rd_.control_time_pre_);
        rd_.q_desired_.segment<8>(25) = rd_.q_.segment<8>(25) + rd_.q_dot_desired_.segment<8>(25)*(rd_.control_time_ - rd_.control_time_pre_);

        Eigen::MatrixXd kp(8,1);
        Eigen::MatrixXd kv(8,1);
        
        for(int i = 0; i<8; i++)
        {
            kp(i) = 9;
            kv(i) = 6;
        }

        for(int i = 0; i<8; i++)
        {
            ControlVal_(i+15) = kp(i)*(rd_.q_desired_(i+15) - rd_.q_(i+15)) + kv(i)*(rd_.q_dot_desired_(i+15) - rd_.q_dot_(i+15));
            ControlVal_(i+25) = kp(i)*(rd_.q_desired_(i+25) - rd_.q_(i+25)) + kv(i)*(rd_.q_dot_desired_(i+25) - rd_.q_dot_(i+25));
        }           
        rd_.control_time_pre_ = rd_.control_time_;
    }
}

void CustomController::computeFast()
{
    if (tc.mode == 10)
    {
    }
    else if (tc.mode == 11)
    {
    }
}