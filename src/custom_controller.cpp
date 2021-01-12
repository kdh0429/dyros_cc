#include "custom_controller.h"

CustomController::CustomController(DataContainer &dc, RobotData &rd) : dc_(dc), rd_(rd), wbc_(dc.wbc_)
{
    ControlVal_.setZero();

	setWeights();
	readMocapData();
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
		mode_time = rd_.control_time_ - tc.command_time;
		if (mode_time == 0.0 || round((mode_time-mode_time_pre)*1000)/1000 >= policy_eval_dt)
		{
			processObservation();
			feedforwardPolicy();
			// cout<<mode_time - mode_time_pre<<endl;
			mode_time_pre = mode_time;
		}
		
		torqueCalculation();
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

void CustomController::setWeights()
{
	file[0].open("/home/kim/red_ws/src/dyros_cc/weight/obs_mean.txt", ios::in);
	file[1].open("/home/kim/red_ws/src/dyros_cc/weight/obs_variance.txt", ios::in);
	file[2].open("/home/kim/red_ws/src/dyros_cc/weight/mlp_extractor_policy_net_0_weight.txt", ios::in);
	file[3].open("/home/kim/red_ws/src/dyros_cc/weight/mlp_extractor_policy_net_0_bias.txt", ios::in);
	file[4].open("/home/kim/red_ws/src/dyros_cc/weight/mlp_extractor_policy_net_2_weight.txt", ios::in);
	file[5].open("/home/kim/red_ws/src/dyros_cc/weight/mlp_extractor_policy_net_2_bias.txt", ios::in);
	file[6].open("/home/kim/red_ws/src/dyros_cc/weight/action_net_weight.txt", ios::in);
	file[7].open("/home/kim/red_ws/src/dyros_cc/weight/action_net_bias.txt", ios::in);

	int index = 0;
	float temp;
	if(!file[0].is_open())
	{
		std::cout<<"Can not find the obs_mean file"<<std::endl;
	}
	while(!file[0].eof())
	{
		file[0] >> temp;
		if(temp != '\n')
		{
			obs_mean[index] = temp;
			index ++;
		}
	}

	index = 0;
	if(!file[1].is_open())
	{
		std::cout<<"Can not find the obs_variance file"<<std::endl;
	}
	while(!file[1].eof())
	{
		file[1] >> temp;
		if(temp != '\n')
		{
			obs_var[index] = temp;
			index ++;
		}
	}

	index = 0;
	if(!file[2].is_open())
	{
		std::cout<<"Can not find the mlp_extractor_policy_net_0_weight file"<<std::endl;
	}
	while(!file[2].eof())
	{
		file[2] >> temp;
		if(temp != '\n')
		{
			W_ih1[index] = temp;
			index ++;
		}
	}
  
	index = 0;
	if(!file[3].is_open())
	{
		std::cout<<"Can not find the mlp_extractor_policy_net_0_bias file"<<std::endl;
	}
	while(!file[3].eof())
	{
		file[3] >> temp;
		if(temp != '\n')
		{
			b_ih1[index] = temp;
			index ++;
		}
	}
  
	index = 0;
	if(!file[4].is_open())
	{
		std::cout<<"Can not find the mlp_extractor_policy_net_2_weight file"<<std::endl;
	}
	while(!file[4].eof())
	{
		file[4] >> temp;
		if(temp != '\n')
		{
			W_h1h2[index] = temp;
			index ++;
		}
	}
  
	index = 0;
	if(!file[5].is_open())
	{
		std::cout<<"Can not find the mlp_extractor_policy_net_2_bias file"<<std::endl;
	}
	while(!file[5].eof())
	{
		file[5] >> temp;
		if(temp != '\n')
		{
			b_h1h2[index] = temp;
			index ++;
		}
	}

	index = 0;
	if(!file[6].is_open())
	{
		std::cout<<"Can not find the action_net_weight file"<<std::endl;
	}
	while(!file[6].eof())
	{
		file[6] >> temp;
		if(temp != '\n')
		{
			W_h2o[index] = temp;
			index ++;
		}
	}

	index = 0;
	if(!file[7].is_open())
	{
		std::cout<<"Can not find the action_net_bias file"<<std::endl;
	}
	while(!file[7].eof())
	{
		file[7] >> temp;
		if(temp != '\n')
		{
			b_h2o[index] = temp;
			index ++;
		}
	}
}

void CustomController::readMocapData()
{
	file[8].open("/home/kim/red_ws/src/dyros_cc/motions/processed_data_tocabi.txt", ios::in);

	int row = 0;
	int col = 0;
	float temp;
	if(!file[8].is_open())
	{
		std::cout<<"Can not find the processed_data_tocabi file"<<std::endl;
	}
	while(!file[8].eof())
	{
		file[8] >> temp;
		if(temp != '\n')
		{
			mocap_data[row][col] = temp;
			col ++;
			if (col==31)
			{
				row ++;
				col = 0;
			}
		}
	}
}

void CustomController::processObservation()
{
	// Phase
	phase = fmod((init_mocap_data_idx + fmod(mode_time, mocap_cycle_period) / mocap_cycle_dt), mocap_data_num) / mocap_data_num;
	obs[0] = phase;

	// qpos
	// Eigen::Matrix3d Rpelvis = (rd_.link_[Pelvis].Rotm);
	// obs[1] = sqrt(1.0 + Rpelvis(0,0) + Rpelvis(1,1) + Rpelvis(2,2)) / 2.0;
	// obs[2] = (Rpelvis(2,1) - Rpelvis(1,2)) / (4*obs[1]);
	// obs[3] = (Rpelvis(0,2) - Rpelvis(2,0)) / (4*obs[1]);
	// obs[4] = (Rpelvis(1,0) - Rpelvis(0,1)) / (4*obs[1]);
	
	obs[1] = rd_.q_virtual_(MODEL_DOF+6);
	obs[2] = rd_.q_virtual_(3);
	obs[3] = rd_.q_virtual_(4);
	obs[4] = rd_.q_virtual_(5);

	obs[5] = rd_.q_(0);
	obs[6] = rd_.q_(1);
	obs[7] = rd_.q_(2);
	obs[8] = rd_.q_(3);
	obs[9] = rd_.q_(4);
	obs[10] = rd_.q_(5);

	obs[11] = rd_.q_(6);
	obs[12] = rd_.q_(7);
	obs[13] = rd_.q_(8);
	obs[14] = rd_.q_(9);
	obs[15] = rd_.q_(10);
	obs[16] = rd_.q_(11);

	obs[17] = rd_.q_(12);
	obs[18] = rd_.q_(13);
	obs[19] = rd_.q_(14);

	obs[20] = rd_.q_(15);
	obs[21] = rd_.q_(16);
	obs[22] = rd_.q_(17);
	obs[23] = rd_.q_(19);

	obs[24] = rd_.q_(25);
	obs[25] = rd_.q_(26);
	obs[26] = rd_.q_(27);
	obs[27] = rd_.q_(29);

	// qvel
	// obs[28] = rd_.link_[Pelvis].v(0);
	// obs[29] = rd_.link_[Pelvis].v(1);
	// obs[30] = rd_.link_[Pelvis].v(2);

	// obs[31] = rd_.link_[Pelvis].w(0);
	// obs[32] = rd_.link_[Pelvis].w(1);
	// obs[33] = rd_.link_[Pelvis].w(2);

	obs[28] = rd_.q_dot_virtual_(0);
	obs[29] = rd_.q_dot_virtual_(1);
	obs[30] = rd_.q_dot_virtual_(2);

	obs[31] = rd_.q_dot_virtual_(3);
	obs[32] = rd_.q_dot_virtual_(4);
	obs[33] = rd_.q_dot_virtual_(5);

	obs[34] = rd_.q_dot_(0);
	obs[35] = rd_.q_dot_(1);
	obs[36] = rd_.q_dot_(2);
	obs[37] = rd_.q_dot_(3);
	obs[38] = rd_.q_dot_(4);
	obs[39] = rd_.q_dot_(5);

	obs[40] = rd_.q_dot_(6);
	obs[41] = rd_.q_dot_(7);
	obs[42] = rd_.q_dot_(8);
	obs[43] = rd_.q_dot_(9);
	obs[44] = rd_.q_dot_(10);
	obs[45] = rd_.q_dot_(11);

	obs[46] = rd_.q_dot_(12);
	obs[47] = rd_.q_dot_(13);
	obs[48] = rd_.q_dot_(14);

	obs[49] = rd_.q_dot_(15);
	obs[50] = rd_.q_dot_(16);
	obs[51] = rd_.q_dot_(17);
	obs[52] = rd_.q_dot_(19);

	obs[53] = rd_.q_dot_(25);
	obs[54] = rd_.q_dot_(26);
	obs[55] = rd_.q_dot_(27);
	obs[56] = rd_.q_dot_(29);

	// Pelvis Z position
	obs[57] = rd_.q_virtual_(2); //rd_.link_[Pelvis].xpos(2);

	// cout<< "OBS: " << obs[0]<<" " << rd_.link_[Pelvis].xpos(2) << " " << obs[3] << " " << obs[4] << endl;
}

void CustomController::feedforwardPolicy()
{
	// Normalize
	for(int i=0; i<58; i++)
	{
		input[i] = (obs[i]-obs_mean[i])/sqrt(obs_var[i]+1.0e-08);
	}

	// Network Feedforward
	// Input Layer
	for(int row=0; row<256; row++)
	{
		hidden1[row] = b_ih1[row];
		for(int col=0; col<58; col++)
		{
			hidden1[row] +=  W_ih1[58*row+col] * input[col];
		}
		if (hidden1[row] < 0.0)
			hidden1[row] = 0.0;
	}
	// Hidden Layer
	for(int row=0; row<256; row++)
	{
		hidden2[row] = b_h1h2[row];
		for(int col=0; col<256; col++)
		{
			hidden2[row] +=  W_h1h2[256*row+col] * hidden1[col];
		}
		if (hidden2[row] < 0.0)
			hidden2[row] = 0.0;
	}

	// Output Layer
	for(int row=0; row<23; row++)
	{
		policy_output[row] = b_h2o[row];
		for(int col=0; col<256; col++)
		{
			policy_output[row] +=  W_h2o[256*row+col] * hidden2[col];
		}
	}

	// Reference Trajectory Calculation
	next_step_time = mode_time + policy_eval_dt;
	local_time = fmod(next_step_time, mocap_cycle_period);
	local_time_plus_init = fmod(local_time + init_mocap_data_idx*mocap_cycle_dt, mocap_cycle_period);
	mocap_data_idx = (init_mocap_data_idx + int(local_time / mocap_cycle_dt)) % mocap_data_num;
	next_idx = mocap_data_idx + 1;


	for(int i=0; i<23; i++)
	{
		target_data_qpos[i] = DyrosMath::cubic(local_time_plus_init, mocap_data[mocap_data_idx][0], mocap_data[next_idx][0], mocap_data[mocap_data_idx][i+8], mocap_data[next_idx][i+8], 0.0, 0.0);
	}
}

void CustomController::torqueCalculation()
{
	// Leg Torque
	float gear_ratio = 200; //DyrosMath::cubic(mode_time, 0.0, 5*mocap_cycle_period, 200.0, 100.0, 0.0, 0.0);
	cout<<"GR: " << gear_ratio<<endl;
	for(int i = 0; i<6; i++)
	{
		ControlVal_(i) = (900*(target_data_qpos[i] + policy_output[i] - rd_.q_(i)) + 60*(-rd_.q_dot_(i)))/gear_ratio;
		ControlVal_(i+6) = (900*(target_data_qpos[i+6] + policy_output[i+6] - rd_.q_(i+6)) + 60*(-rd_.q_dot_(i+6)))/gear_ratio;
	}
	// Waist Torque
	for(int i = 0; i<3; i++)
	{
		ControlVal_(i+12) = (900*(target_data_qpos[i+12] + policy_output[i+12] - rd_.q_(i+12)) +60*(-rd_.q_dot_(i+12)))/gear_ratio;
	}              
	// Arm Torque
	for(int i = 0; i<3; i++)
	{
		ControlVal_(i+15) = (900*(target_data_qpos[i+15] + policy_output[i+15] - rd_.q_(i+15)) + 60*(-rd_.q_dot_(i+15)))/gear_ratio;
		ControlVal_(i+25) = (900*(target_data_qpos[i+19] + policy_output[i+19] - rd_.q_(i+25)) + 60*(-rd_.q_dot_(i+25)))/gear_ratio;
	}
	ControlVal_(19) = (900*(target_data_qpos[18] + policy_output[18] - rd_.q_(19)) + 60*(-rd_.q_dot_(19)))/gear_ratio;
	ControlVal_(29) = (900*(target_data_qpos[22] + policy_output[22] - rd_.q_(29)) + 60*(-rd_.q_dot_(29)))/gear_ratio;

	// Redundant Joints
	// Arm Link Joint
	ControlVal_(18) = 0.0;//(100*(-3.14/2.0 - rd_.q_(18)) + 10*(-rd_.q_dot_(18)))/gear_ratio; // Left
	ControlVal_(28) = 0.0;//(100*(3.14/2.0 - rd_.q_(28)) + 10*(-rd_.q_dot_(28)))/gear_ratio; // Right
	// Neck Joints
	ControlVal_(23) = 0.0;//(100*(0.0 - rd_.q_(23)) + 5*(-rd_.q_dot_(23)))/gear_ratio;
	ControlVal_(24) = 0.0;//(100*(0.0 - rd_.q_(24)) + 5*(-rd_.q_dot_(24)))/gear_ratio;
	// Lower Arm Joints
	ControlVal_(20) = 0.0;//(100*(0.0 - rd_.q_(20)) + 5*(-rd_.q_dot_(20)))/gear_ratio; // Left ForeArm
	ControlVal_(21) = 0.0;//(100*(0.0 - rd_.q_(21)) + 5*(-rd_.q_dot_(21)))/gear_ratio; // Left Wrist1
	ControlVal_(22) = 0.0;//(100*(0.0 - rd_.q_(22)) + 5*(-rd_.q_dot_(22)))/gear_ratio; // Left Wrist2
	ControlVal_(30) = 0.0;//(100*(0.0 - rd_.q_(30)) + 5*(-rd_.q_dot_(30)))/gear_ratio; // Right ForeArm
	ControlVal_(31) = 0.0;//(100*(0.0 - rd_.q_(31)) + 5*(-rd_.q_dot_(31)))/gear_ratio; // Right Wrist1 
	ControlVal_(32) = 0.0;//(100*(0.0 - rd_.q_(32)) + 5*(-rd_.q_dot_(32)))/gear_ratio; // Right Wrist2
	// ControlVal_ = wbc_.gravity_compensation_torque(rd_, false, false);
}