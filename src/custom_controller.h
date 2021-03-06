#include <tocabi_controller/data_container.h>
#include <tocabi_controller/link.h>
#include "math_type_define.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"

class CustomController
{
public:
    CustomController(DataContainer &dc,RobotData &rd);
    Eigen::VectorQd getControl();

    void taskCommandToCC(TaskCommand tc_);
    
    void computeSlow();
    void computeFast();
    void computePlanner();
    
    DataContainer &dc_;
    RobotData &rd_;
    WholebodyController &wbc_;
    TaskCommand tc;

private:
    Eigen::VectorQd ControlVal_;

    void setWeights();
    void readMocapData();
    void processObservation();
    void feedforwardPolicy();
    void torqueCalculation();

    ifstream file[9];
    float W_ih1[59*256];
	float b_ih1[256];
	float W_h1h2[256*256];
	float b_h1h2[256];
	float W_h2o[256*23];
	float b_h2o[23];
    float obs_mean[59];
    float obs_var[59];

    float obs[59];
    float input[59];
    float hidden1[256];
    float hidden2[256];
    float policy_output[23];

    float vel_scale = 1.0;
    float policy_eval_dt = 0.001;
    float mocap_data[39][31];
	float mocap_cycle_dt = vel_scale*0.033332;
    int mocap_data_num = 38;
    float mocap_cycle_period = mocap_data_num * mocap_cycle_dt;
    int init_mocap_data_idx = 11;
    int mocap_data_idx;
    int next_idx;
    float phase;
    float mode_time;
    float mode_time_pre=0.0;
    float next_step_time;
    float local_time;
    float local_time_plus_init;

    float target_data_qpos[23];

    ros::NodeHandle node;
    ros::Subscriber vel_scale_sub;
    void velScaleCallback(const std_msgs::Float32::ConstPtr& msg);

};