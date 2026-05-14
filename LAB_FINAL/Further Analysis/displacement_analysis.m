%% Analisi displacement
 delta_traj_mocap = q_model_lab-q_mocap_cal_lab;
 delta_traj_ekf = q_model_lab-q_ekf_lab;


 plot_unicycle_trajectory(delta_traj_mocap,delta_traj_ekf,'displacement');
