clc;
clear all;
close all;
load("DATI_LABORATORIO_PARTE_1.mat")

%% pulizia dati

% --- esperimento 1 ---
exp_1.Ta = results_part1(1).Ta;
exp_1.Tc = results_part1(1).Tc;
exp_1.q_des = squeeze(results_part1(1).q_des);
exp_1.w_gyro = results_part1(1).w_gyro;
exp_1.acce = results_part1(1).acce;
exp_1.q_motion_capture = results_part1(1).q_motion_capture';
exp_1.ws_des = results_part1(1).ws_des;
exp_1.ws_meas = results_part1(1).ws_meas;


% --- esperimento 2 ---
exp_2.Ta = results_part1(2).Ta;
exp_2.Tc = results_part1(2).Tc;
exp_2.q_des = squeeze(results_part1(2).q_des);
exp_2.w_gyro = results_part1(2).w_gyro;
exp_2.acce = results_part1(2).acce;
exp_2.q_motion_capture = results_part1(2).q_motion_capture';
exp_2.ws_des = results_part1(2).ws_des;
exp_2.ws_meas = results_part1(2).ws_meas;

% --- esperimento 3 ---
exp_3.Ta = results_part1(3).Ta;
exp_3.Tc = results_part1(3).Tc;
exp_3.q_des = squeeze(results_part1(3).q_des);
exp_3.w_gyro = results_part1(3).w_gyro;
exp_3.acce = results_part1(3).acce;
exp_3.q_motion_capture = results_part1(3).q_motion_capture';
exp_3.ws_des = results_part1(3).ws_des;
exp_3.ws_meas = results_part1(3).ws_meas;

%%


plot_unicycle_trajectory(exp_1.q_des, exp_1.q_motion_capture,'Tc = 30');
id1 = id_unicycle(exp_1);

plot_unicycle_trajectory(exp_2.q_des, exp_2.q_motion_capture,'Tc = 18');
id2 = id_unicycle(exp_2);


plot_unicycle_trajectory(exp_3.q_des, exp_3.q_motion_capture, 'Tc = 45');
id3 = id_unicycle(exp_3);
%%

plot_unicycle_trajectory_calibrated(exp_1.q_des, exp_1.q_motion_capture,id1.q_motion_capture_cal,'Tc = 30');
plot_unicycle_trajectory_calibrated(exp_2.q_des, exp_2.q_motion_capture,id2.q_motion_capture_cal,'Tc = 18');
plot_unicycle_trajectory_calibrated(exp_3.q_des, exp_3.q_motion_capture,id3.q_motion_capture_cal,'Tc = 45');

%%

plot_wheel_speeds_exp(exp_1);
plot_wheel_speeds_exp(exp_2);
plot_wheel_speeds_exp(exp_3);