clear all;
close all;
addpath(genpath('utils'));

%% Set simulation parameters
r_nominal = 0.03;
d_nominal = 0.165;
r = r_nominal;
d = d_nominal;
r_actual = r_nominal;
d_actual = d_nominal;
% r_actual = 0.0302;
% d_actual = 0.1694;
omega_M = 12;

controller_index = 2; % 1->cartesian, 2->posture
flg_replanning = true;
% desired configuration
q_d = [0;0;0];
% initial configuration
Q_INIT = [1;1;0];
% simulation time
T_SIM = 10;

%% Set controller parameters
if controller_index == 1
    % cartesian
    k_1 = 10; 
    k_2 = 10;
    control_par = [k_1, k_2, 0];
else
    % posture
    k_1 = 10; 
    k_2 = 10;
    k_3 = 1;
    control_par = [k_1, k_2, k_3];
end