%% Numerical Activity 3 (NA3): Feedback control
%% PART 2: POSTURE REGULATION

clear all;
close all;
addpath(genpath(fullfile(pwd,'..','utils')));

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

controller_index = 1; % 1->cartesian, 2->posture
flg_replanning = true;
% desired configuration
q_d = [2;2;0];
% initial configuration
Q_INIT = [1;1;0];
% simulation time
T_SIM = 20;

%% Set controller parameters
if controller_index == 1
    % cartesian
    k_1 = 10; 
    k_2 = 10;
    control_par = [k_1, k_2, 0];
else
    % posture
    k_1 = 1; 
    k_2 = 1;
    k_3 = 10;
    control_par = [k_1, k_2, k_3];
end