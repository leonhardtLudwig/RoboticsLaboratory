%% Numerical Activity 3 (NA3): Feedback control
%% PART 2: POSTURE REGULATION

clear all;
close all;
addpath(genpath(fullfile(pwd,'..','utils')));

%% Set simulation parameters
r_nominal = 0.03;
d_nominal = 0.165;
%r = r_nominal;
%d = d_nominal;
%r_actual = r_nominal;
%d_actual = d_nominal;
 r_actual = 0.0302;
 d_actual = 0.1694;
 r = r_actual;
d = d_actual;
omega_M = 10;

tol = 0.000001;

controller_index = 2; % 1->cartesian, 2->posture
flg_replanning = true;
% desired configuration
q_d = [0;0;0];
% initial configuration
Q_INIT = [0;-2;pi/2+0.01];
% simulation time
T_SIM = 60;

%% Set controller parameters
if controller_index == 1
    % cartesian
    k_1 = 0.5; 
    k_2 = 1;
    control_par = [k_1, k_2, 0];
else
    % posture
    %k_1 =0.80; 
    %k_2 = 0.75;
    %k_3 = 0.3;
    k_1 = 0.8;
    k_2 = 0.75;
    k_3 = 0.3;
    control_par = [k_1, k_2, k_3];
end

%%
sim("sim_regulation.slx");