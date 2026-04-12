%% Experimental Activity 1 (EA1): Planning, Localization, and Identification
%% PART 3

clear all;
close all;
addpath(fullfile(pwd,'..','utils'));
 
%% Set simulation parameters
T_s = 0.04; 
r = 0.03;
d = 0.165;
omega_max = 10;

%% Trajectory

qi = [-0.5; -0.5; pi/2];
qf = [0.5; 0.5; pi/2];

ki = 5;
kf = 5;

%% tune time law: change k or Tc??

Ta = 1;
Tc = 10;
T_SIM = 2*Ta + Tc;

%%
%[q, u] = run_trajectory_cartisian(qi, qf, ki, kf, T_SIM,T_s);
% here i have to change the time law

    % Time law
    t = 0:T_s:T_SIM;
    [s, s_dot] = time_law_trapezoidal(t, Ta, Tc);
    
    % Traj generation
    [v,w] = trajectory_plan_cartisian(qi, qf, ki, kf, s,s_dot);
    
    % State simulation
    q = simulate_unicycle(qi, v, w, T_s);
    u = [v;w];
    
    % Plot functions for 2D and time
    plot_unicycle_2D(q,50)
    plot_unicycle_wrt_time(q, u, T_s);

[q_dot, omega_wheels] = DDR_Unicycle(q, u, d, r);

plot_wheels_speed(omega_wheels, T_s);

fprintf('max wheel speed: %.3f rad/s\n', max(abs(omega_wheels(:))))


%% Simulation

q_test = q;
u_test = u;