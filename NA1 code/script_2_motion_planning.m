%% Numerical Activity 1 (NA1): Simulation and Trajectory Planning
%% PART 2: MOTION PLANNING

clear all;
close all;
addpath(genpath(fullfile(pwd,'..','utils')));
  
%% Set simulation parameters
T_SIM = 10;
T_s = 0.01;

%% 2.1 CARTISIAN trajetory

qi = [-1; -1; pi/2];
qf = [1; 1; pi/2];

ki = 5; 
kf = 5;

%% Run traj - example of function (run_trajectory_cartisian)
% Time law
t = 0:T_s:T_SIM;
[s, s_dot] = time_law_constant(t, T_SIM);

% Traj generation
[v,w] = trajectory_plan_cartisian(qi, qf, ki, kf, s,s_dot);

% State simulation
q = simulate_unicycle(qi, v, w, T_s);

% Plot functions
plot_unicycle_2D(q,50);
plot_unicycle_wrt_time(q, T_s);

%% Re-Orienting 
qi = [1; 1; -pi/3];
qf = [1; 1; pi/3];

q = run_trajectory_cartisian(qi, qf, ki, kf, T_SIM,T_s);


%% in the origin
qi = [0; 0; -pi/3];
qf = [0; 0; pi/3];

q = run_trajectory_cartisian(qi, qf, ki, kf, T_SIM,T_s);

% ANALYSIS: no problem with the origin; difficulties with big changes
% (example -pi/2 to pi/2) 
% It can't rotate on itself (x,y not zero)

%% 2.2 CHAINED trajectory

qi = [0; 0; -pi/3];
qf = [1; 1; pi/3];

q = run_trajectory_chained(qi, qf, T_SIM,T_s, 0);

%% 
qi = [0; 0; pi + 2/3*pi];
qf = [1; 1; pi/3];

q = run_trajectory_chained(qi, qf, T_SIM,T_s, 0);

%% 
qi = [1; 1; -pi/3];
qf = [2; 2; pi/3];

q = run_trajectory_chained(qi, qf, T_SIM,T_s, 0);


% ANALYSIS: same traj for 1 and 3 (but translated)
% completely different traj with 2 (other angle definition)
% in general no straight line

%% Re-Orienting 
qi = [1; 1; -pi/2];
qf = [1; 1; pi/2];

q = run_trajectory_chained(qi, qf, T_SIM,T_s, 0);

%% in the origin
qi = [0; 0; -pi/2];
qf = [0; 0; pi/2];

q = run_trajectory_chained(qi, qf, T_SIM,T_s, 0);

% ANALYSIS: in the orgin perfect rotation on the spot, in an other
% configuration not so...

%% 2.2.5 Traslation of the reference frame

qi = [1; 1; -pi/2];
qf = [1; 1; pi/2];

q = run_trajectory_chained(qi, qf, T_SIM,T_s ,1);

%% in the origin

qi = [1; 1; -pi/2];
qf = [1; 1; pi/2];

q = run_trajectory_chained(qi, qf, T_SIM,T_s ,1);

% now it works in both origin and not configuratio



%% RUN OPTIONAL 2
%% Set simulation parameters
T_SIM = 23.0;  % 23 sec limit value to not overcome 15 rad/s
T_s = 0.01;
r = 0.03;
d = 0.165;

%% 2.1 CARTISIAN trajetory

qi = [-1; -1; pi/2];
qf = [1; 1; pi/2];

ki = 4.5;    % 4.5 limit value to not hit the obstacle
kf = 10;

%%
[q, u] = run_trajectory_cartisian(qi, qf, ki, kf, T_SIM,T_s);

[q_dot, omega_wheels] = DDR_Unicycle(q, u, d, r);

plot_wheels_speed(omega_wheels, T_s);