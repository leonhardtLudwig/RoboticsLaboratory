%% PARTE 3

clear all;
close all;
%addpath(genpath('utils'));
addpath(fullfile(pwd,'..','utils'));
  
%% Set simulation parameters

T_SIM = 20;  % tentativo

T_s = 0.04; 
r = 0.03;
d = 0.165;
omega_max = 10;

d = 0.18428;
r = 0.03316;

%% Time Law params

%trapezoidal
Ta = 2;
Tc = 3;

%% Trajectory params
qi = [-0.5; -0.5; pi/2];
qf = [0.5; 0.5; pi/2];

ki = 5;
kf = 5;



%% time law costante


% Traj generation
ki = 1;
kf = 6.5;
T_SIM = 20;

t = 0:T_s:T_SIM;
[s, s_dot] = time_law_constant(t, T_SIM);

[v,w] = trajectory_plan_cartisian(qi, qf, ki, kf, s,s_dot);
% State simulation
q = simulate_unicycle(qi, v, w, T_s);
[~, omega_wheels] = DDR_Unicycle(q, [v; w], d, r);
plot_wheels_speed(omega_wheels, T_s);
fprintf('max wheel speed: %.3f rad/s\n', max(abs(omega_wheels(:))))
% itera fintanto che max(|omega_wheels|)<=omega_max
% su time-law costante dipende da kf e T_SIM 
% se T_SIM FISSO allora max(|omega_wheels|) cresce al crescere di kf
% se kf FISSO allora max(|omega_wheels|) cresce al decrescere di T_SIM


% Plot functions
figure;
plot_unicycle_2D(q,50);
%% time law trapezoidale

ki = 1;
kf = 6;
Ta = 1;
Tc = 18;

[s, s_dot] = time_law_trapezoidal(t, Ta,Tc);
% Traj generation
[v,w] = trajectory_plan_cartisian(qi, qf, ki, kf, s,s_dot);

% State simulation
q = simulate_unicycle(qi, v, w, T_s);
[~, omega_wheels] = DDR_Unicycle(q, [v; w], d, r);
plot_wheels_speed(omega_wheels, T_s);
fprintf('max wheel speed: %.3f rad/s\n', max(abs(omega_wheels(:))))

% Plot functions
figure;
plot_unicycle_2D(q,50);

%% prova usandro run_trajectory
%figure;
%[q, u] = run_trajectory_cartisian(qi, qf, ki, kf, T_SIM,T_s);
 
%[q_dot, omega_wheels] = DDR_Unicycle(q, u, d, r);

%plot_wheels_speed(omega_wheels, T_s);
%fprintf('max wheel speed: %.3f rad/s\n', max(abs(omega_wheels(:))))

%%


