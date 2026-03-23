clear all;
close all;
addpath(genpath('utils'));

%% 

T_SIMULATION = 1;
ki = 10;
kf = 10;

T_s = 1e-3;
t = 0:T_s:T_SIMULATION;

qi = [-1;-1;pi/2];
qf = [1;1;pi/2];

N = length(t);

s = zeros(size(t));
s_dot = zeros(size(t));
v = zeros(size(t));
omega = zeros(size(t));

for i = 1:N
    [s(i), s_dot(i)] = time_law_constant(t(i), T_SIMULATION);
    [v(i),omega(i)]= trajectory_plan_cartesian(qi,qf,ki,kf,s(i),s_dot(i));
end

q = simulate_unicycle(qi, v, omega, T_s);
%% 
plot_unicycle_2D(q,50);


