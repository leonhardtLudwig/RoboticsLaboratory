clear all;
close all;
clc;

%% Set simulation parameters
r = 0.03;
d = 0.165;
T_SIMULATION = 1;
Q_INITIAL = [1;1;0];

%% Linear and angular velocity
v = 0; 
omega = 2;

%%
out = sim('EX_opt_1_model.slx',T_SIMULATION);

%%

z = out.z.signals.values;
q = out.q.signals.values;