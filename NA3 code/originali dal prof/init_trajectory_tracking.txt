clear all;
close all;

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

controller_index = 3; % 1->lin, 2->nonlin, 3->FL
trj_index = 4;% 1->line, 2->circle, 3->square, 4->8-shaped

% --- line (1/T_trj [m/s] along the Y axis)
if trj_index == 1
    Q_INIT = [1; 0.; pi/2];
    T_trj = 2;
    T_SIM = 10;
elseif trj_index == 2
    % --- circle (radius 0.5 [m] with angular vel 2*pi/T_trj)
    Q_INIT = [0.1; -0.2; pi/2];
    T_trj = 5;
    T_SIM = 10;
elseif trj_index == 3
    % --- square (side_length 1 [m] with linear velocity side_length/(T_trj/4)[m/s])
    Q_INIT = [0.1; 0.; pi/2];
    T_trj = 20;
    T_SIM = 40;
else 
    % --- 8-shape (R=0.4 [m] with period T_trj)
    Q_INIT = [.2; 0.; pi/2];
    T_trj = 18;
    T_SIM = T_trj*4;
end

%% Set controller parameters
if controller_index == 1
    % linear
    xi = 0.9; 
    a = 5;
    control_par = [xi, a, 0];
elseif controller_index ==2
    % nonlinear
    xi = 0.9; 
    b = 25;
    control_par = [xi, b, 0];
elseif controller_index ==3
    % feedback_linearization
    k1 = 10; 
    k2 = 10;
    b = 0.01;
    control_par = [k1, k2,b];
end