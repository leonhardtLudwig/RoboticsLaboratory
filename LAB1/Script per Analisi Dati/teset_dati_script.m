%% Experimental Activity 1 (EA1): Planning, Localization, and Identification
%% PART 3

clear all;
close all;
addpath(genpath(fullfile(pwd,'..','utils')));

%% Set simulation parameters
T_s = 0.04; 
r = 0.03;
d = 0.165;
omega_max = 10;

%% Load Data from Part1 and extract configuration 1

load('EA1_Part1_Data.mat');

results = results_part1(1);
disp(['Results with Ta, Tc: ', num2str(results.Ta), ', ', num2str(results.Tc)]);

T_SIM = 2*results.Ta + results.Tc;


%% 
% tiro fuori i dati per comodità 
q_des = results.q_desired;
q_motion_capture = results.q_motion_capture;
gyro = results.gyro;
acce = results.acce;
ws_des = results.wheels_speed_desired;
ws_meas = results.wheels_speed_measured;

% motion capture has y and z axis opposite
q_motion_capture(:, 2) = -results.q_motion_capture(:, 2); 
q_motion_capture(:, 3) = -results.q_motion_capture(:, 3);



plot_unicycle_2D(q_loc_kalman,50);

plot_unicycle_2D(q_motion_capture,50);

plot_EKF_results(q_motion_capture, q_loc_exact, q_loc_EKF);