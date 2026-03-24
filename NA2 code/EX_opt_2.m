clear all;
close all;
addpath(genpath('utils'));

%% OPTIONAL 2

% test localization strategies based on Kalman filter without GPS data
% but provided distance robot-origin

% displacement params r and d 
% no data loss
% Ts = 0.04

%% Load Data

load("NA2_Full_Results.mat")

%% Plot p_loss = 0.90 

Ts_values = [0.1, 0.04, 0.99];

for i = 1:length(Ts_values)
    T_s = Ts_values(i);
    
    plot_EKF_results(opt2_data(i).q_actual, ...
                     opt2_data(i).q_loc_exact, ...
                     opt2_data(i).z_estimate);
    title(['EKF GPS: T_s = ', num2str(T_s), ' | p_{loss} = ', num2str(p_loss)]);
end

