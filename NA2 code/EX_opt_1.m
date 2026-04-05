%% Numerical Activity 2 (NA2): Localization and Identification

clear all;
close all;
addpath(genpath(fullfile(pwd,'..','utils')));

%% OPTIONAL 1

% test localization strategies based on Kalman filter with GPS data

% displacement params r and d 
% p_loss = 0.99 / 0.9
% Ts = 0.1, 0.04, 0.99

%% Load Data

load("NA2_Full_Results.mat")

%% Plot p_loss = 0.90 

Ts_values = [0.1, 0.04, 0.001];
p_loss = 0.90; 

for i = 1:length(Ts_values)
    T_s = Ts_values(i);
    
    plot_EKF_results(opt1_data_p90(i).q_actual, ...
                     opt1_data_p90(i).q_loc_exact, ...
                     opt1_data_p90(i).z_estimate);
    title(['EKF GPS: T_s = ', num2str(T_s), ' | p_{loss} = ', num2str(p_loss)]);
    
end

% p_loss = 0.99; 
% 
% for i = 1:length(Ts_values)
%     T_s = Ts_values(i);
% 
%     plot_EKF_results(opt1_data_p99(i).q_actual, ...
%                      opt1_data_p99(i).q_loc_exact, ...
%                      opt1_data_p99(i).z_estimate);
%     title(['EKF GPS: T_s = ', num2str(T_s), ' | p_{loss} = ', num2str(p_loss)]);
% end

%%



plot_unicycle_2D_EKF_filter(opt1_data_p90(1).q_actual, ...
                            opt1_data_p90(1).z_estimate, ...
                            opt1_data_p90(1).P_filt_current,...
                            50);