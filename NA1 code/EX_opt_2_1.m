% Numerical Activity 1 (NA1): Simulation and Trajectory Planning
% MOTION PLANNING - Optional 2_1

clear all;
close all;
addpath(genpath(fullfile(pwd,'..','utils')));
  
%% test localization strategies based on Kalman filter with GPS data

% displacement params r and d 
% p_loss = 0.99 / 0.9
% Ts = 0.1, 0.04, 0.99

%% Load Data

load("NA2_Full_Results.mat")

