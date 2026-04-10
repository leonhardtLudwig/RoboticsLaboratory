%% Experimental Activity 1 (EA1): Planning, Localization, and Identification
%% PART 2

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

%% IDENTIFICATION / CALIBRATION



%% Set matrix H for EKF

%Encoder
H1 = [0, 0, 0, 1, 0, 0, 0;
      0, 0, 0, 0, 1, 0, 0];

%Angular velocities
H2 = [0, 0, 0, 0, 0, 1, 0;
      0, 0, 0, 0, 0, 0, 1]; 
%Configuration
H3 = [1, 0, 0, 0, 0, 0, 0;
      0, 1, 0, 0, 0, 0, 0;
      0, 0, 1, 0, 0, 0, 0]; 

H = H1;
%H = [H2;H1];