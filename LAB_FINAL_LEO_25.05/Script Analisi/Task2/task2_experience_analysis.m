clear all;
close all;
clc;
addpath(genpath(fullfile(pwd,'..','utils')));
addpath('../../Functions');
out1 = load('/Users/llp/Desktop/RoboLab_Report/LAB_FINAL_LEO_19.05/TASK2_DATI_19_05/out_TSIM_15.mat');
out2 = load('/Users/llp/Desktop/RoboLab_Report/LAB_FINAL_LEO_19.05/TASK2_DATI_19_05/out_TSIM_20.mat');

%% exp1

exp1 = create_tracking_exp_struct(out1.out, 15);
plot_all_tracking_experiment_data(exp1, 'Experiment 1: NL Tracking', 1);

%% exp2

exp2 = create_tracking_exp_struct(out2.out, 20);
plot_all_tracking_experiment_data(exp2, 'Experiment 2: NL Tracking', 2);


%%

all_exp = {exp1, exp2};
label_exp = {'1','2'};

compare_multiple_tracking_experiments(all_exp,label_exp,11);