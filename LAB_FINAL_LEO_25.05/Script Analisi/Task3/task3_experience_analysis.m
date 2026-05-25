clear all;
close all;
clc;
addpath(genpath(fullfile(pwd,'..','utils')));
addpath('../../Functions');
out1 = load('/Users/llp/Desktop/RoboLab_Report/LAB_FINAL_LEO_19.05/TASK3_DATI/test_ctrl1.mat');
out3 = load('/Users/llp/Desktop/RoboLab_Report/LAB_FINAL_LEO_19.05/TASK3_DATI/test_ctrl3_sbagliati.mat');
out4 = load('/Users/llp/Desktop/RoboLab_Report/LAB_FINAL_LEO_19.05/TASK3_DATI/test_ctrl4.mat');

%% exp1

exp1 = create_tracking_exp_struct(out1.res.out_backup, 25);
plot_all_tracking_experiment_data(exp1, 'Experiment 1: NL Tracking', 1);

%% exp3

exp3 = create_tracking_exp_struct(out3.res.out_backup, 25);
plot_all_tracking_experiment_data(exp3, 'Experiment 3: NL Tracking', 2);

%% exp4

exp4 = create_tracking_exp_struct(out4.res.out_backup, 25);
plot_all_tracking_experiment_data(exp4, 'Experiment 4: NL Tracking', 3);



%%

all_exp = {exp1, exp3, exp4};
label_exp = {'1','3', '4'};

compare_multiple_tracking_experiments(all_exp,label_exp,16);