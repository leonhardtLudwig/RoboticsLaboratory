clear all;
close all;
clc;
addpath(genpath(fullfile(pwd,'..','utils')));
addpath('../../Functions');
out1 = load('/Users/llp/Desktop/RoboLab_Report/LAB_FINAL_LEO_19.05/TASK1_DATI_19_05/out_sim1.mat');
out2 = load('/Users/llp/Desktop/RoboLab_Report/LAB_FINAL_LEO_19.05/TASK1_DATI_19_05/out_sim2.mat');
out3 = load('/Users/llp/Desktop/RoboLab_Report/LAB_FINAL_LEO_19.05/TASK1_DATI_19_05/out_sim3.mat');
out4 = load('/Users/llp/Desktop/RoboLab_Report/LAB_FINAL_LEO_19.05/TASK1_DATI_19_05/out_sim4.mat');

%% exp1

exp1 = create_regulation_exp_struct(out1.out, 15);
plot_all_regulation_experiment_data(exp1, 'Experiment 1: Base Regulation', 1);

%% exp2

exp2 = create_regulation_exp_struct(out2.out, 15);
plot_all_regulation_experiment_data(exp2, 'Experiment 2: Base Regulation', 2);

%% exp3

exp3 = create_regulation_exp_struct(out3.out, 15);
plot_all_regulation_experiment_data(exp3, 'Experiment 3: Base Regulation', 3);

%% exp4

exp4 = create_regulation_exp_struct(out4.out, 15);
plot_all_regulation_experiment_data(exp4, 'Experiment 4: Base Regulation', 4);

%%

all_exp = {exp1, exp2, exp3, exp4};
label_exp = {'1','2','3','4'};

compare_multiple_regulation_experiments(all_exp,label_exp,25);