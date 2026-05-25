clear all;
close all;
clc;
addpath(genpath(fullfile(pwd,'..','utils')));
addpath('./Functions');
out1 = load('/Users/llp/Desktop/RoboLab_Report/LAB_FINAL_LEO_19.05/TASK1_DATI_19_05/out_sim1.mat');
out2 = load('/Users/llp/Desktop/RoboLab_Report/LAB_FINAL_LEO_19.05/TASK1_DATI_19_05/out_sim2.mat');
out3 = load('/Users/llp/Desktop/RoboLab_Report/LAB_FINAL_LEO_19.05/TASK1_DATI_19_05/out_sim3.mat');
out4 = load('/Users/llp/Desktop/RoboLab_Report/LAB_FINAL_LEO_19.05/TASK1_DATI_19_05/out_sim4.mat');
%%

q_EKF1 = squeeze(out1.out.q_EKF.signals.values);
q_EKF1 = q_EKF1(:,1:376);
ws_meas_1 = out1.out.ws_meas.signals.values(1:376,:);

q_EKF2 = squeeze(out2.out.q_EKF.signals.values);
q_EKF2 = q_EKF2(:,1:376);
ws_meas_2 = out2.out.ws_meas.signals.values(1:376,:);

q_EKF3 = squeeze(out3.out.q_EKF.signals.values);
q_EKF3 = q_EKF3(:,1:376);
ws_meas_3 = out3.out.ws_meas.signals.values(1:376,:);

q_EKF4 = squeeze(out4.out.q_EKF.signals.values);
q_EKF4 = q_EKF4(:,1:376);
ws_meas_4 = out4.out.ws_meas.signals.values(1:376,:);

q_model1 = squeeze(out1.out.q_model.signals.values);
q_model1 = q_model1(:,1:376);
ws_des_1 = squeeze(out1.out.ws_des.signals.values)';
ws_des_1 = ws_des_1(1:376,:);

q_model2 = squeeze(out2.out.q_model.signals.values);
q_model2 = q_model2(:,1:376);
ws_des_2 = squeeze(out2.out.ws_des.signals.values)';
ws_des_2 = ws_des_2(1:376,:);

q_model3 = squeeze(out3.out.q_model.signals.values);
q_model3 = q_model3(:,1:376);
ws_des_3 = squeeze(out3.out.ws_des.signals.values)';
ws_des_3 = ws_des_3(1:376,:);

q_model4 = squeeze(out4.out.q_model.signals.values);
q_model4 = q_model4(:,1:376);
ws_des_4 = squeeze(out4.out.ws_des.signals.values)';
ws_des_4 = ws_des_4(1:376,:);


%%
labels_ekf = {'q-ekf-1', 'q-ekf-2', 'q-ekf-3', 'q-ekf-4'};

labels_model= {'q-model-1', 'q-model-2', 'q-model-3', 'q-model-4'};

labels_gt = {'gt1', 'gt2', 'gt3', 'gt4'};


plot_4_unicycle_trajectories(q_EKF1,q_EKF2,q_EKF3,q_EKF4,labels_ekf,'Traj',1);
plot_4_unicycle_orientation_error(q_EKF1,q_EKF2,q_EKF3,q_EKF4,labels_ekf,'Orientation Error',2);
plot_4_wheel_velocities(ws_meas_1,ws_meas_2,ws_meas_3,ws_meas_4,labels_ekf,'Wheels speed',3);


plot_4_unicycle_trajectories(q_model1,q_model2,q_model3,q_model4,labels_model,'Traj',4);
plot_4_unicycle_orientation_error(q_model1,q_model2,q_model3,q_model4,labels_model,'Orientation Error',5);
plot_4_wheel_velocities(ws_des_1,ws_des_2,ws_des_3,ws_des_4,labels_model,'Wheels speed',6);

%%
gt1 = out1.out.vicon_gt.signals.values(1:376,:)';
gt2 = out2.out.vicon_gt.signals.values(1:376,:)';
gt3 = out3.out.vicon_gt.signals.values(1:376,:)';
gt4 = out4.out.vicon_gt.signals.values(1:376,:)';

plot_4_unicycle_trajectories(gt1,gt2,gt3,gt4,labels_gt,'Traj',7);
plot_4_unicycle_orientation_error(gt1,gt2,gt3,gt4,labels_gt,'Orientation Error',8);

