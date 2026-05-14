%% verificare qualità kalman
addpath('../Functions');


z_EKF = squeeze(out.z_EKF.signals.values);
P_EKF = out.P_filt_EKF.signals.values;

analyze_EKF_results(z_EKF,P_EKF,q_model_lab,5);
plot_EKF_covariance_evolution(P_EKF,6);