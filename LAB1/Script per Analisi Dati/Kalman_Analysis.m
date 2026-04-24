close all;
clc;

%  i = 1 => ploss = 1
%  i = 2 => ploss = 0.99
%  i = 3 => ploss = 0

i = 3;

P = results_part3(i).out_backup.P_filt_EKF.signals.values; % 7x7x301

T = size(P,3);
n = size(P,1);

%% ===== FIGURA 1: VARIANZE (DIAGONALE) =====
P_diag = zeros(n,T);

for k = 1:T
    P_diag(:,k) = diag(P(:,:,k));
end

figure
plot(P_diag')
xlabel('time step')
ylabel('variance')
legend('x1','x2','x3','x4','x5','x6','x7')
grid on
title('EKF Covariance Diagonal')

%% ===== FIGURA 2: NORMA DELLA COVARIANZA =====
P_norm = zeros(1,T);

for k = 1:T
    P_norm(k) = norm(P(:,:,k),'fro');
end

figure
plot(P_norm)
xlabel('time step')
ylabel('||P||_F')
grid on
title('EKF Covariance Norm')

%% eigenvalues 
figure
eigvals = zeros(7,T);

for k = 1:T
    eigvals(:,k) = eig(P(:,:,k));
end

plot(eigvals')
xlabel('time step')
ylabel('eigenvalues')
grid on
legend('x1','x2','x3','x4','x5','x6','x7')
grid on