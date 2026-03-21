clear all;
close all;
clc;

%% Set simulation parameters
r = 0.03;
d = 0.165;
T_SIMULATION = 1;
Q_INITIAL = [1;1;0];

%% Linear and angular velocity
v = 0; 
omega = 2;

%%
open_system('EX_opt_1_model.slx');
load_system('EX_opt_1_model.slx');
out = sim('EX_opt_1_model.slx',T_SIMULATION);

%%

z_chained = out.z.signals.values;
q = out.q.signals.values;
v_chained = out.v_chained.signals.values;
%% 
N = size(q, 3);

z_prod = zeros(3,1,N);

for i = 1:N
    qi = q(:,:,i);          % 3x1
    theta = qi(3);

    T = [0 0 1;
         cos(theta) sin(theta) 0;
         sin(theta) -cos(theta) 0];

    z_prod(:,:,i) = T * qi;
end
%% just a check to see if the transformation has been done properly

t = 1:N;

figure

% --- z_prod ---
subplot(1,2,1)
hold on
plot(t, squeeze(z_prod(1,1,:)), 'LineWidth', 2)
plot(t, squeeze(z_prod(2,1,:)), 'LineWidth', 2)
plot(t, squeeze(z_prod(3,1,:)), 'LineWidth', 2)
title('z\_prod')
legend('z1','z2','z3')
grid on
xlim([0 1000])
ylim([-1 2])
xticks([0 1000])
xticklabels({'0','1 sec'})

% --- z_chained ---
subplot(1,2,2)
hold on
plot(t, squeeze(z_chained(1,1,:)), 'LineWidth', 2)
plot(t, squeeze(z_chained(2,1,:)), 'LineWidth', 2)
plot(t, squeeze(z_chained(3,1,:)), 'LineWidth', 2)
title('z\_chained')
legend('z1','z2','z3')
grid on
xlim([0 1000])
ylim([-1 2])
xticks([0 1000])
xticklabels({'0','1 sec'})

%%

u = zeros(2,N);
u(1,:) = v;
u(2,:) = omega;
figure

% --- q(t) ---
subplot(2,2,1)
hold on
plot(t, squeeze(q(1,1,:)), 'LineWidth', 2)
plot(t, squeeze(q(2,1,:)), 'LineWidth', 2)
plot(t, squeeze(q(3,1,:)), 'LineWidth', 2)
title('q(t)')
legend('x','y','\theta')
grid on
xlim([0 1000])
ylim([0 2])

% --- u(t) ---
subplot(2,2,3)
hold on
plot(t, u(1,:), 'LineWidth', 2)
plot(t, u(2,:), 'LineWidth', 2)
title('u(t)')
legend('v','\omega')
grid on
xlim([0 1000])
ylim([-0.5 2.5])

% --- z(t) ---
subplot(2,2,2)
hold on
plot(t, squeeze(z_chained(1,1,:)), 'LineWidth', 2)
plot(t, squeeze(z_chained(2,1,:)), 'LineWidth', 2)
plot(t, squeeze(z_chained(3,1,:)), 'LineWidth', 2)
title('z(t)')
legend('z1','z2','z3')
grid on
xlim([0 1000])
ylim([-1 2])

% --- v_chained(t) ---
subplot(2,2,4)
hold on
plot(t, squeeze(v_chained(1,1,:)), 'LineWidth', 2)
plot(t, squeeze(v_chained(2,1,:)), 'LineWidth', 2)
title('v(t) chained')
legend('v1','v2')
grid on
xlim([0 1000])
ylim([-3 2.5])

