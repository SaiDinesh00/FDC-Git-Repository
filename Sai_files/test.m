clc; clear;
close all

parameters;
X_init = [0; 0; 0; 0; 0; 0; 1; 0; 0; 0; 0; 0; 0; 0.004; 0.004; 0.004; 0.004];

X(:, 1) = X_init;
Xd = [0, 0, -4, pi/10]; % [x y z psi]


dt = 0.001;
t = 0:dt:2.5;

pos_ddot = [0; 0; 182];
psi_des = pi/10;
neta(:, 1) = X(7:10, 1);
for k = 1:length(t)-1
    pos_ddot = Translational_error_dynamics(X(:, k), Xd);
    [Td, outer_sol] = outer_loop_control(Xd, pos_ddot);
    tau_des = inner_loop(X(:, k),outer_sol);
    [CT_dot] = control_allocation(X(:,k), Td, tau_des);
    X(:, k+1) = state(X(:, k), CT_dot, dt);
    t(:, k+1) = t(:, k) + dt;
    neta(:, k+1) = X(7:10, k+1);
end

% plot(t(1:500), T, LineStyle="-")
% hold on
% plot(t, T_des*ones(length(t)), LineStyle="--")
% legend('T', 'T_des')
% 
% figure
% plot(t(1:500), L, LineStyle="-")
% hold on
% plot(t, tau_des(1)*ones(length(t)), LineStyle="--")
% legend('L', 'L_des')
% 
% figure
% plot(t(1:500), M, LineStyle="-")
% hold on
% plot(t, tau_des(2)*ones(length(t)), LineStyle="--")
% legend('M', 'M_des')
% 
% figure
% plot(t(1:500), N, LineStyle="-")
% hold on
% plot(t, tau_des(3)*ones(length(t)), LineStyle="--")
% legend('N', 'N_des')


