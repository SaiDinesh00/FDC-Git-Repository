clc, close all
clear

parameters
Initial_state = zeros(17, 1);
Initial_state(7:10) = eul2quat([0, 0, 0], "ZYX");
Initial_state(14:17) = CT_init;
X(:, 1) = Initial_state;

t0 = 0;
tf = 5; % seconds
dt = 0.01; % time step
N = floor((tf - t0) / dt) + 1;
t(1, :) = 0;

U = zeros(4, N - 1);

%% Desired State

Xd = [0, 0, -5, 0, 0, 0, 0, 0, 0, 0, 0, 0]; % Waypoint Desired [xd, yd, zd, ud, vd, wd, phi_d, theta_d, psi_d, pd, qd, rd]
for k = 1:N-1
    
    pos_ddot = translational_dynamics(X(:, k), Xd);
    [neta_des, T_des] = outer_loop_control(Xd, pos_ddot);
    tau_des = inner_loop(X(:, k), neta_des');
    U(:, k) =  control_allocation(X(:, k), T_des, tau_des);
    pos_err(:, k) = (-5 - X(3,k));
    % 
    X(:, k+1) = state(X(:, k), U(:, k), dt);
    t(:, k+1) = t(:, k) + dt;
    % eul_ang(:, i) = quat2eul([X(7,k), X(8,k), X(9,k), X(10,k)]);

    % X(:, k+1) = X(:, k) + dt .* dynamics2(X(:, k), U(:, k));
    
end

plot(X(3,:))
figure
plot(X(1,:))
