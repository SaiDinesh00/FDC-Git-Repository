clc, close all
clear

Initial_state = zeros(17,1);
Initial_state(7:10) = eul2quat([0, 0, 0],"ZYX");
Initial_state(14:17) = 0.004;
X(:,1) = Initial_state;
dt = 0.01;

%% Desired State

Xd = [0, 0, -15, 0, 0, 0, 0, 0, 0, 0, 0, 0]; % Waypoint Desired [xd, yd, zd, ud, vd, wd, phi_d, theta_d, psi_d, pd, qd, rd]
for i = 1:1e3-1
    pos_ddot = translational_dynamics(X, Xd);
    [neta_des, T_des] = outer_loop_control(Xd, pos_ddot);
    tau_des = inner_loop(X(:, i), Xd, neta_des);
    U(:, i) = control_allocation(X(:, i),T_des, tau_des);
    X(:, i+1) = X(:, i) + dt .* dynamics2(X(:, i), U(:, i));
end
plot(X(3,:))