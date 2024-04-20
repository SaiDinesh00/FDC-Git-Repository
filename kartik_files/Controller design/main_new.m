clear all;
% close all;
% clc;

%% Time information

count   = 0;                                % Counting number of iterations
t0      = 60;                                % Initial time in seconds
tf      = 70;                               % Final time in seconds
dt      = 0.01;                             % time step
N       = floor((tf-t0)/dt) + 1;            % No. of iterations
t       = zeros(N, 1);                      % Time vector initialization
t(1)    = t0;
count   = 0;

Initial_conditions_new;

%% Iteration begins

for k = 1:N-1
    
    
    % %% Desired trajectory for the fixed wing
    % [psi_d(1, k), Xd(1:3, k+1), Xd_dot(1:3, k)] = Desired_trajectory(t(k), Xd(1:3, k), dt, V_w_d);
   T_Quad_des = 0;
   M_wing = [1; 2; 3];
   M_Quad_des = Inner_loop_new(X(:, k), Xd(:, k), M_wing);
   F_M_Quad_des(:, k) = [T_Quad_des; M_Quad_des];

   U(:, k+1) = Control_input_new(F_M_Quad_des(:, k), U(:, k), dt);

   X_dot(:, k) = Biplane_6DOF_new(X(:, k), U(:, k+1));

   X(:, k+1) = X(:, k) + X_dot(:, k) * dt;

   X(4:7, k+1) = X(4:7, k+1) / norm(X(4:7, k+1));

   t(k+1) = t(k) + dt;

   count = count + 1;
   disp(count)
end
Plotting;