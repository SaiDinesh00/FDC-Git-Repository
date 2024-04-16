clear all;
close all;
clc;

%% Time information

count   = 0;                                % Counting number of iterations
t0      = 60;                                % Initial time in seconds
tf      = 90;                               % Final time in seconds
dt      = 0.01;                             % time step
N       = floor((tf-t0)/dt) + 1;            % No. of iterations
t       = zeros(N, 1);                      % Time vector initialization
t(1)    = t0;

Initial_conditions;

%% Iteration begins

for k = 1:N-1
    
    
    %% Desired trajectory for the fixed wing
    [psi_d(1, k), Xd(1:3, k+1), Xd_dot(1:3, k)] = Desired_trajectory(t(k), Xd(1:3, k), dt, V_w_d);

    %% Control design   
    if k == 1
        aa = k;
    else
        aa = k-1;
    end
    [U(:, k), F_M_quad(:, k), F_M_wing(:, k), u_des, Qd, Quaterd_dot, w_d, w_d_dot, Euler_angles_d]  = control(X(:, k), X_dot(:, k) , Xd(:, aa:k), Xd_dot(:, k), psi_d(k), dt);
    Xd(8, k+1) = u_des;
    Xd(4:7, k+1) = Qd;
    Xd(11:13, k+1) = w_d;
    Xd_dot(8, k+1) = (Xd(8, k+1) - Xd(8, k)) * dt;
    Xd_dot(4:7, k+1) = Quaterd_dot;
    Xd_dot(11:13, k+1) = w_d_dot;
    Euler_angles_des(:, k) = Euler_angles_d;

    [X(:,k+1), X_dot(:, k+1)] = state(X(:,k), U(:,k), F_M_wing(:, k), dt);

    % Quaternion adjustments
    X(4:7, k+1) = X(4:7, k+1) / norm(X(4:7, k+1));

    Thrust_coeffs = X(14:17, k+1);
    Thrust_coeffs(Thrust_coeffs > 0.6) = 0.6;
    Thrust_coeffs(Thrust_coeffs < 0.000001) = 0.000001;
    X(14:17, k+1) = Thrust_coeffs;

    t(k+1) = t(k) + dt; 
    count = count + 1
    
%     if count > 114
%         break
%     end
  
end
Plotting;

    
    
    
    
