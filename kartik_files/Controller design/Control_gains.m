parameters;
%% Throttle gains

K_u = 2;
T_max = 400;  %Newtons

%% FW inner loop

kh = 0.15;
kb = 0.8;

%% Quaternion error dynamics
zeta_q = [0.95; 35; 0.95; 35];
omega_n_q = 1 * [3; 0.5; 3; 0.5];
zeta_quat = eye(4) * zeta_q;
omega_n_quat = eye(4) * omega_n_q;

%% Control allocation
K_T = 10;
K_L = 35;  % 35
K_M = 120;  % 120
K_N = 35;  % 35