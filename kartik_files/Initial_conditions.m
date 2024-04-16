
parameters;
V_w_d = 15;

% Initial conditions 
X           = zeros(17, N);
X(3, 1)     = -190;
X(4, 1)     = 1;
X_dot       = zeros(17, N);
Xd          = zeros(17, N);
Euler_angles_des = zeros(3, N-1);
Xd(3, 1)    = -200; 
Xd(4, 1)    = 1;
Xd_dot      = zeros(17, N);
psi_d       = zeros(1, N);
F_M_quad    = zeros(4, N-1);
F_M_wing    = zeros(6, N-1);
U           = zeros(4, N-1);

% Initial_thrust_coefficients
ct_1 = CT_init;
ct_2 = CT_init;
ct_3 = CT_init;
ct_4 = CT_init;
X(14:17, 1) = [ct_1; ct_2; ct_3; ct_4];

        
