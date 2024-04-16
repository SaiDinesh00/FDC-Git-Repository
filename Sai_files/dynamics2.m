function X_dot = dynamics2(X, U)

% k_t = 1e-3; % Thrust Coefficient

%% Parameters
mass = 12; % Kg
g = 9.81;
R      = 0.18;
l      = 0.3;
K      = 820;


I = diag([1.86, 2.031, 3.617]);
I_x = I(1,1);
I_y = I(2,2);
I_z = I(3,3);
I_xz = 0;


%% State Unpacking
% w1 = u(1);
% w2 = u(2);
% w3 = u(3);
% w4 = u(4);



% Linear Velocity
u = X(4);
v = X(5);
w = X(6);

% Orientation In Quaternion
n0 = X(7);
n1 = X(8);
n2 = X(9);
n3 = X(10);

% Angular Velocity
p = X(11);
q = X(12);
r = X(13);

% Thrust Coefficients
ct1 = X(14);
ct2 = X(15);
ct3 = X(16);
ct4 = X(17);

%% Aero Dynamic Forces (TBD)
F_ax = 0;
F_ay = 0;
F_az = 0;

L_abar = 0;
% L_tbar = 0;
M_a = 0;
% M_t = 0;
N_a = 0;
% N_t = 0;


%% Constants
C1 = (1/(I_x*I_z - I_xz)) .* [(I_y - I_z)*I_z - I_xz^2;
                             (I_x - I_y + I_z)*I_xz;
                             I_z;
                             I_xz;
                             (I_x - I_y)*I_x + I_xz^2;
                             I_x];
c1 = C1(1);
c2 = C1(2);
c3 = C1(3);
c4 = C1(4);
c8 = C1(5);
c9 = C1(6);

C2 = (1/I_y) .*  [(I_z - I_x);
                  I_xz;
                  1];

c5 = C2(1);
c6 = C2(2);
c7 = C2(3);

%% Control Allocation
T = K*(ct1 + ct2 + ct3 + ct4);
L_tbar = K * l *(ct1 - ct2 - ct3 + ct4);
M_t = K * l * (ct1 + ct2 - ct3 - ct4);
N_t = (K*R/sqrt(2)) * (ct1^1.5 - ct2^1.5 + ct3^1.5 - ct4^1.5);
%% True Air Speed and Dynamic Pressure
% rho = 1.225;
% V = sqrt(u^2 + v^2 + w^2);
% Q_inf = 0.5*rho*V^2;


% Rotation Matrices
Rot_mat_body2iner_inquat = [n0^2+n1^2-n2^2-n3^2, 2*n1*n2-2*n3*n0, 2*n1*n3 + 2*n2*n0;
                     2*n1*n2+2*n3*n0, n0^2-n1^2+n2^2-n3^2, 2*n2*n3-2*n0*n1;
                     2*n1*n3-2*n2*n0, 2*n2*n3+2*n1*n0, n0^2-n1^2-n2^2+n3^2];

%% Dynamics
% T = k_t*(w1^2 + w2^2 + w3^2 + w4^2);
lin_vel_dot = [0; 0; -T/mass] + [F_ax/mass; F_ay/mass; F_az/mass] +g .* [2*n1*n3 - 2*n0*n2; 2*n2*n3 + 2*n0*n1; n0^2-n1^2-n2^2+n3^2] + [r*v - q*w; p*w - r*u; q*u - p*v];
ang_vel_dot = [(c1*r + c2*p)*q + c3*(L_abar + L_tbar) + c4*(N_a + N_t);
                c5*p*r - c6*(p^2 - r^2) + c7*(M_a + M_t);
                (c8*p - c2*r)*q + c4*(L_abar + L_tbar) + c9*(N_a + N_t)];

%% Kinematics 
pos_dot = Rot_mat_body2iner_inquat*[u; v; w];
n_dot = 0.5 .* [-n1, -n2, -n3;
                n0, -n3, n2;
                n3, n0, -n1;
                -n2, n1, n0] * [p; q; r];

%% Inputs
ct_dot = U;

X_dot = [pos_dot; lin_vel_dot; n_dot; ang_vel_dot; ct_dot];