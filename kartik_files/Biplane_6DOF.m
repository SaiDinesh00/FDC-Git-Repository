function X_dot = Biplane_6DOF(X, U, U_a)

parameters; 

% U_a represents aerodynamic forces in Fixed wing frame
% U represents thrust cooefficient rates
% X represents states in Quadcopter frame
% X = [x, y, z, eta_0, eta_1, eta_2, eta_3, u, v, w, p, q, r, CT1, CT2, CT3, CT4]

%% States 
% Inertial frame
x = X(1,:);
y = X(2,:);
z = X(3,:);

% Vehicle, Vehicle 1, Vehicle 2 frames
quaternions = X(4:7, :);
% Euler_angles = quat2eul(X(4:7,:),"ZYX");
% phi   = Euler_angles(1);
% theta = Euler_angles(2);
% psi   = Euler_angles(3);

% Linear velocities in Body frame 
u = X(8,:);
v = X(9,:);
w = X(10,:);

% Angular velocities in Body frame
p = X(11,:);
q = X(12,:);
r = X(13,:);

%% Forces and moments due to Thrust
Thrust_coeffs = X(14:17, :);
Thrust_coeffs(Thrust_coeffs > 0.6) = 0.6;
Thrust_coeffs(Thrust_coeffs < 0.000001) = 0.000001;
X(14:17, :) = Thrust_coeffs;

% Thrust Coefficients
CT_1 = X(14, :);
CT_2 = X(15, :);
CT_3 = X(16, :);
CT_4 = X(17, :);

% Control Input

T_t = K * (CT_1 + CT_2 + CT_3 + CT_4);
L_t = -K * R * (1 / sqrt(2)) * (CT_1^(1.5) - CT_2^(1.5) ...
    + CT_3^(1.5) - CT_4^(1.5));
M_t = K * l * (CT_1 + CT_2 - CT_3 - CT_4);
N_t = K * l * (CT_1 - CT_2 - CT_3 + CT_4);

% LMN_d = [L; M; N];
% % Limiter block : Moments and thrust 
% LMN_d(LMN_d >  2.35) =  2.35;
% LMN_d(LMN_d < -2.35) = -2.35;
% T(T > 31) = 31;
% T(T < 0)  =  0;

%% Forces and moments due to aerodynamics in quadcopter frame
F_ax = U_a(1);
F_ay = U_a(2);
F_az = U_a(3);
L_a  = U_a(4);
M_a  = U_a(5);
N_a  = U_a(6);

%% Six degree of the freedom model 

%%%%%%%%%%% Kinematics %%%%%%%%%%%
% Translation 
Quat_rot_mat_B2E = Quaternion_rot_mat(quaternions, 1);
Pos_dot = Quat_rot_mat_B2E*[u; v; w];

% Rotational
Quat_rot_mat2 = Quaternion_rot_mat(quaternions, 2);
Quat_dot = 0.5 * Quat_rot_mat2*[p; q; r];
%%%%%%%%%%% Kinematics %%%%%%%%%%%

%%%%%%%%%%% Dynamics %%%%%%%%%%%%%
% Translation 
Quat_rot_mat_E2B = Quat_rot_mat_B2E^-1;
Vel_dot = [T_t/m; 0; 0] + Quat_rot_mat_E2B*[0; 0; g] ...
           + (1/m) * [F_ax; F_ay; F_az] + [r*v-q*w;p*w-r*u;q*u-p*v];

% Rotation
Att_dot = [ (L_a + L_t)/(Ixx) + ((Iyy - Izz)/(Ixx))*q*r;
            (M_a + M_t)/(Iyy) + ((Izz - Ixx)/(Iyy))*p*r;
            (N_a + N_t)/(Izz) + ((Ixx - Iyy)/(Izz))*p*q; ];
%%%%%%%%%%% Dynamics %%%%%%%%%%%%%    

%% The rates
    X_dot = [Pos_dot
             Quat_dot
             Vel_dot
             Att_dot
             U];