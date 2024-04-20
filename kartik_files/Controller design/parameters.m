%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Geometric and inertial parameters

m               = 12;                       % Mass in kg
Ixx             = 1.86;                     % Ixx in kgm^2
Iyy             = 2.031;                    % Iyy in kgm^2          
Ixy             = 0;                        % Ixy in kgm^2
Ixz             = 0;                        % Ixz in kgm^2
Iyz             = 0;                        % Iyz in kgm^2
Izz             = 3.617;                    % Izz in kgm^2
I_w             = [Ixx 0 0; 0 Iyy 0; 0 0 Izz];
g               = 9.81;                     % Accel. due to g in m/s^2
b_w             = 2.29;                     % Wing span in m
S_w             = 2*0.754;                  % Wing area in m^2
c_r             = 0.39;                     % Root chord in m
c_t             = 0.176;                    % Tip chord in m
AR              = 6.9;                      % Aspect ratio
GC              = 2.56;                     % Gap to chord ratio
X_cg            = 0.157;                    % CG from the leading edge in m
X_ac            = 0.0838;                   % Aero centre from the leading edge in m
taper           = c_t / c_r;                % Taper ratio
c_mac           = c_r * (2/3) * ((1+taper+taper^2)/(1+taper)); % Mean aero chord
b_w_mac         = S_w / c_mac;

%% Longitudinal and lateral/directional aerodynamics derivatives

CD_0            = 0.009;
CL_0            = 0.4918;
CL_alp          = 4.695;
CL_q            = 0;
Cm_0            = -0.0156;
Cm_alp          = 0.995;
Cm_q            = -0.51;
CY_beta         = -0.951;
CY_p            = 0;
CY_r            = 0.008;
Cl_beta         = 0;
Cl_p            = -0.43;
Cl_r            = 0.29;
Cn_beta         = 0.0812;
Cn_p            = -0.4044;
Cn_r            = -0.05;
alp_0           = 0.05;
M_0             = 1;

%% Rotor specifications

b               = 0.032;                        % Blade chord in m
l               = 0.40;                         % Arm length in m
RPM             = 2000;
omega_w         = 2 * pi * RPM / 60;            % Ang vel in rad/s
R               = 0.355;                        % Blade radius in meters
rho             = 1.225;                        % Air density in kg/m^3
K               = rho * pi * R^2 * (omega_w * R)^2;   % Constant K
V_tip           = omega_w * R;                        % Blade tip velocity in m/s
blade_area      = pi * R^2;                     % Blade area
CT_init         = (0.25*m*g/(rho*blade_area*V_tip^2));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
