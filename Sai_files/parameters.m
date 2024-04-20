 
m               = 12;                       % Mass in kg
Ixx             = 1.86;                     % Ixx in kgm^2
Iyy             = 2.031;                    % Iyy in kgm^2          
Ixy             = 0;                        % Ixy in kgm^2
Ixz             = 0;                        % Ixz in kgm^2
Iyz             = 0;                        % Iyz in kgm^2
Izz             = 3.617;                    % Izz in kgm^2
g               = 9.81;   
b               = 0.032;                        % Blade chord in m
l               = 0.40;                         % Arm length in m
RPM             = 3000;
omega_w         = 2 * pi * RPM / 60;            % Ang vel in rad/s
R               = 0.355;                        % Blade radius in meters
rho             = 1.225;                        % Air density in kg/m^3
K               = rho * pi * R^2 * (omega_w * R)^2;   % Constant K
V_tip           = omega_w * R;                        % Blade tip velocity in m/s
blade_area      = pi * R^2;                     % Blade area
CT_init         = (0.25*m*g/(rho*blade_area*V_tip^2));
I = diag([Ixx, Iyy, Izz]);

zitaxt = 0.95;
wnxt = 5;
%% Gains
zeta_quat = diag([0.95, 0.95, 0.95, 0.95]);
omega_n_quat = diag([25, 25, 25, 25]);

zeta_x = diag([0.95, 0.95, 0.95]);
    omega_n_x = diag([5, 5, 5]);
